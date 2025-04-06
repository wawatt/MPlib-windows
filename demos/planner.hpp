#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
#include <Eigen/Eigen>

#include <mplib/planning_world.h>
#include <mplib/core/articulated_model.h>
#include <mplib/planning/ompl/ompl_planner.h>
#include <mplib/kinematics/pinocchio/pinocchio_model.h>
#include <mplib/planning/ompl/fixed_joint.h>
#include <mplib/collision_detection/fcl/types.h>

#include <toppra/toppra.hpp>
#include <torch/torch.h>

class Planner {
public:
    Planner(
        const std::string& urdf_path,
        const std::string& srdf_path,
        const std::string& move_group="tool0",
        bool use_convex = false,
        const std::vector<std::string>& user_link_names = {},
        const std::vector<std::string>& user_joint_names = {},
        const std::vector<double>& joint_vel_limits = {}, // Use Tensor
        const std::vector<double>& joint_acc_limits = {}, // Use Tensor
        // const std::vector</*FCLObject placeholder*/>& objects = {}, // Needs actual FCL object type
        bool verbose = true) {

        robot_ = std::make_shared<mplib::ArticulatedModeld>(
            urdf_path,
            srdf_path, 
            std::string_view {}, Eigen::Vector3d(0, 0, -9.81),
            user_link_names,    user_joint_names,
            use_convex,verbose
        );

        pinocchio_model_ = robot_->getPinocchioModel();
        user_link_names_ = pinocchio_model_->getLinkNames();
        user_joint_names_ = pinocchio_model_->getJointNames();

        for (int i = 0; i < user_joint_names.size(); ++i)
            joint_name_2_idx_[user_joint_names[i]] = i;
        for (int i = 0; i < user_link_names.size(); ++i)
            link_name_2_idx_[user_link_names[i]] = i;

        // TODO: assert
        move_group_ = move_group;
        robot_->setMoveGroup(move_group);
        move_group_joint_indices_ = robot_->getMoveGroupJointIndices();
        move_group_joint_indices_t_ = torch::from_blob(
            const_cast<size_t*>(move_group_joint_indices_.data()), 
            {static_cast<int64_t>(move_group_joint_indices_.size())}, 
            torch::kInt64
        ).clone();

        joint_types_ = pinocchio_model_->getJointTypes();
        auto joint_limits = pinocchio_model_->getJointLimits(); 
        joint_limits_eigen_.resize(joint_limits.size(), 2); // Use Eigen matrix for easier access
        joint_limits_ = torch::zeros({(int)joint_limits.size(), 2}, torch::kDouble);
        for (size_t i = 0; i < joint_limits.size(); ++i) {
            joint_limits_[i][0] = joint_limits[i](0, 0);
            joint_limits_[i][1] = joint_limits[i](0, 1);
            joint_limits_eigen_(i, 0) = joint_limits[i](0, 0);
            joint_limits_eigen_(i, 1) = joint_limits[i](1, 0); // Correct indexing for Eigen::Matrix<double, 1, 2>
        }

        if (joint_vel_limits.empty()) 
            joint_vel_limits_ = torch::ones({(int)move_group_joint_indices_.size()}, torch::kDouble);
        if (joint_acc_limits.empty())
            joint_acc_limits_ = torch::ones({(int)move_group_joint_indices_.size()}, torch::kDouble);
        move_group_link_id_ = link_name_2_idx_[move_group];

        // TODO:ASSERT

        // # Mask for joints that have equivalent values (revolute joints with range > 2pi)
        std::vector<bool> equiv_joint_mask;
        for (size_t i = 0; i < joint_types_.size(); ++i) {
            // Assuming joint_types_ contains strings like "JointModelR"
            if( joint_types_[i].find("JointModelR") != std::string::npos) // Adjust as per actual types
                equiv_joint_mask.push_back(true);
            else
                equiv_joint_mask.push_back(false);
        }
        std::cout << "Joint Types: " << joint_types_.size() << std::endl;
        std::cout << "joint_limits: " << joint_limits.size() << std::endl;
        equiv_joint_mask_ = torch::zeros({joint_limits_.size(0)}, torch::kBool);
        for (size_t i = 0; i < joint_types_.size(); ++i) {
        //     // Assuming joint_types_ contains strings like "JointModelR"
            torch::Tensor is_revolute = torch::tensor({
                joint_types_[i].find("JointModelR") != std::string::npos // Adjust as per actual types
            }, torch::kBool); // 转换为 torch::Tensor
            equiv_joint_mask_[i] = torch::logical_and(
                is_revolute,
                (joint_limits_[i][1] - joint_limits_[i][0]) > 2.0 * M_PI
            ); // Add tolerance
        } //     double range = joint_limits_[i][1] - joint_limits_[i][0];

        planning_world_ = std::make_shared<mplib::PlanningWorldd>(
            std::vector<mplib::ArticulatedModeldPtr>({robot_})/*, objects*/
        );
        acm_ = planning_world_->getAllowedCollisionMatrix();

        planner_ = std::make_shared<mplib::planning::ompl::OMPLPlannerTpld>(planning_world_);

    }

    bool Planner::wrapJointLimit(Eigen::VectorXd& qpos) {
        // Tolerance for floating point comparisons
        const double tolerance = 1e-3;

        for (int i = 0; i < joint_types_.size(); ++i) {

            double q = qpos(i); // Get current joint value
            const std::string& joint_type = joint_types_[i];
    
            // Use .item<double>() for accessing single values from the limits tensor
            double q_min = joint_limits_[i][0].item<double>();
            double q_max = joint_limits_[i][1].item<double>();

            // Check if it's a revolute joint (adjust string check if needed)
            if (joint_type.find("JointModelR") != std::string::npos) {
                // Handle small negative tolerance near lower bound: [-tol, 0) relative to q_min
                if (q - q_min >= -tolerance && q - q_min < 0) {
                    continue; // Consider it within limits
                }

                // Wrap the angle q into the range [q_min, q_min + 2*pi)
                // Equivalent to Python: q -= 2 * np.pi * np.floor((q - q_min) / (2 * np.pi))
                q = q - 2.0 * M_PI * std::floor((q - q_min) / (2.0 * M_PI));

                // Update the tensor element
                qpos(i) = q;

                // Check if the wrapped angle exceeds the upper limit (with tolerance)
                if (q > q_max + tolerance) {
                    // std::cerr << "Warning: Revolute joint " << i << " (" << user_joint_names_[i]
                    //           << ") wrapped value " << q << " exceeds limit " << q_max << std::endl;
                    return false;
                }

            } else { // Non-revolute joint (e.g., prismatic)
                // Check if the value is outside the limits (with tolerance)
                if (q < q_min - tolerance || q > q_max + tolerance) {
                    // std::cerr << "Warning: Non-revolute joint " << i << " (" << user_joint_names_[i]
                    //          << ") value " << q << " is outside limits [" << q_min << ", " << q_max << "]" << std::endl;
                    return false;
                }
            }
        }

        // If the loop completes without returning false, the configuration is valid
        return true;
    }

    Eigen::VectorXd Planner::padMoveGroupQpos(const Eigen::VectorXd& qpos 
        /*mplib::ArticulatedModeldPtr& articulation=nullptr*/) {

        // if (articulation == nullptr)
        //     articulation = robot_;
        auto qpos_new = qpos;
        size_t ndim = qpos.size();
        if (ndim == robot_->getQposDim()) {
            auto tmp = robot_->getQpos();
            tmp.head(ndim) = qpos;
            qpos_new = tmp;
   
        }        
        return qpos_new;
    }

    struct ToppResult {
        std::string status = "Failure"; // Default to failure
        std::vector<double> time_steps;
        std::vector<Eigen::VectorXd> positions;
        std::vector<Eigen::VectorXd> velocities;
        std::vector<Eigen::VectorXd> accelerations;
        double duration = 0.0;
    };
    ToppResult TOPP(const std::vector<Eigen::VectorXd>& path, // Input path (sequence of joint states)
        double step = 0.1,
        bool verbose = false,
        std::optional<double> duration = std::nullopt)
    {
        ToppResult result;
        if (path.empty()) {
            result.status = "Input path is empty.";
            return result;
        }

        size_t n_samples = path.size();
        int dof = path[0].size();
        auto ss = torch::linspace(0, 1, N_samples);

        path = Toppra::GeometricPath::SplineInterpolator (ss, path)
    }

    void updatePointCloud(const Eigen::Matrix<double, Eigen::Dynamic, 3>& points, 
        double resolution=1e-3, const std::string& name="scene_pcd") {
        planning_world_->addPointCloud(name, points, resolution);
    }

    bool removePointCloud(const std::string& name="scene_pcd") {
        return planning_world_->removeObject(name);
    }

    void updateAttachedObject(
        const fcl::CollisionGeometryPtr<double>& collision_geometry,
        const mplib::Pose<double>& pose,
        const std::string& name="attached_geom",
        const std::string& art_name="",
        const int& link_id=-1)
    {
        auto link_id_TMP = link_id;
        if (link_id == -1)
            link_id_TMP = move_group_link_id_;
        auto art_name_TMP = art_name;
        if (art_name == "")
            art_name_TMP = robot_->getName();
        planning_world_->attachObject(
            name, collision_geometry,
            art_name_TMP, link_id_TMP, pose);
    }

    void update_attached_sphere(const double& radius, 
        const mplib::Pose<double>& pose,
        const std::string& art_name="",
        const int& link_id=-1)
    {
        // """
        // Attach a sphere to some link

        // :param radius: radius of the sphere
        // :param pose: attaching pose (relative pose from attached link to object)
        // :param art_name: name of the articulated object to attach to.
        //                  If None, attach to self.robot.
        // :param link_id: if not provided, the end effector will be the target.
        // """
        auto link_id_TMP = link_id;
        if (link_id == -1) 
            link_id_TMP = move_group_link_id_;
        auto art_name_TMP = art_name;
        if (art_name == "")
            art_name_TMP = robot_->getName();

        planning_world_->attachSphere(
            radius, art_name_TMP,
            link_id_TMP, pose
        );
    }

    void updateAttachedBox(const Eigen::Vector3d& size,
        const mplib::Pose<double>& pose,
        const std::string& art_name="",
        int link_id=-1)
    {
        // # """
        // # Attach a box to some link

        // # :param size: box side length
        // # :param pose: attaching pose (relative pose from attached link to object)
        // # :param art_name: name of the articulated object to attach to.
        // #                  If None, attach to self.robot.
        // # :param link_id: if not provided, the end effector will be the target.
        // # """
        auto link_id_TMP = link_id;
        if (link_id == -1) 
            link_id_TMP = move_group_link_id_;
        auto art_name_TMP = art_name;
        if (art_name == "")
            art_name_TMP = robot_->getName();
        planning_world_->attachBox(
            size, art_name_TMP,
            link_id_TMP, pose
        );
    }

    void updateAttachedMesh(const std::string& mesh_path,
        const mplib::Pose<double>& pose,
        bool convex=false,
        const std::string& art_name="",
        const int& link_id=-1)
    {
        // """
        // Attach a mesh to some link

        // :param mesh_path: path to a mesh file
        // :param pose: attaching pose (relative pose from attached link to object)
        // :param art_name: name of the articulated object to attach to.
        //                  If None, attach to self.robot.
        // :param link_id: if not provided, the end effector will be the target.
        // """
        auto link_id_TMP = link_id;
        if (link_id == -1) 
            link_id_TMP = move_group_link_id_;
        auto art_name_TMP = art_name;
        if (art_name == "")
            art_name_TMP = robot_->getName();
        
        planning_world_->attachMesh(
            mesh_path, art_name_TMP,
            link_id_TMP, pose, convex
        );
    }

    bool detachObject(const std::string& name="attached_geom",
        bool also_remove=false)
    {
        // """
        // Detact the attached object with given name

        // :param name: object name to detach
        // :param also_remove: whether to also remove object from world
        // :return: ``True`` if success, ``False`` if the object with given name is not  attached
        // """
        return planning_world_->detachObject(name, also_remove);
    }

    void set_base_pose(const mplib::Pose<double>& pose) {
        // """
        // tell the planner where the base of the robot is w.r.t the world frame

        // Args:
        //     pose: pose of the base
        // """
        robot_->setBasePose(pose);
    }

    bool removeobject(const std::string& name) {
        // # """
        // # remove the object with given name

        // # :param name: object name to remove
        // # :return: ``True`` if success, ``False`` if the object with given name is not found
        // # """
        return planning_world_->removeObject(name);
    }

    struct PlanResult {
        std::string status;
        std::vector<Eigen::VectorXd> path; // OMPL path output
        // Omit TOPP related fields for now:
        // std::vector<double> time;
        // std::vector<Eigen::VectorXd> position;
        // std::vector<Eigen::VectorXd> velocity;
        // std::vector<Eigen::VectorXd> acceleration;
        // double duration;
    };
    PlanResult planQpos(
        const std::vector<Eigen::VectorXd>& goal_qposes, // Use vector of Eigen vectors
        const Eigen::VectorXd& current_qpos,
        float time_step=0.1f,
        float rrt_range=0.1f,
        float planning_time=1.0f,
        bool fix_joint_limits = true,
        const std::vector<size_t>& fixed_joint_indices = {},
        bool simplify = true,
        // Use the actual types from OMPLPlannerTpl
        const std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)> &constraint_function = nullptr,
        const std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)> &constraint_jacobian = nullptr,
        double constraint_tolerance = 1e-3,
        bool verbose = false
        // time_step removed as TOPP is omitted
    ) {
        PlanResult result;
        auto current_qpos_tmp = current_qpos;
        torch::Tensor current_qpos_torch = torch::from_blob(
            current_qpos_tmp.data(), // 数据指针
            {current_qpos_tmp.size()}, // 张量的形状
            torch::kFloat64 // 数据类型
        ); // 使用 clone() 确保张量拥有自己的内存

        if (fix_joint_limits) {
            current_qpos_torch = torch::clip(
                current_qpos_torch, 
                joint_limits_.index({torch::indexing::Slice(), 0}),
                joint_limits_.index({torch::indexing::Slice(), 1})
            ).clone();
        }

        // Ensure current_qpos has the full DOF for state setting and collision checks
        Eigen::VectorXd current_qpos_eigen = Eigen::Map<Eigen::VectorXd>(current_qpos_torch.data_ptr<double>(), current_qpos_torch.size(0));
        current_qpos_eigen = padMoveGroupQpos(current_qpos_eigen);
        current_qpos_torch = torch::from_blob(
            current_qpos_tmp.data(), // 数据指针
            {current_qpos_tmp.size()}, // 张量的形状
            torch::kFloat64 // 数据类型
        );

        // Set current state and check for collisions
        robot_->setQpos(current_qpos_eigen, true); // Update internal state
        auto collisions = planning_world_->checkCollision();
        if (!collisions.empty()) {
            result.status = "Invalid start state due to collision: ";
             if (verbose) {
                 std::cerr << "Warning: Invalid start state!" << std::endl;
                for (const auto& collision : collisions) {
                     std::cerr << "Collision between " << collision.link_name1 << " of entity "
                              << collision.object_name1 << " with " << collision.link_name2
                              << " of entity " << collision.object_name2 << std::endl;
                     result.status += collision.link_name1 + " and " + collision.link_name2 + "; ";
                }
            }
            return result; // Early exit
        }

        std::vector<Eigen::VectorXd> goal_qposes_;
        for (const auto& goal_qpos : goal_qposes) {
            auto goal_qpos_t = torch::from_blob(
                const_cast<double*>(goal_qpos.data()), // 数据指针
                {goal_qpos.size()},                   // 张量形状
                torch::kDouble                        // 数据类型
            ).clone(); // 使用 clone() 确保张量拥有自己的内存
            goal_qpos_t = goal_qpos_t[move_group_joint_indices_t_];
            goal_qposes_.push_back(Eigen::Map<Eigen::VectorXd>(
                goal_qpos_t.data_ptr<double>(), goal_qpos_t.size(0)));
        }

        // Prepare fixed joints
        mplib::planning::ompl::FixedJointsTpl<double> fixed_joints;
        for (size_t joint_idx : fixed_joint_indices) {
            if (joint_idx < current_qpos_eigen.size()) { // Check bounds
                // The FixedJoint constructor needs the articulation index (0 for the first robot)
                fixed_joints.insert(mplib::planning::ompl::FixedJointTpl<double>(0, joint_idx, current_qpos_eigen[joint_idx]));
            } else {
                std::cerr << "Warning: Fixed joint index " << joint_idx << " out of bounds." << std::endl;
            }
        }
    
        // assert len(current_qpos[move_joint_idx]) == len(goal_qpos_[0])

        auto plan_current_qpos = current_qpos_torch[move_group_joint_indices_t_];

        // Perform planning
        auto [status, path] = planner_->plan(
            Eigen::Map<Eigen::VectorXd>(plan_current_qpos.data_ptr<double>(), plan_current_qpos.size(0)),
            goal_qposes_, // Pass vector of goals
            planning_time,
            rrt_range,
            fixed_joints, // Pass the set of fixed joints
            simplify,
            constraint_function,
            constraint_jacobian,
            constraint_tolerance,
            verbose
        );

        // if (status == "Exact solution")
        //     if (verbose)
        //         ta.setup_logging("INFO");
        //     else
        //         ta.setup_logging("WARNING");
        //     times, pos, vel, acc, duration = self.TOPP(path, time_step)
        //     return {
        //         "status": "Success",
        //         "time": times,
        //         "position": pos,
        //         "velocity": vel,
        //         "acceleration": acc,
        //         "duration": duration,
        //     }
        // else:
        //     return {"status": f"RRTConnect Failed. {status}"}
      
        return result;
    }









    
   

private:
    // --- Member Variables ---
    mplib::ArticulatedModeldPtr robot_;
    mplib::kinematics::pinocchio::PinocchioModeldPtr pinocchio_model_;
    std::vector<std::string> user_link_names_;
    std::vector<std::string> user_joint_names_;
    
    // Maps for names to indices
    std::unordered_map<std::string, int>  joint_name_2_idx_;
    std::unordered_map<std::string, int>  link_name_2_idx_;

    std::string move_group_;
    std::vector<size_t> move_group_joint_indices_;
    torch::Tensor move_group_joint_indices_t_;
    std::vector<std::string> joint_types_; // Joint types (e.g., revolute, prismatic)

    torch::Tensor joint_limits_;     // Combined limits [n_joints, 2]
    Eigen::MatrixX2d joint_limits_eigen_;
    torch::Tensor joint_vel_limits_; // For move group joints 1d
    torch::Tensor joint_acc_limits_; // For move group joints 1d

    size_t move_group_link_id_; // ID of the move group link
    torch::Tensor equiv_joint_mask_; // bool

    mplib::PlanningWorlddPtr planning_world_;
    std::shared_ptr<mplib::collision_detection::AllowedCollisionMatrix> acm_;
    mplib::planning::ompl::OMPLPlannerTpldPtr planner_;


};