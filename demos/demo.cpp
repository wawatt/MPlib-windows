#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
#include <pybind11/stl.h>
// #include <Eigen/Core>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

#include <pybind11/embed.h>

namespace py = pybind11;

class PythonInterpreterManager {
public:
    // 获取单例实例
    static PythonInterpreterManager& getInstance() {
        static PythonInterpreterManager instance; // 静态实例，确保只初始化一次
        return instance;
    }

private:
    // 构造函数：初始化 Python 解释器
    PythonInterpreterManager() {
        static py::scoped_interpreter guard{}; // 初始化 Python 解释器
    }

    // 禁用拷贝构造和赋值操作
    PythonInterpreterManager(const PythonInterpreterManager&) = delete;
    PythonInterpreterManager& operator=(const PythonInterpreterManager&) = delete;

    // 析构函数：自动销毁 Python 解释器
    ~PythonInterpreterManager() = default;
};

class Planner {
public:
    Planner(const std::string& urdf_filename, const std::string& srdf_filename) {
        // 确保 Python 解释器已初始化
        PythonInterpreterManager::getInstance();

        py::module sys = py::module::import("sys");
        sys.attr("path").attr("append")(R"(E:\Pro\github\MPlib-windows\demos\scripts)");  // 替换为实际路径

        py::module planner_py = py::module::import("planner");

        std::vector<double> joint_vel_limits = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
        std::vector<double> joint_acc_limits = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
        py::object Planner = planner_py.attr("Planner");
        planner_ = Planner(urdf_filename, "panda_hand", 
          py::arg("srdf") = srdf_filename,
          py::arg("joint_vel_limits") = joint_vel_limits,
          py::arg("joint_acc_limits") = joint_acc_limits,
          py::arg("verbose") = true
        );
    }
    // Destructor
    ~Planner() {
        // Finalize the Python interpreter
        planner_ = py::none();
    }

    void checkSelfCollision(const std::vector<double>& state={})
    {
      py::list res_list;
      if (state.empty()) {
        res_list = planner_.attr("check_for_self_collision")().cast<py::list>();
      }
      else {
        py::array_t<double> state_array = py::array_t<double>(state.size(), state.data());
        res_list = planner_.attr("check_for_self_collision")(state_array).cast<py::list>();
      }
      for (auto res : res_list) {
        std::cout << "link_name1: " << res.attr("link_name1").cast<std::string>() << std::endl;
        std::cout << "link_name2: " << res.attr("link_name2").cast<std::string>() << std::endl;
      }
    }

    void FK(const std::vector<double>& qpos)
    {
        py::array_t<double> qpos_array = py::array_t<double>(qpos.size(), qpos.data());

        auto out = planner_.attr("FK")(qpos_array);
        // // Convert to std::vector<double> and print
        // std::vector<double> result = out.cast<std::vector<double>>();
        // std::cout << "FK result: ";
        // for (double val : result) {
        //     std::cout << val << " ";
        // }
        // std::cout << std::endl;
    }
    void IK(
        const std::vector<double>& goal_pose, 
        const std::vector<double>& start_qpos,
        const std::vector<bool>& mask={},
        int n_init_qpos = 20,
        double threshold = 1e-3,
        bool return_closest = false,
        bool verbose = false)
    {
        py::array_t<double> goal_pose_array = py::array_t<double>(goal_pose.size(), goal_pose.data());
        py::array_t<double> start_qpos_array = py::array_t<double>(start_qpos.size(), start_qpos.data());

        py::object out = planner_.attr("IK")(
          goal_pose_array, start_qpos_array, 
          mask.empty() ? py::none() : py::cast(mask),
          n_init_qpos, threshold, return_closest, verbose);
        // tuple[str, Union[list[np.ndarray], np.ndarray, None]]
        auto result = py::cast<std::tuple<std::string, py::object>>(out);
        std::string status = std::get<0>(result); // Success
        py::object qpos = std::get<1>(result);
        if (qpos.is_none()) {
            std::cout << "No solution found." << std::endl;
        } else if (py::isinstance<py::list>(qpos)) {
            std::cout << "Multiple solutions found:" << std::endl;
            for (auto q : qpos.cast<py::list>()) {
                // Convert to std::vector<double> and print
                std::vector<double> q_vector = q.cast<std::vector<double>>();
                std::cout << "Solution: ";
                for (double val : q_vector) {
                    std::cout << val << " ";
                }
                std::cout << std::endl;
            }
        } else {
            // Convert to std::vector<double> and print
            std::vector<double> q_vector = qpos.cast<std::vector<double>>();
            std::cout << "Single solution found: ";
            for (double val : q_vector) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
    }

private:
  py::object planner_;
};

int main() {

  // set up an articulated model
  std::string urdf_filename = "E:/Pro/github/MPlib-windows/data/panda/panda.urdf";
  std::string srdf_filename = "E:/Pro/github/MPlib-windows/data/panda/panda.srdf";


try{
    Planner planner(urdf_filename, srdf_filename);
    std::vector<double> state={0,0,0,-3.069,0,1.77,0};
    planner.checkSelfCollision(state);
    planner.FK(state);
    // planner.checkSelfCollision(state);
  }
  catch (const py::error_already_set& e) {
      std::cerr << "Python error: " << e.what() << std::endl;
  }

    printf("done!\n");
    return 0;
}
