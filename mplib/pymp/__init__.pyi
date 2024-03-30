"""
Motion planning python binding
"""

import typing

import numpy

from . import collision_detection, kinematics, planning

__all__ = [
    "ArticulatedModel",
    "PlanningWorld",
    "collision_detection",
    "kinematics",
    "planning",
    "set_global_seed",
]

class ArticulatedModel:
    """
    Supports initialization from URDF and SRDF files, and provides access to
    underlying Pinocchio and FCL models.
    """
    def __init__(
        self,
        urdf_filename: str,
        srdf_filename: str,
        gravity: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        link_names: list[str],
        joint_names: list[str],
        convex: bool = False,
        verbose: bool = False,
    ) -> None:
        """
        Construct an articulated model from URDF and SRDF files.

        :param urdf_filename: path to URDF file, can be relative to the current working
            directory
        :param srdf_filename: path to SRDF file, we use it to disable self-collisions
        :param gravity: gravity vector
        :param link_names: list of links that are considered for planning
        :param joint_names: list of joints that are considered for planning
        :param convex: use convex decomposition for collision objects. Default:
            ``False``.
        :param verbose: print debug information. Default: ``False``.
        """
    def get_base_pose(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]:
        """
        Get the base pose of the robot.

        :return: base pose of the robot in [x, y, z, qw, qx, qy, qz] format
        """
    def get_fcl_model(self) -> collision_detection.fcl.FCLModel:
        """
        Get the underlying FCL model.

        :return: FCL model used for collision checking
        """
    def get_move_group_end_effectors(self) -> list[str]:
        """
        Get the end effectors of the move group.

        :return: list of end effectors of the move group
        """
    def get_move_group_joint_indices(self) -> list[int]:
        """
        Get the joint indices of the move group.

        :return: list of user joint indices of the move group
        """
    def get_move_group_joint_names(self) -> list[str]:
        """
        Get the joint names of the move group.

        :return: list of joint names of the move group
        """
    def get_move_group_qpos_dim(self) -> int:
        """
        Get the dimension of the move group qpos.

        :return: dimension of the move group qpos
        """
    def get_name(self) -> str:
        """
        Get name of the articulated model.

        :return: name of the articulated model
        """
    def get_pinocchio_model(self) -> kinematics.pinocchio.PinocchioModel:
        """
        Get the underlying Pinocchio model.

        :return: Pinocchio model used for kinematics and dynamics computations
        """
    def get_qpos(
        self,
    ) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Get the current joint position of all active joints inside the URDF.

        :return: current qpos of all active joints
        """
    def get_user_joint_names(self) -> list[str]:
        """
        Get the joint names that the user has provided for planning.

        :return: list of joint names of the user
        """
    def get_user_link_names(self) -> list[str]:
        """
        Get the link names that the user has provided for planning.

        :return: list of link names of the user
        """
    def set_base_pose(
        self,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Set the base pose of the robot.

        :param pose: base pose of the robot in [x, y, z, qw, qx, qy, qz] format
        """
    @typing.overload
    def set_move_group(self, end_effector: str) -> None:
        """
        Set the move group, i.e. the chain ending in end effector for which to compute
        the forward kinematics for all subsequent queries.

        :param end_effector: name of the end effector link
        """
    @typing.overload
    def set_move_group(self, end_effectors: list[str]) -> None:
        """
        Set the move group but we have multiple end effectors in a chain. I.e., Base -->
        EE1 --> EE2 --> ... --> EEn

        :param end_effectors: list of links extending to the end effector
        """
    def set_name(self, name: str) -> None:
        """
        Set name of the articulated model.

        @param: name of the articulated model
        """
    def set_qpos(
        self,
        qpos: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        full: bool = False,
    ) -> None:
        """
        Let the planner know the current joint positions.

        :param qpos: current qpos of all active joints or just the move group joints
        :param full: whether to set the full qpos or just the move group qpos. If full
            is ``False``, we will pad the missing joints with current known qpos. The
            default is ``False``
        """
    def update_SRDF(self, SRDF: str) -> None:
        """
        Update the SRDF file to disable self-collisions.

        :param srdf: path to SRDF file, can be relative to the current working directory
        """

class PlanningWorld:
    """
    Planning world for collision checking

    Mimicking MoveIt2's ``planning_scene::PlanningScene``,
    ``collision_detection::World``, ``moveit::core::RobotState``

    https://moveit.picknik.ai/main/api/html/classplanning__scene_1_1PlanningScene.html
    https://moveit.picknik.ai/main/api/html/classcollision__detection_1_1World.html
    https://moveit.picknik.ai/main/api/html/classmoveit_1_1core_1_1RobotState.html
    """
    def __init__(
        self,
        articulations: list[ArticulatedModel],
        articulation_names: list[str],
        normal_objects: list[collision_detection.fcl.CollisionObject] = [],
        normal_object_names: list[str] = [],
    ) -> None:
        """
        Constructs a PlanningWorld with given (planned) articulations and normal objects

        :param articulations: list of planned articulated models
        :param articulation_names: name of the articulated models
        :param normal_objects: list of collision objects that are not articulated
        :param normal_object_names: name of the normal objects
        """
    def add_articulation(
        self, name: str, model: ArticulatedModel, planned: bool = False
    ) -> None:
        """
        Adds an articulation (ArticulatedModelPtr) with given name to world

        :param name: name of the articulated model
        :param model: articulated model to be added
        :param planned: whether the articulation is being planned
        """
    def add_normal_object(
        self, name: str, collision_object: collision_detection.fcl.CollisionObject
    ) -> None:
        """
        Adds a normal object (CollisionObjectPtr) with given name to world

        :param name: name of the collision object
        :param collision_object: collision object to be added
        """
    def add_point_cloud(
        self,
        name: str,
        vertices: numpy.ndarray[
            tuple[M, typing.Literal[3]], numpy.dtype[numpy.float64]
        ],
        resolution: float = 0.01,
    ) -> None:
        """
        Adds a point cloud as a collision object with given name to world

        :param name: name of the point cloud collision object
        :param vertices: point cloud vertices matrix
        :param resolution: resolution of the point OcTree
        """
    def attach_box(
        self,
        size: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        art_name: str,
        link_id: int,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Attaches given box to specified link of articulation (auto touch_links)

        :param size: box side length
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        """
    def attach_mesh(
        self,
        mesh_path: str,
        art_name: str,
        link_id: int,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Attaches given mesh to specified link of articulation (auto touch_links)

        :param mesh_path: path to a mesh file
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        """
    @typing.overload
    def attach_object(
        self,
        name: str,
        art_name: str,
        link_id: int,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        touch_links: list[str],
    ) -> None:
        """
        Attaches existing normal object to specified link of articulation. If the object
        is currently attached, disallow collision between the object and previous
        touch_links. Updates acm_ to allow collisions between attached object and
        touch_links.

        :param name: normal object name to attach
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        :param touch_links: link names that the attached object touches
        :raises ValueError: if normal object with given name does not exist or if
            planned articulation with given name does not exist
        """
    @typing.overload
    def attach_object(
        self,
        name: str,
        art_name: str,
        link_id: int,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Attaches existing normal object to specified link of articulation. If the object
        is not currently attached, automatically sets touch_links as the name of self
        links that collide with the object in the current state. Updates acm_ to allow
        collisions between attached object and touch_links. If the object is already
        attached, the touch_links of the attached object is preserved and acm_ remains
        unchanged.

        :param name: normal object name to attach
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        :raises ValueError: if normal object with given name does not exist or if
            planned articulation with given name does not exist
        """
    @typing.overload
    def attach_object(
        self,
        name: str,
        p_geom: collision_detection.fcl.CollisionGeometry,
        art_name: str,
        link_id: int,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        touch_links: list[str],
    ) -> None:
        """
        Attaches given object (w/ p_geom) to specified link of articulation. This is
        done by removing normal object and then adding and attaching object. As a
        result, all previous acm_ entries with the object are removed

        :param name: normal object name to attach
        :param p_geom: pointer to a CollisionGeometry object
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        :param touch_links: link names that the attached object touches
        """
    @typing.overload
    def attach_object(
        self,
        name: str,
        p_geom: collision_detection.fcl.CollisionGeometry,
        art_name: str,
        link_id: int,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Attaches given object (w/ p_geom) to specified link of articulation. This is
        done by removing normal object and then adding and attaching object. As a
        result, all previous acm_ entries with the object are removed. Automatically
        sets touch_links as the name of self links that collide with the object in the
        current state (auto touch_links).

        :param name: normal object name to attach
        :param p_geom: pointer to a CollisionGeometry object
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        """
    def attach_sphere(
        self,
        radius: float,
        art_name: str,
        link_id: int,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Attaches given sphere to specified link of articulation (auto touch_links)

        :param radius: sphere radius
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        """
    def collide(self, request: collision_detection.fcl.CollisionRequest = ...) -> bool:
        """
        Check full collision and return only a boolean indicating collision

        :param request: collision request params.
        :return: ``True`` if collision exists
        """
    def collide_full(
        self, request: collision_detection.fcl.CollisionRequest = ...
    ) -> list[collision_detection.WorldCollisionResult]:
        """
        Check full collision (calls selfCollide() and collideWithOthers())

        :param request: collision request params.
        :return: List of WorldCollisionResult objects
        """
    def collide_with_others(
        self, request: collision_detection.fcl.CollisionRequest = ...
    ) -> list[collision_detection.WorldCollisionResult]:
        """
        Check collision with other scene bodies (planned articulations with attached
        objects collide against unplanned articulations and scene objects)

        :param request: collision request params.
        :return: List of WorldCollisionResult objects
        """
    def detach_object(self, name: str, also_remove: bool = False) -> bool:
        """
        Detaches object with given name. Updates acm_ to disallow collision between the
        object and touch_links.

        :param name: normal object name to detach
        :param also_remove: whether to also remove object from world
        :return: ``True`` if success, ``False`` if the object with given name is not
            attached
        """
    def distance(self, request: collision_detection.fcl.DistanceRequest = ...) -> float:
        """
        Returns the minimum distance-to-collision in current state

        :param request: distance request params.
        :return: minimum distance-to-collision
        """
    def distance_full(
        self, request: collision_detection.fcl.DistanceRequest = ...
    ) -> collision_detection.WorldDistanceResult:
        """
        Compute the min distance to collision (calls distanceSelf() and
        distanceOthers())

        :param request: distance request params.
        :return: a WorldDistanceResult object
        """
    def distance_with_others(
        self, request: collision_detection.fcl.DistanceRequest = ...
    ) -> collision_detection.WorldDistanceResult:
        """
        Compute the min distance between a robot and the world

        :param request: distance request params.
        :return: a WorldDistanceResult object
        """
    def get_allowed_collision_matrix(
        self,
    ) -> collision_detection.AllowedCollisionMatrix:
        """
        Get the current allowed collision matrix
        """
    def get_articulation(self, name: str) -> ArticulatedModel:
        """
        Gets the articulation (ArticulatedModelPtr) with given name

        :param name: name of the articulated model
        :return: the articulated model with given name or ``None`` if not found.
        """
    def get_articulation_names(self) -> list[str]:
        """
        Gets names of all articulations in world (unordered)
        """
    def get_attached_object(self, name: str) -> ...:
        """
        Gets the attached body (AttachedBodyPtr) with given name

        :param name: name of the attached body
        :return: the attached body with given name or ``None`` if not found.
        """
    def get_normal_object(self, name: str) -> collision_detection.fcl.CollisionObject:
        """
        Gets the normal object (CollisionObjectPtr) with given name

        :param name: name of the normal object
        :return: the normal object with given name or ``None`` if not found.
        """
    def get_normal_object_names(self) -> list[str]:
        """
        Gets names of all normal objects in world (unordered)
        """
    def get_planned_articulations(self) -> list[ArticulatedModel]:
        """
        Gets all planned articulations (ArticulatedModelPtr)
        """
    def has_articulation(self, name: str) -> bool:
        """
        Check whether the articulation with given name exists

        :param name: name of the articulated model
        :return: ``True`` if exists, ``False`` otherwise.
        """
    def has_normal_object(self, name: str) -> bool:
        """
        Check whether the normal object with given name exists

        :param name: name of the normal object
        :return: ``True`` if exists, ``False`` otherwise.
        """
    def is_articulation_planned(self, name: str) -> bool:
        """
        Check whether the articulation with given name is being planned

        :param name: name of the articulated model
        :return: ``True`` if exists, ``False`` otherwise.
        """
    def is_normal_object_attached(self, name: str) -> bool:
        """
        Check whether normal object with given name is attached

        :param name: name of the normal object
        :return: ``True`` if it is attached, ``False`` otherwise.
        """
    def print_attached_body_pose(self) -> None:
        """
        Prints global pose of all attached bodies
        """
    def remove_articulation(self, name: str) -> bool:
        """
        Removes the articulation with given name if exists. Updates acm_

        :param name: name of the articulated model
        :return: ``True`` if success, ``False`` if articulation with given name does not
            exist
        """
    def remove_normal_object(self, name: str) -> bool:
        """
        Removes (and detaches) the collision object with given name if exists. Updates
        acm_

        :param name: name of the non-articulated collision object
        :return: ``True`` if success, ``False`` if normal object with given name does
            not exist
        """
    def self_collide(
        self, request: collision_detection.fcl.CollisionRequest = ...
    ) -> list[collision_detection.WorldCollisionResult]:
        """
        Check self collision (including planned articulation self-collision, planned
        articulation-attach collision, attach-attach collision)

        :param request: collision request params.
        :return: List of WorldCollisionResult objects
        """
    def self_distance(
        self, request: collision_detection.fcl.DistanceRequest = ...
    ) -> collision_detection.WorldDistanceResult:
        """
        Get the min distance to self-collision given the robot in current state

        :param request: distance request params.
        :return: a WorldDistanceResult object
        """
    def set_articulation_planned(self, name: str, planned: bool) -> None:
        """
        Sets articulation with given name as being planned

        :param name: name of the articulated model
        :param planned: whether the articulation is being planned
        :raises ValueError: if the articulation with given name does not exist
        """
    def set_qpos(
        self,
        name: str,
        qpos: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
    ) -> None:
        """
        Set qpos of articulation with given name

        :param name: name of the articulated model
        :param qpos: joint angles of the *movegroup only* // FIXME: double check
        """
    def set_qpos_all(
        self,
        state: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
    ) -> None:
        """
        Set qpos of all planned articulations
        """

def set_global_seed(seed: int) -> None:
    """
    Sets the global seed for MPlib (``std::srand()``, OMPL's RNG, and FCL's RNG).

    :param seed: the random seed value
    """
