from pinocchio.robot_wrapper import RobotWrapper
import pinocchio as pin
import example_robot_data as robex
import numpy as np 

class Robot(RobotWrapper):
    """_Class to load a given urdf_

    Args:
        RobotWrapper (_type_): _description_
    """
    def __init__(self,robot_urdf,package_dirs,isFext=False,freeflyer_ori =None,):
        """_Init of the robot class. User can choose between floating base or not and to set the transformation matrix for this floating base._

        Args:
            robot_urdf (_str_): _path to the robot urdf_
            package_dirs (_str_): _path to the meshes_
            isFext (bool, optional): _Adds a floating base if set to True_. Defaults to False.
            freeflyer_ori (_array_, optional): _Orientation of the floating base, given as a rotation matrix_. Defaults to None.
        """

        # intrinsic dynamic parameter names
        self.params_name = (
            "Ixx",
            "Ixy",
            "Ixz",
            "Iyy",
            "Iyz",
            "Izz",
            "mx",
            "my",
            "mz",
            "m",
        )

        # defining conditions
        self.isFext = isFext

        # folder location
        self.robot_urdf = robot_urdf

        # initializing robot's models
        if not isFext:
            self.initFromURDF(robot_urdf, package_dirs=package_dirs)
        else:
            self.initFromURDF(robot_urdf, package_dirs=package_dirs,
                              root_joint=pin.JointModelFreeFlyer())
            
        if freeflyer_ori is not None and isFext == True : 
            self.model.jointPlacements[self.model.getJointId('root_joint')].rotation = freeflyer_ori
            ub = self.model.upperPositionLimit
            ub[:7] = 1
            self.model.upperPositionLimit = ub
            lb = self.model.lowerPositionLimit
            lb[:7] = -1
            self.model.lowerPositionLimit = lb
            self.data = self.model.createData()

        ## \todo test that this is equivalent to reloading the model
        self.geom_model = self.collision_model

def create_scene():
    """_Create the pinocchio models and data for the collaborative human-robot scene, featuring a 5 dof planar human model and a 7 dof Franka Emika Panda robot. Both entity are represented by one independant kinematic chain. Both are spaced with a given transformation matrix._

    Returns:
        _Model_: _The pinocchio model corresponding to the scene_
        _Data_: _The pinocchio data corresponding to the scene_
        _Geom_Model_: _The pinocchio visual model corresponding to the scene_
        _Geom_Model_: _The pinocchio  collision model corresponding to the scene_
    """
    # Loading human urdf
    human = Robot('models/others/robots/human_urdf/urdf/5dof_mpc.urdf','models/others/robots') 
    human_model = human.model
    human_visual_model = human.visual_model

    human_model_pose = pin.SE3(np.eye(3), np.matrix([-0.6,0.6,0.05]).T)

    # Loading Franka urdf
    robot = robex.load('panda')
    # Create a list of joints to lock
    jointsToLock = ['panda_finger_joint1', 'panda_finger_joint2']
 
    # Get the ID of all existing joints
    jointsToLockIDs = []
    for jn in jointsToLock:
        if robot.model.existJointName(jn):
            jointsToLockIDs.append(robot.model.getJointId(jn))
        else:
            print('Warning: joint ' + str(jn) + ' does not belong to the model!')

    initialJointConfig = pin.neutral(robot.model)

    geom_models = [robot.visual_model, robot.collision_model]
    model_reduced, geometric_models_reduced = pin.buildReducedModel(robot.model,list_of_geom_models=geom_models,list_of_joints_to_lock=jointsToLockIDs,reference_configuration=initialJointConfig)

    # geometric_models_reduced is a list, ordered as the passed variable "geom_models" so:
    visual_model_reduced, collision_model_reduced = geometric_models_reduced[0], geometric_models_reduced[1]

    # changing the names of frames and joints to avoid name conflicts
    # build custom model for human
    custom_human_model = human_model.copy()
    for i, name in enumerate(custom_human_model.names):
        custom_human_model.names[i] = name + '_human'
    for i, frame in enumerate(custom_human_model.frames):
        custom_human_model.frames[i].name = frame.name + '_human'

    # build custom model for panda
    custom_panda_model = model_reduced.copy()
    for i, name in enumerate(custom_panda_model.names):
        custom_panda_model.names[i] = name + '_robot'
    for i, frame in enumerate(custom_panda_model.frames):
        custom_panda_model.frames[i].name = frame.name + '_robot'

    # merge models
    scene_model,scene_visual_model = pin.appendModel(custom_panda_model,custom_human_model,visual_model_reduced,human_visual_model,model_reduced.getFrameId('universe'),human_model_pose)
    scene_data = scene_model.createData()

    scene_collision_model = scene_visual_model

    return scene_model,scene_data,scene_visual_model,scene_collision_model