#!/usr/bin/env python
#import ast
#import copy
import yaml
#import genpy
import rospy
#import scipy
import criros
import criutils
#import random
#import tabulate
#import itertools
#import collections
import numpy as np
#import COPE as cope
#import scipy.spatial
import openravepy as orpy
## Transformations
#import tf2_ros
#import tf.transformations as tr
## XML
#from lxml import etree
## Body holes
#from ikea_openrave.holes import BodyHoles
## ccplanner
#import bimanual.planners.cc_planner as ccp
## utils
#import bimanual.utils.utils as bimanual_utils
#import ikea_openrave.utils as rave_utils
## Messages
#from geometry_msgs.msg import Point, Vector3
#from ikea_msgs.msg import PlanningOptions, JointGoalRequest, InsertRequest, PickRequest
#from std_msgs.msg import String
#from sensor_msgs.msg import JointState
## Services
#from ikea_msgs.srv import PoseEstimation


#class ChainState(object):
#  def __init__(self):
#    self.body_pose = np.zeros(7)
#    self.qleft = np.zeros(6)
#    self.qright = np.zeros(6)
#
#  def __repr__(self):
#    if self.is_empty():
#      msg = '<ChainState: Empty>'
#    else:
#      msg = '<ChainState: Initialized>'
#    return msg
#
#  def __str__(self):
#    return self.__repr__()
#
#  def is_empty(self):
#    empty = dict()
#    for attr, value in self.__dict__.iteritems():
#      try:
#        iterator = iter(value)
#        empty[attr] = np.allclose(value, np.zeros_like(value))
#      except TypeError:
#        empty[attr] = True
#    return np.all(empty.values())

class ManagerBase(object):
  # TODO: remove object?
  def __init__(self, config_yaml):
    # Parse config file
    #yaml_path = resource_retriever.get_filename(config_uri, use_protocol=False)
    with open(config_yaml) as f:
      config = yaml.load(f)
    # TODO: only crucial sub-classes
    if not criutils.misc.has_keys(config, ['environment', 'robot', 'planning', 'execution']):
      raise IOError('Failed to load config file: {}'.format(config_yaml))

    # Retrieve all classes in config file
    # TODO
    # Give user warning to load registered non-crucial sub-classes
    # on their own.
    # TODO
    # Start only the crucial sub-classes
    self.env = orpy.Environment()
    self.env_mng = EnvironmentManager(config['environment'], self.env)
    self.pln_mng = PlanningManager(config['planning'],
                   env_manager = self.env_mng)
    # RobotManager
    #TODO:self.rbt_mng = RobotManager(config['robot'])
    #self.exe_mng = EnvironmentManager(config['execution'], self.pln_mng)

#class CollisionManager():
#  NO_COLLISION = 0
#  SELF_COLLISION = 1
#  ENV_COLLISION = 2
#  def __init__(self, env, checker_name):
#    self.env = env
#    self.collision_checking_enabled = False
#    self.alive_bodies = criros.raveutils.get_enabled_bodies(self.env)
#    self.change_collision_checker(checker_name)
#
#  def change_collision_checker(self, checker_name):
#    self.checker_name = checker_name
#    self.collision_checker = orpy.RaveCreateCollisionChecker(self.env, checker_name)
#
#  def check_robot_collisions(self, robot_name, qconfig):
#    robot = self.env.GetRobot(robot_name)
#    was_enable = self.collision_checking_enabled
#    self.enable_collision_checking(True)
#    qinit = robot.GetActiveDOFValues()
#    # Check collisions
#    collision_code = self.NO_COLLISION
#    with self.env:
#      robot.SetActiveDOFValues(qconfig)
#      if robot.CheckSelfCollision():
#        collision_code = self.SELF_COLLISION
#      if self.env.CheckCollision(robot):
#        collision_code = self.ENV_COLLISION
#    # Restore the robot position
#    with self.env:
#      robot.SetActiveDOFValues(qinit)
#    self.enable_collision_checking(was_enable)
#    return collision_code
#
#  def enable_collision_checking(self, enable):
#    # Avoid executing this method unnecessarily
#    if self.collision_checking_enabled == enable:
#      return
#    # Check we have created a valid collision checker
#    if self.collision_checker is None:
#      return False
#    # Include bodies that were added to the environment since last time we checked
#    current_enabled = criros.raveutils.get_enabled_bodies(self.env)
#    if self.alive_bodies != current_enabled:
#      self.alive_bodies = self.alive_bodies.union(current_enabled)
#    # Enable/disable bodies
#    for name in self.alive_bodies:
#      criros.raveutils.enable_body(self.env.GetKinBody(name), enable)
#    # Configure environment collision checker
#    with self.env:
#      if enable:
#        self.collision_checking_enabled = self.env.SetCollisionChecker(self.collision_checker)
#      else:
#        self.env.SetCollisionChecker(None)
#        self.collision_checking_enabled = False
#    return (enable == self.collision_checking_enabled)
#
#  def get_checker_name(self):
#    return self.checker_name

class EnvironmentManager():
  #joint_names = ['j1','j2','j3','j4','j5','j6']
  def __init__(self, config, env=None, logger=rospy):
    self.logger = logger
    #self.blacklisted = set()
    #self.updated_robots = dict()
    #self.updated_bodies = dict()
    #self.max_collision_penetration = config['max_collision_penetration']
    # Setup OpenRAVE environment
    if env is None:
      self.env = orpy.Environment()
    else:
      self.env = env
    orpy.RaveSetDebugLevel(orpy.DebugLevel.Fatal)
    self.environment_from_dict(config)
    if self.env is None:
      if config.has_key('world'):
        msg = config['world']
      else:
        msg = config
      self.logger.logerr('Failed to load OpenRAVE environment: {0}'.format(msg))
      return
    """
    # Start TF listener
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    # Subscribe to the joint_states topics
    self.js_topics = dict()
    self.js_msgs = dict()
    self.robot_frames = dict()
    self.children = dict()
    for robotname in config['robots'].keys():
      if self.env.GetRobot(robotname) is None:
        self.logger.logwarn('Failed to find robot in OpenRAVE: {0}'.format(robotname))
        continue
      topic = config['robots'][robotname]['topic']
      rospy.Subscriber(topic, JointState, self.cb_joint_states, callback_args=robotname, queue_size=1)
      self.js_topics[robotname] = topic
      self.robot_frames[robotname] = config['robots'][robotname]['frame_id']
      # Get robot children
      if config['robots'][robotname].has_key('children'):
        self.children[robotname] = []
        for body in self.env.GetBodies():
          for child in config['robots'][robotname]['children']:
            if child in body.GetName():
              self.children[robotname].append(body.GetName())
    """
    # Move origin to reference body
    # TODO: body name is currently upper/lowercase sensitive
    #       or offer suggestions
    if config.has_key('reference_body'):
      for body in self.env.GetBodies():
        if body.GetName() == config['reference_body']:
          self.reference_body = body
      self.move_origin_to_body()
    """
    # Create a publisher that will report the affected OpenRAVE objects
    self.report_pub = rospy.Publisher('/environment_manager/report', String, queue_size=3)
    # Start timer that will report what has been updated
    self.fixed_frame = config['fixed_frame']
    self.start_time = rospy.Time.now()
    self.elapsed_time = 0.0
    self.report_timer = rospy.Timer(rospy.Duration(config['report_rate']), self.report)
    rospy.on_shutdown(self.on_shutdown)
    """

  # TODO: Move TextColors
  def environment_from_dict(self, config, logger=criros.utils.TextColors()):
    """
    Loads and configures and OpenRAVE environment from a configuration dictionary.
    This approach allows to encapsulate additional information that would be tedious
    to include if we only used the OpenRAVE XML specification.
    @type  config: dict
    @param config: The configuration dictionary
    """
    if not isinstance(config, dict):
      logger.logwarn('config is not a dict')
      return None
    # Check required fields are in the config dict
    required_fields = ['world']
    if not criutils.misc.has_keys(config, required_fields):
      logger.logwarn( 'config dict does not have the required fields: {0}'.format(required_fields) )
      return None
    if not self.env.Load(config['world']):
      self.env = None
      return None
    # OPTIONAL parameters
    # Viewer parameters
    if config.has_key('viewer'):
      viewer_name = config['viewer']['name']
      if viewer_name == 'default':
        self.env.SetDefaultViewer()
      else:
        self.env.SetViewer(viewer_name)
      # The camera where we look the viewer from
      if config['viewer'].has_key('camera'):
        transform_dict = config['viewer']['camera']
        camera_fields = ['rotation','translation']
        if not criutils.misc.has_keys(transform_dict, camera_fields):
          logger.logwarn('camera dict does not have the required fields: {0}'.format(camera_fields))
        elif self.env.GetViewer() is not None:
          Tcam = criutils.conversions.from_dict(transform_dict)
          self.env.GetViewer().SetCamera(Tcam)
    # Return configured environment
    return self.env

  def move_origin_to_body(self):
    """
    Moves everything in the OpenRAVE scene so that the C{refbody} ends-up at the origin.
    @type  refbody: orpy.KinBody
    @param refbody: The body that will be at the origin
    """
    #env = refbody.GetEnv()
    #TODO: Move to criutils
    Toffset = criros.spalg.transform_inv( self.reference_body.GetTransform() )
    grabbed_names = [body.GetName() for robot in self.env.GetRobots() for body in robot.GetGrabbed()]
    with self.env:
      for body in self.env.GetBodies():
        # Dont move Grabbed bodies. They will move once we move the robot grabbing them.
        if body.GetName() in grabbed_names:
          continue
        Tbody = body.GetTransform()
        body.SetTransform( np.dot(Toffset, Tbody) )

#NOTE: Seems specific to ikea_assembly
#  def blacklist_body(self, body):
#    success = False
#    if type(body) is str:
#      self.blacklisted.add(body)
#      success = True
#    elif type(body) in [orpy.Robot, orpy.KinBody]:
#      self.blacklisted.add(body.GetName())
#      success = True
#    return success

# NOTE: To test
#  def cb_joint_states(self, msg, robotname):
#    """
#    Callback that will be executed every time a message is published to
#    the subscribed topics.
#    @type  msg: sensor_msgs.msg.JointState
#    @param msg: the published C{JointState} message
#    @type  robotname: string
#    @param robotname: name of the robot to whom the state belongs.
#    """
#    if robotname not in self.js_topics.keys():
#      return
#    self.js_msgs[robotname] = criros.utils.sorted_joint_state_msg(msg, self.joint_names)

# NOTE: Redundant getter
#  def get_fixed_frame(self):
#    return self.fixed_frame

# NOTE: To test, required in update_rave_environment
#  def get_transform_from_tf(self, parent, child, time=None):
#    """
#    Gets the transformation of the C{child} frame w.r.t the C{parent} frame from TF.
#    @type  parent: string
#    @param parent: The parent frame in the TF tree
#    @type  child: string
#    @param child: The child frame in the TF tree
#    @type  time: rospy.Time
#    @param time: The time when we want the transform. Use C{None} to get the latest available transform.
#    @rtype: np.array
#    @return: The transformation of the C{child} w.r.t the C{parent} frame. C{None} if failed.
#    """
#    if time is None:
#      time = rospy.Time()
#    try:
#      msg = self.tf_buffer.lookup_transform(parent, child, time)
#    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#      return None
#    return criros.conversions.from_transform(msg.transform)

# NOTE: To test
#  def get_updated_bodies(self):
#    return self.updated_bodies.keys()

# NOTE: To test
#  def get_updated_robots(self):
#    return self.updated_robots.keys()
#
  def is_valid(self):
    return (self.env is not None)

# NOTE: To test
#  def on_shutdown(self):
#    """
#    Stops running timers so that the script can exit gracefully.
#    """
#    if self.report_timer.is_alive():
#      self.report_timer.shutdown()

# NOTE: To test
#  def report(self, event):
#    """
#    Publishes a report (C{std_msgs/String} with the updated robots and bodies
#    @type  event: rospy.timer.TimerEvent
#    @param event: Unused. It's required so that the function can be called from a C{rospy.Timer}.
#    """
#    updated_robots = []
#    updated_bodies = []
#    for name, stamp in self.updated_robots.items():
#      updated_robots.append([stamp.to_sec(), name, self.js_topics[name]])
#    for name, stamp in self.updated_bodies.items():
#      frame_id = name
#      remark = ''
#      if self.robot_frames.has_key(name):
#        frame_id = self.robot_frames[name]
#      if frame_id == self.fixed_frame:
#        remark = '* Fixed frame'
#      updated_bodies.append([stamp.to_sec(), name, frame_id, remark])
#    state_msg = ''
#    if len(updated_robots) > 0:
#      state_msg += '\n\n'
#      state_msg += tabulate.tabulate(updated_robots, headers=['Stamp', 'OpenRAVE Robot', 'ROS Topic'])
#    if len(updated_bodies) > 0:
#      state_msg += '\n\n'
#      state_msg += tabulate.tabulate(updated_bodies, headers=['Stamp', 'OpenRAVE KinBody', 'TF frame_id', 'Remark'])
#    if len(updated_robots+updated_bodies) == 0:
#      state_msg += 'Neither robots nor bodies have been updated'
#    self.report_pub.publish(data=state_msg)

# NOTE: Required in PlanningManager
#  def update_rave_joints(self):
#    """
#    Updates the OpenRAVE robots joint positions.
#    """
#    updated_robots = []
#    for robotname, msg in self.js_msgs.items():
#      robot = self.env.GetRobot(robotname)
#      if robot is None:
#        self.logger.logwarn( 'Failed to find robot in OpenRAVE: {0}'.format(robotname) )
#        continue
#      if len(msg.position) != robot.GetActiveDOF():
#        self.logger.logwarn( 'Inconsistent msg size in joint_states: {0}!={1}'.format(len(msg.position), robot.GetActiveDOF()) )
#        continue
#      with self.env:
#        robot.SetActiveDOFValues(msg.position)
#        self.updated_robots[robotname] = rospy.Time.now() - self.start_time
#        updated_robots.append(robotname)
#    return updated_robots

# NOTE: Required in PlanningManager
#  def update_rave_environment(self, max_collision_penetration=None):
#    """
#    Reads the all the transformations of the bodies matching TF frames. The transformation
#    is given with respect to the C{fixed_frame}, then, updates the OpenRAVE environment.
#    @type  event: rospy.timer.TimerEvent
#    @param event: Event details when called from a C{rospy.Timer}.
#    """
#    if max_collision_penetration is None:
#      max_collision_penetration = self.max_collision_penetration
#    updated_bodies = []
#    updated_robots = []
#    # Blacklist grabbed bodies
#    blacklisted = set(self.blacklisted)
#    for robot in self.env.GetRobots():
#      for body in robot.GetGrabbed():
#        blacklisted.add(body.GetName())
#    # Attach children to the robots before moving the robot
#    attached = dict()
#    for robotname in self.children.keys():
#      robot = self.env.GetRobot(robotname)
#      attached[robotname] = []
#      for child in self.children[robotname]:
#        if child in blacklisted:
#          # Ignore blacklisted bodies
#          continue
#        body = self.env.GetKinBody(child)
#        if body is None:
#          # Ignore non existing bodies
#          continue
#        if body.IsRobot():
#          # Do not attach robots together
#          continue
#        robot.Grab(body)
#        attached[robotname].append(child)
#    # Update bodies transform
#    yaml_dict = yaml.load(self.tf_buffer.all_frames_as_yaml())
#    if not yaml_dict:
#      self.logger.logerr('No tf transform available.')
#      return
#    available_frames = yaml_dict.keys()
#    for body in self.env.GetBodies():
#      name = body.GetName()
#      if name in blacklisted:
#        # Do not update blacklisted bodies
#        continue
#      frame_id = None
#      if body.IsRobot() and name in self.robot_frames:
#        frame_id = self.robot_frames[name]
#      else:
#        frame_id = name
#      if (frame_id is None) or (frame_id not in available_frames):
#        continue
#      if frame_id == self.fixed_frame:
#        # Report that we have updated the body but don't waste time looking for a identity transform
#        self.updated_bodies[name] = rospy.Time.now() - self.start_time
#        continue
#      T = self.get_transform_from_tf(parent=self.fixed_frame, child=frame_id)
#      if T is None:
#        continue
#      Tinit = body.GetTransform()
#      with self.env:
#        body.SetTransform(T)
#      # Check that the new transform is not in collision with the environment
#      incollision = not criros.raveutils.move_out_of_collision(self.env,
#                                        body, max_collision_penetration)
#      if incollision:
#        self.logger.logwarn('New transform puts the body in collision: {0}'.format(name))
#        with self.env:
#          body.SetTransform(Tinit)
#      else:
#        self.updated_bodies[name] = rospy.Time.now() - self.start_time
#        updated_bodies.append(name)
#    # Deattach bodies
#    for robotname in attached.keys():
#      robot = self.env.GetRobot(robotname)
#      for child in attached[robotname]:
#        if child in blacklisted:
#          # Ignore blacklisted bodies
#          continue
#        body = self.env.GetKinBody(child)
#        robot.Release(body)
#    # Update robots joint values
#    for robotname, msg in self.js_msgs.items():
#      robot = self.env.GetRobot(robotname)
#      if robot is None:
#        self.logger.logwarn( 'Failed to find robot in OpenRAVE: {0}'.format(robotname) )
#        continue
#      if len(msg.position) != robot.GetActiveDOF():
#        self.logger.logwarn( 'Inconsistent msg size in joint_states: {0}!={1}'.format(len(msg.position), robot.GetActiveDOF()) )
#        continue
#      with self.env:
#        robot.SetActiveDOFValues(msg.position)
#        self.updated_robots[robotname] = rospy.Time.now() - self.start_time
#        updated_robots.append(robotname)
#    return updated_robots, updated_bodies

# NOTE: Seems specific to ikea_assembly
#  def detect_bodies(self, bodies, srv_node='/ikea_pose_estimation', srv_timeout=5):
#    # Set up rosservice proxy
#    pose_estimation = rospy.ServiceProxy(srv_node, PoseEstimation)
#    try:
#      pose_estimation.wait_for_service(timeout=srv_timeout)
#    except rospy.ROSException:
#      self.logger.logerr('Service [{0}] is not available.'.format(srv_node))
#      return
#    # Detect bodies
#    for body_name in bodies:
#      res = pose_estimation(body_name)
#      if res.success:
#        self.logger.loginfo('Pose estimation SUCCEEDED for body: {0}'.format(body_name))
#      else:
#        self.logger.logwarn('Pose estimation FAILED for body: {0}'.format(body_name))

# NOTE: Seems specific to ikea_assembly
#  @staticmethod
#  def calibrate_frame_pose(qgrasps, frame, manips):
#    # NB: this function only applies to upper (right) frame for ikea assembly
#    lqgrasp, rqgrasp = qgrasps
#    lmanip, rmanip = manips
#    T_frame_0 = frame.GetTransform()
#    # start from right gripper
#    T_rel = np.dot(np.linalg.inv(rave_utils.ComputeTGripper2(frame, rqgrasp[0], rqgrasp)), T_frame_0)
#    T_right = rmanip.GetEndEffectorTransform()
#    T_left = lmanip.GetEndEffectorTransform()
#    T_frame_1 = np.dot(T_right, T_rel)
#    frame.SetTransform(T_frame_1)
#    T_upper_link = frame.GetLink('link_upper').GetTransform()
#    cali_axis = np.dot(T_upper_link[:3, :3], [0, 1, 0])
#    T_back = frame.GetLink('back3').GetTransform()
#    back_axis = np.dot(T_back[:3, :3], [0, 0, 1])
#    back_trans = T_back[:3, 3]
#    gripper_axis = np.dot(T_left[:3, :3], [0, 0, 1])
#    gripper_trans = T_left[:3, 3]
#    BAx, BAy, BAz = back_axis
#    BOx, BOy, BOz = back_trans
#    GAx, GAy, GAz = gripper_axis
#    GOx, GOy, GOz = gripper_trans
#    CAx, CAy, CAz = cali_axis
#    Matrix_A = np.array([[GAx, BAx, CAx],
#                         [GAy, BAy, CAy],
#                         [GAz, BAz, CAz]])
#    Matrix_B = np.array([GOx - BOx, GOy - BOy, GOz - BOz])
#    Vector_x = np.dot(np.linalg.inv(Matrix_A), Matrix_B)
#    coef = Vector_x[2]
#    T_frame_2 = np.array(T_frame_1)
#    T_frame_2[:3, 3] += coef * cali_axis

#class MotionPlan():
#  COMMAND_GRIPPER   = 'CommandGripper'
#  CLOSE_GRIPPER     = 'CloseGripper'
#  INSERT_PIN        = 'InsertPin'
#  MULTIPLE_ROBOTS   = 'MultipleRobots'
#  OPEN_GRIPPER      = 'OpenGripper'
#  TRAJECTORY        = 'Trajectory'
#  CLOSED_CHAIN      = 'ClosedChain'
#  valid_categories = [COMMAND_GRIPPER, CLOSE_GRIPPER, INSERT_PIN, MULTIPLE_ROBOTS, OPEN_GRIPPER, TRAJECTORY, CLOSED_CHAIN]
#  def __init__(self):
#    self.divider = ' '
#    self.clear()
#
#  def __add__(self, other):
#    res = MotionPlan()
#    res.steps   = self.steps  + other.steps
#    return res
#
#  def __getitem__(self, index):
#    if isinstance(index, int):
#      step = self.steps[index]
#      class_name = step['category'] + 'Step'
#      step_class = collections.namedtuple(class_name, step.iterkeys())
#      retval = step_class(**step)
#    elif isinstance(index, slice):
#      retval = MotionPlan()
#      retval.steps = self.steps[index]
#    else:
#        raise TypeError("index must be int or slice")
#    return retval
#
#  def __len__(self):
#    return len(self.steps)
#
#  def __repr__(self):
#    return '<MotionPlan:{0} steps>'.format(len(self))
#
#  def __str__(self):
#    return self.serialize()
#
#  def add_step(self, step_type, robots, data=None, wait=True):
#    if step_type not in self.valid_categories:
#      return False
#    step = dict()
#    step['category'] = step_type
#    step['wait'] = wait
#    # Robot or robot names
#    if step_type in [self.CLOSED_CHAIN, self.MULTIPLE_ROBOTS, self.INSERT_PIN]:
#      step['robots'] = robots
#    else:
#      step['robot'] = robots
#    # Depending on the step type we have different 'data' element
#    if   step_type == self.COMMAND_GRIPPER:
#      step['command'] = data
#    elif step_type == self.CLOSE_GRIPPER:
#      step['body_name'] = data
#    elif step_type == self.INSERT_PIN:
#      step.update(data)
#    elif step_type == self.MULTIPLE_ROBOTS or step_type == self.TRAJECTORY:
#      step['trajectory'] = data
#    elif step_type == self.CLOSED_CHAIN:
#      step.update(data)
#    elif step_type == self.OPEN_GRIPPER:
#      pass
#    self.steps.append(step)
#    return True
#
#  def clear(self):
#    self.steps = []
#
#  def deserialize(self, env, xml_str):
#    root = etree.fromstring(xml_str)
#    if root.find('version').text != self.get_version():
#      return False
#    self.clear()
#    for child in root:
#      if child.tag != 'step':
#        continue
#      # Check we have a valid step category
#      if child.find('category').text not in self.valid_categories:
#        self.clear()
#        return False
#      # Extract steps information
#      step = dict()
#      for grandchild in child:
#        class_type = grandchild.get('type')
#        if class_type == orpy.Trajectory.__name__:
#          del grandchild.attrib['type']
#          traj_xml = etree.tostring(grandchild)
#          traj = orpy.RaveCreateTrajectory(env, '')
#          traj.deserialize(traj_xml)
#          step[grandchild.tag] = traj
#        elif class_type == ccp.CCTrajectory.__name__:
#          step[grandchild.tag] = ccp.CCTrajectory.deserialize(grandchild.text)
#          # create correspoding ccplanner
#          obj = env.GetKinBody(child.find('obj').text)
#          robots = [env.GetRobot(robot_name) for robot_name in child.find('robots').text.split(self.divider)]
#          step['planner'] = ccp.CCPlanner(obj, robots)
#        elif class_type == list.__name__:
#          step[grandchild.tag] = grandchild.text.split(self.divider)
#        elif class_type == bool.__name__:
#          step[grandchild.tag] = ast.literal_eval(grandchild.text)
#        elif class_type == type(None).__name__:
#          step[grandchild.tag] = None
#        else:
#          class_object = eval(class_type)
#          step[grandchild.tag] = class_object(grandchild.text)
#      self.steps.append(step)
#    return True
#
#  def get_robot_names(self):
#    names = set()
#    for step in self.steps:
#      if step['category'] in [self.CLOSED_CHAIN, self.MULTIPLE_ROBOTS, self.INSERT_PIN]:
#        names.union(set(step['robots']))
#      else:
#        names.add(step['robot'])
#    return names
#
#  def get_plan_info(self):
#    info = []
#    for i,step in enumerate(self):
#      if step.category == self.COMMAND_GRIPPER:
#        info.append((i, step.robot, step.category, step.command))
#      elif step.category in [self.MULTIPLE_ROBOTS, self.CLOSED_CHAIN]:
#        info.append((i, ', '.join(step.robots), step.category))
#      elif step.category == self.INSERT_PIN:
#        info.append((i, step.category, step.body_to_insert,
#                                                  step.body_with_hole))
#      else:
#        info.append((i, step.robot, step.category))
#    return info
#
#  def get_version(self):
#    return '0.1.0'
#
#  def is_valid(self):
#    return True
#
#  def reversed(self):
#    plan_rev = MotionPlan()
#    for step in self.steps:
#      step_rev = dict(step)
#      if   step['category'] in [self.COMMAND_GRIPPER, self.OPEN_GRIPPER]:
#        continue # Do nothing
#      elif step['category'] == self.CLOSE_GRIPPER:
#        step_rev['category'] = self.OPEN_GRIPPER
#      elif step['category'] in [self.MULTIPLE_ROBOTS, self.TRAJECTORY]:
#        # Reverse trajectory
#        traj_rev = orpy.planningutils.ReverseTrajectory(step['trajectory'])
#        step_rev['trajectory'] = traj_rev
#      elif step['category'] == self.CLOSED_CHAIN:
#        traj_rev = ccp.CCTrajectory.reverse(step['trajectory'])
#        step_rev['trajectory'] = traj_rev
#      plan_rev.steps.append(step_rev)
#    plan_rev.steps.reverse()
#    return plan_rev
#
#  def serialize(self):
#    plan = etree.Element('plan')
#    version = etree.SubElement(plan, 'version')
#    version.text = self.get_version()
#    for step in self.steps:
#      # Create the xml elements
#      element_step = etree.SubElement(plan, 'step')
#      for key, value in step.items():
#        # OpenRAVE trajectory
#        if type(value) == orpy.Trajectory:
#          sub_element = etree.fromstring(value.serialize())
#          sub_element.attrib['type'] = type(value).__name__
#          element_step.append( copy.deepcopy(sub_element) )
#          continue
#        if type(value) == ccp.CCTrajectory:
#          sub_element = etree.SubElement(element_step, key)
#          sub_element.attrib['type'] = type(value).__name__
#          sub_element.text = ccp.CCTrajectory.serialize(value)
#          continue
#        if type(value) == ccp.CCPlanner:
#          continue
#        # Convert unicode strings to normal strings
#        if type(value) == unicode:
#          value = str(value)
#        # Other data types
#        sub_element = etree.SubElement(element_step, key)
#        sub_element.attrib['type'] = type(value).__name__
#        if type(value) == list:
#          sub_element.text = self.divider.join(value)
#        else:
#          sub_element.text = str(value)
#    return etree.tostring(plan, pretty_print=True)

class PlanningManager():
  PLANNERS = ['BiRRT', 'BasicRRT']
  PP_PLANNERS = ['shortcut_linear', 'LinearTrajectoryRetimer', 'ParabolicTrajectoryRetimer', 'LinearSmoother', 'ParabolicSmoother']
  def __init__(self, config, env=None, env_manager=None, logger=rospy):
#    self.axes = []
    self.logger = logger
    if env is None and env_manager is None:
      self.logger.logerr('Please provide an OpenRAVE environment or a EnvironmentManager')
      return
    elif env_manager is not None:
      if not env_manager.is_valid():
        self.logger.logerr('Please provide a valid EnvironmentManager')
        return
      else:
        self.env = env_manager.env
        self.env_manager = env_manager
    else:
      self.env = env
      self.env_manager = env_manager
    # Configure environment collision checker
#    self.collision = CollisionManager(self.env, config['collision_checker'])
#    if not self.collision.enable_collision_checking(True):
#      self.logger.logwarn( 'Failed to enable collision checker: {0}'.format(self.collision.get_checker_name()) )
    # NOTE: Why involve rosmsg? My guess is to make it publishable
    # Load the default planning options
    self.default_options = PlanningOptions()
    genpy.message.fill_message_args(self.default_options, [config['default_options']])
    self.default_options = self.merge_default_options(self.default_options)
    # Load available IK solvers
    # NOTE: Load all available IK solvers (for all supported iktypes)
    #       for all robot-manipulator permutations
    # NOTE: Why load all iktypes for a particular robot-manipulator combi tho?
    iktypes = []
    for robot_name in config['robots'].keys():
      robot = self.env.GetRobot(robot_name)
      if robot is None:
        self.logger.logwarn('Could not find robot in OpenRAVE: {0}'.format(robot_name))
        continue
      robot_iktypes = self.get_robot_iktypes(robot)
      for manip_name in robot_iktypes.keys():
        for iktype in robot_iktypes[manip_name]:
          iktypes.append((robot_name, manip_name, iktype))
    # Load available IK models
    self.ikmodels = dict()
    for robot_name, manip_name, iktype in iktypes:
      manip = self.env.GetRobot(robot_name).GetManipulator(manip_name)
      ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(iktype=iktype, manip=manip)
      if not ikmodel.load():
        self.logger.logwarn('Failed to load IKFast {0} for manip {1}'.format(iktype.name, manip_name))
      else:
        key = (robot_name, manip_name, iktype.name)
        self.ikmodels[key] = ikmodel
    # Store initial robot configuration
    self.joint_names = dict()
    self.velocity_limits = dict()
    self.acceleration_limits = dict()
    # Active manipulator parameters
    self.iktype = dict()
    self.dofindices = dict()
    for robot_name in config['robots'].keys():
      robot = self.env.GetRobot(robot_name)
      self.joint_names[robot_name] = config['robots'][robot_name]['joint_names']
      self.velocity_limits[robot_name] = robot.GetDOFVelocityLimits()
      self.acceleration_limits[robot_name] = robot.GetDOFVelocityLimits()
      # NOTE: In here, every robot listed in config will have to use
      #       the same manipulator and iktype in default_options?
      #       Maybe check if user provided specific *manipulator* and 
      #       *iktype* options under robot?
      self.change_active_manipulator(robot_name, self.default_options.manipulator, self.default_options.iktype)
      # TODO: Investigate
      self.update_link_stats(robot_name)
      # Check that the robots are not in collision
      qstart = robot.GetActiveDOFValues()
      if self.collision.check_robot_collisions(robot_name, qstart) == CollisionManager.SELF_COLLISION:
        self.logger.logwarn('Robot is in self-collision: {0}'.format(robot_name) )
      elif self.collision.check_robot_collisions(robot_name, qstart) == CollisionManager.ENV_COLLISION:
        self.logger.logwarn('Robot is in collision with the environment: {0}'.format(robot_name) )
#    # Gripper parameters
#    self.gripper_parameters = dict(config['gripper'])
#    # Get the body holes transformations
#    self.holes = dict()
#    for body in self.env.GetBodies():
#      if body.IsRobot():
#        continue
#      body_name = body.GetName()
#      bholes = BodyHoles(body, relative=True, logger=self.logger)
#      if bholes.get_num_holes() == 0:
#        continue
#      self.holes[body_name] = bholes.get_transforms()
#    # Report bodies in collision
#    for body in self.env.GetBodies():
#      if body.IsRobot():
#        continue
#      if self.env.CheckCollision(body):
#        self.logger.logwarn('Object is in collision with the environment: {0}'.format(body.GetName()) )

  def get_robot_iktypes(robot):
    # TODO: Modify docstring format
    # NOTE: I think this returns all the iksolvers (include all supported iktypes)
    #       that are available in the database
    """
    Returns a dict with the manipulator:[iktypes] pairs of available iksolvers .
    @type  refbody: orpy.Robot
    @param refbody: The OpenRAVE robot
    @rtype: orpy.Environment
    @return: The dict with the manipname:[iktypes] pairs.
    """
    robot_iktypes = dict()
    for manip in robot.GetManipulators():
      iktypes = []
      # TODO: Add SUPPORTED_IK_TYPES
      for iktype in SUPPORTED_IK_TYPES:
        ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(iktype=iktype, manip=manip)
        if ikmodel.load():
          iktypes.append(iktype)
      if iktypes:
        robot_iktypes[manip.GetName()] = list(iktypes)
    return robot_iktypes

#  def apply_plan(self, plan):
#    # Check we have a valid plan
#    if not self.check_plan_consistency(plan):
#      return False
#    # Apply
#    for step in plan:
#      if step.category == MotionPlan.MULTIPLE_ROBOTS:
#        robot1 = self.env.GetRobot(step.robots[0])
#        robot2 = self.env.GetRobot(step.robots[1])
#        traj1 = criros.conversions.ros_trajectory_from_openrave(step.robots[0], step.trajectory)
#        traj2 = criros.conversions.ros_trajectory_from_openrave(step.robots[1], step.trajectory)
#        robot1.SetActiveDOFValues(traj1.points[-1].positions)
#        robot2.SetActiveDOFValues(traj2.points[-1].positions)
#      elif step.category == MotionPlan.TRAJECTORY:
#        robot = self.env.GetRobot(step.robot)
#        traj = criros.conversions.ros_trajectory_from_openrave(step.robot, step.trajectory)
#        robot.SetActiveDOFValues(traj.points[-1].positions)
#      else:
#        self.visualize_step(step)
#    return True
#
#  def balance_body_transform(self, start, goal, weights=[1.,10.]):
#    transform_inv = criros.spalg.transform_inv
#    def obj_func(x, T0, Tleft, Tright):
#      T = np.dot(T0, cope.VecToTran(x))
#      cost_left = np.linalg.norm(cope.TranToVec(np.dot(Tleft,T)))
#      cost_right = np.linalg.norm(cope.TranToVec(np.dot(Tright,T)))
#      cost = np.array([cost_left, cost_right])**2
#      return np.dot(weights, cost)
#    # Working entities
#    left_robot = self.env.GetRobot('left')
#    right_robot = self.env.GetRobot('right')
#    Tbody_start = orpy.matrixFromPose(start.body_pose)
#    Tbody_goal = orpy.matrixFromPose(goal.body_pose)
#    # Left robot
#    with left_robot:
#      left_robot.SetActiveDOFValues(start.qleft)
#      left_manip = left_robot.GetActiveManipulator()
#      Tleft_gripper_start = left_manip.GetEndEffectorTransform()
#      Tleft_rel_start = np.dot(transform_inv(Tleft_gripper_start), Tbody_start)
#      left_robot.SetActiveDOFValues(goal.qleft)
#      Tleft_gripper_goal = left_manip.GetEndEffectorTransform()
#      Tleft = np.dot(transform_inv(Tleft_rel_start), transform_inv(Tleft_gripper_goal))
#    # Right robot
#    with right_robot:
#      right_robot.SetActiveDOFValues(start.qright)
#      right_manip = right_robot.GetActiveManipulator()
#      Tright_gripper_start = right_manip.GetEndEffectorTransform()
#      Tright_rel_start = np.dot(transform_inv(Tright_gripper_start), Tbody_start)
#      right_robot.SetActiveDOFValues(goal.qright)
#      Tright_gripper_goal = right_manip.GetEndEffectorTransform()
#      Tright = np.dot(transform_inv(Tright_rel_start), transform_inv(Tright_gripper_goal))
#    # Minimize
#    res = scipy.optimize.minimize(obj_func, np.zeros(6), args = (Tbody_goal, Tleft, Tright), method='BFGS', options={'eps': 1e-8, 'disp': False})
#    twist = res['x']
#    Tbody_opt = np.dot(Tbody_goal, cope.VecToTran(twist))
#    opt = ChainState()
#    opt.body_pose = orpy.poseFromMatrix(Tbody_opt)
#    opt.qleft = np.array(goal.qleft)
#    opt.qright = np.array(goal.qright)
#    return opt, twist
#
#  def bimanual_trajectory(self, bimanual_config, options):
#    # Plan multiple robots
#    robot1 = self.env.GetRobot(bimanual_config[0][0])
#    robot2 = self.env.GetRobot(bimanual_config[1][0])
#    goal_config = np.hstack((bimanual_config[0][1], bimanual_config[1][1]))
#    # Apply velocity and acceleration factors
#    robot_name = robot1.GetName()
#    robot1.SetDOFVelocityLimits(self.velocity_limits[robot_name]*options.velocity_factor)
#    robot1.SetDOFAccelerationLimits(self.acceleration_limits[robot_name]*options.acceleration_factor)
#    robot_name = robot2.GetName()
#    robot2.SetDOFVelocityLimits(self.velocity_limits[robot_name]*options.velocity_factor)
#    robot2.SetDOFAccelerationLimits(self.acceleration_limits[robot_name]*options.acceleration_factor)
#    # Planner parameters
#    params = orpy.Planner.PlannerParameters()
#    spec1 = robot1.GetActiveManipulator().GetArmConfigurationSpecification()
#    spec2 = robot2.GetActiveManipulator().GetArmConfigurationSpecification()
#    params.SetConfigurationSpecification(self.env, spec1+spec2)
#    params.SetGoalConfig(goal_config)
#    params.SetMaxIterations(options.max_iterations)
#    params.SetPostProcessing(options.pp_planner, '<_nmaxiterations>{0}</_nmaxiterations>'.format(options.pp_iterations))
#    # Start the planner
#    traj = orpy.RaveCreateTrajectory(self.env, '')
#    planner = orpy.RaveCreatePlanner(self.env, options.planner)
#    success = planner.InitPlan(None, params)
#    if not success:
#      self.logger.logwarn('Failed to initialize planning for bimanual motion.')
#      return None
#    status = planner.PlanPath(traj)
#    if status != orpy.PlannerStatus.HasSolution:
#      self.logger.logwarn('Failed to plan bimanual motion.')
#      return None
#    return traj
#
  def change_active_manipulator(self, robot_name, manip_name, iktype_name):
    manip_key = (robot_name, manip_name, iktype_name)
    if manip_key not in self.ikmodels:
      return False
    robot = self.env.GetRobot(robot_name)
    if robot is None:
      return False
    try:
      manipulator = robot.SetActiveManipulator(manip_name)
      manipulator.SetIKSolver(self.ikmodels[manip_key].iksolver)
    except:
      return False
    # Store the active iksolver parameters
    self.iktype[robot_name] = orpy.IkParameterizationType.names[iktype_name]
    self.dofindices[robot_name] = manipulator.GetArmIndices()
    robot.SetActiveDOFs(self.dofindices[robot_name])
    return True
#
#  def clear_axes(self):
#    del self.axes
#    self.axes = []
#
#  def compute_velocity_from_pose(self, robot_name, target_pose, qrobot=None, enforce_limits=True):
#    robot = self.env.GetRobot(robot_name)
#    manipulator = robot.GetActiveManipulator()
#    qinit = robot.GetActiveDOFValues()
#    if qrobot is None:
#      qrobot = np.array(qinit)
#    # Get the robot Jacobian
#    with self.env:
#      robot.SetActiveDOFValues(qrobot)
#      robot_pose = manipulator.GetTransformPose()
#      Jtrans = manipulator.CalculateJacobian()
#      Jquat = manipulator.CalculateRotationJacobian()
#      robot.SetActiveDOFValues(qinit)
#    # Enforce Slerp shortest path
#    if np.dot(robot_pose[:4], target_pose[:4]) < 0:
#      robot_pose[:4] *= -1.
#      Jquat *= -1.
#    J = np.vstack([Jquat, Jtrans])
#    # Funny enough, this way to compute the pose error is much better than
#    # estimating the quaternion error: q1 * inv(q2) properly
#    pose_error = target_pose - robot_pose
#    # I compared np.dot(np.linalg.pinv(J), pose_error) vs np.linalg.lstsq(J, pose_error)
#    # and the pseudo-inverse is slightly faster
#    qd = np.dot(np.linalg.pinv(J), pose_error)
#    # Make sure that velocities are within the limits
#    max_velocities = robot.GetActiveDOFMaxVel()
#    if enforce_limits and not (np.abs(qd) < max_velocities).all():
#      qd *= np.min(max_velocities / np.abs(qd))
#    return qd
#
#  def compute_velocity_from_position(self, robot_name, target_position, qrobot=None, enforce_limits=True):
#    robot = self.env.GetRobot(robot_name)
#    manipulator = robot.GetActiveManipulator()
#    qinit = robot.GetActiveDOFValues()
#    if qrobot is None:
#      qrobot = np.array(qinit)
#    # Get the robot Jacobian
#    with self.env:
#      robot.SetActiveDOFValues(qrobot)
#      robot_position = manipulator.GetEndEffectorTransform()[:3,3]
#      J = manipulator.CalculateJacobian()
#      robot.SetActiveDOFValues(qinit)
#    error = target_position - robot_position
#    qd = np.dot(np.linalg.pinv(J), error)
#    # Make sure that velocities are within the limits
#    max_velocities = robot.GetActiveDOFMaxVel()
#    if enforce_limits and not (np.abs(qd) < max_velocities).all():
#      qd *= np.min(max_velocities / np.abs(qd))
#    return qd
#
#  def draw_axes(self, T, dist=0.03, linewidth=2.):
#    """
#    Adds an RGB axes to the OpenRAVE environment
#    @type  T: string
#    @param T: homogeneous transformation where to draw the axes
#    @type  dist: float
#    @param dist: length of the axes in meters
#    @type  linewidth: float
#    @param linewidth: linewidth of the axes in pixels
#    @rtype: int
#    @return: The total axes added to the scene
#    """
#    with self.env:
#      self.axes.append(orpy.misc.DrawAxes(self.env, T, dist=dist,
#                                                  linewidth=linewidth))
#    return len(self.axes)
#
#  def draw_ray(self, ray, dist=0.03, linewidth=2.):
#    iktype = orpy.IkParameterizationType.TranslationDirection5D
#    param = orpy.IkParameterization(ray, iktype)
#    self.axes.append(orpy.misc.DrawIkparam2(self.env, param, dist=dist,
#                                                  linewidth=linewidth))
#    return len(self.axes)
#
#  def find_bimanual_insertion_configuration(self, req):
#    """
#    @type  support_aabb: orpy.AABB
#    @param support_aabb: Axis-aligned bounding box
#    """
#    # Check the body and the hole exist
#    if req.body_with_hole not in self.holes:
#      self.logger.logdebug('Invalid body_with_hole was given: {0}'.format(req.body_with_hole))
#      return None
#    if req.hole_index < 0 or req.hole_index >= len(self.holes[req.body_with_hole]):
#      self.logger.logdebug('Invalid hole_index was given: {0}'.format(req.hole_index))
#      return None
#    Thole_rel = self.holes[req.body_with_hole][req.hole_index]
#    # Get the grasping and inserting robots
#    grasping = None
#    inserting = None
#    for robot in self.env.GetRobots():
#      for body in robot.GetGrabbed():
#        if body.GetName() == req.body_with_hole:
#          grasping = robot
#        elif body.GetName() == req.body_to_insert:
#          inserting = robot
#    if grasping is None or inserting is None:
#      self.logger.logdebug('Failed to find the grasping and inserting robots')
#      return None
#    # Start configuration
#    qstart = []
#    qstart.append(grasping.GetActiveDOFValues())
#    qstart.append(inserting.GetActiveDOFValues())
#    if req.regrasp_insertion_object:
#      robot = inserting
#      taskmanip = orpy.interfaces.TaskManipulation(robot)
#      taskmanip.CloseFingers()
#      robot.WaitForController(0)
#    if req.regrasp_receptacle_object:
#      robot = grasping
#      taskmanip = orpy.interfaces.TaskManipulation(robot)
#      taskmanip.CloseFingers()
#      robot.WaitForController(0)
#    # Working entities
#    body_with_hole = self.env.GetKinBody(req.body_with_hole)
#    body_to_insert = self.env.GetKinBody(req.body_to_insert)
#    # Match insertion directions
#    direction =  tr.unit_vector( criros.conversions.from_vector3(req.direction) )
#    direction_in_hole_frame =  tr.unit_vector( criros.conversions.from_vector3(req.direction_in_hole_frame) )
#    direction_in_pin_frame =  tr.unit_vector( criros.conversions.from_vector3(req.direction_in_pin_frame) )
#    # Align hole
#    Rhole_target = criros.spalg.rotation_matrix_from_axes(direction, direction_in_hole_frame)
#    Rbody_target = np.eye(4)
#    Rbody_target[:3,:3] = np.dot(Thole_rel[:3,:3].T, Rhole_target[:3,:3])
#    # Align pin
#    pin = orpy.RaveCreateKinBody(self.env, body_to_insert.GetXMLId())
#    pin.Clone(body_to_insert, 0)
#    self.env.Add(pin, True)
#    Rpin_target = criros.spalg.rotation_matrix_from_axes(direction, direction_in_pin_frame)
#    pin.SetTransform(Rpin_target)
#    aabb = pin.ComputeAABB()
#    self.env.Remove(pin)
#    pin_height = abs(np.dot(direction, aabb.extents())*2)
#    pin_offset = -abs(req.standoff + pin_height/2.0)*direction
#    # Get transform of the TCP w.r.t the body_with_hole
#    Tgrasped = body_with_hole.GetTransform()
#    Ttcp = grasping.GetActiveManipulator().GetTransform()
#    Ttcp_wrt_grasped = np.dot(criros.spalg.transform_inv(Tgrasped), Ttcp)
#    # Check if we need a support surface
#    dist_to_hole = np.linalg.norm(np.dot(Tgrasped, Thole_rel)[:2,3] - Ttcp[:2,3])
#    use_support = req.use_support and dist_to_hole > 0.2
#    extra_offset = 0
#    if use_support:
#      support_link = inserting.GetLink('top_plate')
#      support_aabb = support_link.ComputeAABB()
#      extents = support_aabb.extents()
#      # Crop XY so that the hole is really on top
#      extents[:2] -= 15e-3
#      support_aabb = orpy.AABB(support_aabb.pos(), extents)
#      corners = []
#      for k in itertools.product([-1,1],[-1,1]):
#        corners.append(support_aabb.pos()[:2] + np.array(k)*support_aabb.extents()[:2])
#      triangulation = scipy.spatial.Delaunay(corners)
#    # Get transform of the TCP w.r.t the pin
#    Tpin = body_to_insert.GetTransform()
#    Ttcp = inserting.GetActiveManipulator().GetTransform()
#    Ttcp_wrt_pin = np.dot(criros.spalg.transform_inv(Tpin), Ttcp)
#    # Find where to place the grasped object
#    grasped = orpy.RaveCreateKinBody(self.env, body_with_hole.GetXMLId())
#    grasped.Clone(body_with_hole, 0)
#    self.env.Add(grasped, True)
#    grasped.SetTransform(Rbody_target)
#    aabb = grasped.ComputeAABB()        # Need the bounding box with the hole aligned (Rbody_target)
#    self.env.Remove(grasped)
#    # Get body centroid in the body frame
#    trimesh = self.env.Triangulate(body_with_hole)
#    centroid = np.mean(trimesh.vertices, axis=0)
#    centroid_offset = np.dot(Tgrasped[:3,:3].T, centroid-Tgrasped[:3,3])
#    # Random positions in a search sphere with the given radius for the body_with_hole centroid
#    midpoint_offset = criros.conversions.from_point(req.midpoint_offset)
#    pivot_pos = (grasping.GetTransform()[:3,3] + inserting.GetTransform()[:3,3]) / 2.0
#    pivot_pos += midpoint_offset
#    pivot_pos[2] =  aabb.extents()[2]
#    radius = req.search_radius * np.random.random(size=(req.max_search_positions,1))
#    positions = np.random.normal(size=(req.max_search_positions, 3))
#    positions /= np.linalg.norm(positions, axis=1)[:, np.newaxis]
#    positions = pivot_pos + radius*positions
#    # Range of orientations
#    yaws = np.arange(0, req.max_search_angle, req.search_angle_step)
#    direction_in_body_frame = np.dot(Rbody_target[:3,:3].T, direction)
#    orientations = [np.dot(Rbody_target, tr.compose_matrix(angles=y*direction_in_body_frame)) for y in yaws]
#    combinations = list(itertools.product(orientations, positions))
#    random.shuffle(combinations)
#    # Find a solution
#    qgrasping = None
#    qinserting = None
#    # Try first the current system configuration
#    Tbody = body_with_hole.GetTransform()
#    orientation = np.array(Tbody)
#    orientation[:3,3] = np.zeros(3)
#    position = Tbody[:3,3] + np.dot(Tbody[:3,:3], centroid_offset)
#    combinations.insert(0, (orientation, position))
#    for orientation, position in combinations:
#      Tbody = np.array(orientation)
#      Tbody[:3,3] = position - np.dot(Tbody[:3,:3], centroid_offset)
#      Ttcp = np.dot(Tbody, Ttcp_wrt_grasped)
#      grasping.SetActiveDOFValues(qstart[0])
#      qgrasping = self.ik_find_solution(grasping.GetName(), Ttcp, req.options_grasping)
#      if qgrasping is None:
#        continue
#      grasping.SetActiveDOFValues(qgrasping)
#      Thole = np.dot(Tbody, Thole_rel)
#      # Check we can support the hole on the top plate
#      if use_support:
#        on_top = triangulation.find_simplex(Thole[:2,3]) >= 0
#        body_aabb = body_with_hole.ComputeAABB()
#        zbody = body_aabb.pos()[2] - body_aabb.extents()[2]
#        zsupport = support_aabb.pos()[2] + support_aabb.extents()[2]
#        extra_offset = zbody - zsupport
#        good_distance = 5e-3 < extra_offset < 20e-3
#        if not (on_top and good_distance):
#          continue
#      Tpin = np.array(Rpin_target)
#      Tpin[:3,3] = Thole[:3,3] + pin_offset
#      Ttcp = np.dot(Tpin, Ttcp_wrt_pin)
#      inserting.SetActiveDOFValues(qstart[1])
#      qinserting = self.ik_find_solution(inserting.GetName(), Ttcp, req.options_inserting)
#      if qinserting is not None:
#        break
#    # Restore robots configuration
#    grasping.SetActiveDOFValues(qstart[0])
#    inserting.SetActiveDOFValues(qstart[1])
#    # Return the found solution, if any
#    if (qgrasping is not None) and (qinserting is not None):
#      configuration = []
#      configuration.append((grasping.GetName(), qgrasping, use_support, extra_offset))
#      configuration.append((inserting.GetName(), qinserting))
#      return configuration
#    else:
#      self.logger.logdebug('Failed to find a valid bimanual insertion configuration')
#      return None
#
#  def get_available_robots(self):
#    names = set()
#    for robot_name, manip_name, iktype in self.ikmodels.keys():
#      names.add(robot_name)
#    return names
#
#  def get_default_planning_options(self):
#    return copy.deepcopy(self.default_options)
#
#  def ik_find_diff_solution(self, robot_name, goal, options, position_only=False):
#    self.collision.enable_collision_checking(not options.allow_collisions)
#    robot = self.env.GetRobot(robot_name)
#    manipulator = robot.GetActiveManipulator()
#    qinit = robot.GetActiveDOFValues()
#    qrobot = np.array(qinit)
#    if goal.shape == (4,4):
#      goal_pose = orpy.poseFromMatrix(goal)
#    else:
#      goal_pose = goal
#    found = False
#    for i in range(0, options.ik_max_iterations):
#      # Get current robot pose
#      with self.env:
#        robot.SetActiveDOFValues(qrobot)
#        robot_pose = manipulator.GetTransformPose()
#      if position_only:
#        found = np.allclose(robot_pose[-3:], goal_pose[-3:], atol=options.ik_tolerance, rtol=0)
#      else:
#        found = np.allclose(robot_pose, goal_pose, atol=options.ik_tolerance, rtol=0)
#      if found:
#        break
#      if position_only:
#        qd = self.compute_velocity_from_position(robot_name, goal_pose[-3:], qrobot=qrobot, enforce_limits=False)
#      else:
#        qd = self.compute_velocity_from_pose(robot_name, goal_pose, qrobot=qrobot, enforce_limits=False)
#      qrobot += (options.ik_alpha * qd)
#    self.logger.logdebug('Diff IK solver run {0} iterations. Solution found: {1}'.format(i+1, found))
#    with self.env:
#      robot.SetActiveDOFValues(qinit)
#    if found:
#      # Check for collisions
#      if not options.allow_collisions:
#        if self.collision.check_robot_collisions(robot_name, qrobot) != CollisionManager.NO_COLLISION:
#          return None
#      # Return the solution
#      return qrobot
#    else:
#      return None
#
#  def ik_find_solution(self, robot_name, goal, options):
#    ikparam = self.ik_valid_parameterization(robot_name, goal)
#    if ikparam is None:
#      return None
#    robot = self.env.GetRobot(robot_name)
#    qinit = robot.GetActiveDOFValues()
#    manipulator = robot.GetActiveManipulator()
#    self.collision.enable_collision_checking(not options.allow_collisions)
#    sols = manipulator.FindIKSolutions(ikparam, orpy.IkFilterOptions.CheckEnvCollisions)
#    if   options.ikcriterion == PlanningOptions.BEST_INDEX:
#      qsol = self.ik_get_best_solution(robot, sols, options.only_translation)
#    elif options.ikcriterion == PlanningOptions.CLOSEST:
#      weights = robot.GetDOFWeights(self.dofindices[robot_name])
#      qsol = self.ik_get_closest_solution(sols, qseed=qinit, weights=weights)
#    with self.env:
#      robot.SetActiveDOFValues(qinit)
#    return qsol
#
#  def ik_get_best_solution(self, robot, solutions, only_translation=True):
#    manipulator = robot.GetActiveManipulator()
#    bestindex = float('-inf')
#    qsol = None
#    for sol in solutions:
#      with self.env:
#        robot.SetActiveDOFValues(sol)
#        if only_translation:
#          J = manipulator.CalculateJacobian()
#        else:
#          J = np.zeros((6,6))
#          J[:3,:] = manipulator.CalculateJacobian()
#          J[3:,:] = manipulator.CalculateAngularVelocityJacobian()
#      index = np.sqrt(np.linalg.det(np.dot(J, J.T)))
#      if index > bestindex:
#        bestindex = index
#        qsol = np.array(sol)
#    return qsol
#
#  def ik_get_closest_solution(self, solutions, qseed, weights=None):
#    if len(solutions) > 0:
#      distances = [sum(weights*(qseed-qsol)**2) for qsol in solutions]
#      closest = np.argmin(distances)
#      return solutions[closest]
#    else:
#      return None
#
#  def ik_valid_parameterization(self, robot_name, goal):
#    target = goal
#    iktype = self.iktype[robot_name]
#    if iktype == orpy.IkParameterizationType.TranslationDirection5D:
#      if type(goal) is not orpy.Ray:
#        target = criros.conversions.to_ray(goal)
#    elif iktype == orpy.IkParameterizationType.Transform6D:
#      if type(goal) is orpy.Ray:
#        target = criros.conversions.from_ray(goal)
#    else:
#      return None
#    return orpy.IkParameterization(target, iktype)
#
#  def check_plan_consistency(self, plan):
#    # Check plan is valid
#    if not plan.is_valid():
#      self.logger.logdebug('Given plan is not valid')
#      return False
#    # Check robot names are correct
#    if not self.get_available_robots().issuperset( plan.get_robot_names() ):
#      self.logger.logdebug('Cannot find all the robots in OpenRAVE: {0}'.format(list(plan.get_robot_names())))
#      return False
#    return True
#
#  def is_valid(self):
#    return hasattr(self, 'env')
#
#  def merge_default_options(self, options):
#    empty_msg = PlanningOptions()
#    merged_msg = copy.deepcopy(options)
#    for member in PlanningOptions.__slots__:
#      member_type = type(getattr(empty_msg, member))
#      if getattr(empty_msg, member) == getattr(options, member):
#        newvalue = getattr(self.default_options, member)
#      else:
#        newvalue = getattr(options, member)
#      setattr(merged_msg, member, member_type(newvalue))
#    # Check that planner is valid
#    if merged_msg.planner.lower() not in [x.lower() for x in self.PLANNERS]:
#      self.logger.logwarn('Invalid planner [{0}], using default planner: {1}'.format(merged_msg.planner, self.default_options.planner))
#      merged_msg.planner = self.default_options.planner
#    # Check that pp_planner is valid
#    if merged_msg.pp_planner.lower() not in [x.lower() for x in self.PP_PLANNERS]:
#      self.logger.logwarn('Invalid pp_planner [{0}], using default pp_planner: {1}'.format(merged_msg.pp_planner, self.default_options.pp_planner))
#      merged_msg.pp_planner = self.default_options.pp_planner
#    # Crop velocity and acceleration factors
#    merged_msg.velocity_factor = max(0.01, min(merged_msg.velocity_factor, 1.0))
#    merged_msg.acceleration_factor = max(0.01, min(merged_msg.acceleration_factor, 1.0))
#    return merged_msg
#
#  def plan_closed_chain(self, req):
#    plan = MotionPlan()
#    robots = []
#    qstarts = []
#    qgrippers = []
#    # Get robots
#    for robot_name in req.robot_names:
#      robot = self.env.GetRobot(robot_name)
#      robots.append(robot)
#      qstarts.append(robot.GetActiveDOFValues())
#      gripper_index = robot.GetActiveManipulator().GetGripperIndices()[0]
#      qgrippers.append(robot.GetDOFValues()[gripper_index])
#    obj = self.env.GetKinBody(req.obj_name)
#    # Get robot goal states
#    IK_indices = []
#    for IK_index in req.goal_IK_indices:
#      if IK_index == -1:
#        IK_indices.append(None)
#      else:
#        IK_indices.append(IK_index)
#    T_obj_start = obj.GetTransform()
#    T_obj_goal = criros.conversions.from_pose(req.obj_goal_pose)
#    qgoals = bimanual_utils.compute_bimanual_goal_configs(
#              robots, obj, qstarts, qgrippers, T_obj_start, T_obj_goal,
#              IK_indices=IK_indices)
#    # Get obj translational limits
#    obj_translation_limits = [[req.obj_translation_limits[0].x,
#                               req.obj_translation_limits[0].y,
#                               req.obj_translation_limits[0].z],
#                              [req.obj_translation_limits[1].x,
#                               req.obj_translation_limits[1].y,
#                               req.obj_translation_limits[1].z]]
#    # Set velocity
#    for robot in robots:
#      robot.SetDOFVelocityLimits(bimanual_utils.get_denso_limits()[0]*req.velocity_factor)
#    # Start planning
#    ccplanner = ccp.CCPlanner(obj, robots, logger=self.logger,
#                              planner_type=req.planner_type)
#    ccquery = ccp.CCQuery(obj_translation_limits, qstarts, qgoals, qgrippers,
#                          T_obj_start, nn=req.nn, step_size=req.step_size,
#                          velocity_scale=req.velocity_factor,
#                          enable_bw=req.enable_bw)
#    if not ccplanner.set_query(ccquery):
#      return None
#    if not ccplanner.solve(timeout=req.timeout):
#      return None
#    ccplanner.shortcut(ccquery, maxiter=req.smooth_iter)
#    data = dict()
#    data['planner']          = ccplanner
#    data['trajectory']       = ccquery.cctraj
#    data['obj']              = req.obj_name
#    data['force_threshold']  = req.force_threshold
#    data['torque_threshold'] = req.torque_threshold
#    plan.add_step(MotionPlan.CLOSED_CHAIN, req.robot_names, data)
#    return plan
#
#  def plan_insertion(self, req):
#    # Merge default planning options
#    req.options = self.merge_default_options(req.options)
#    req.options_grasping = self.merge_default_options(req.options_grasping)
#    req.options_inserting = self.merge_default_options(req.options_inserting)
#    # Plan
#    plan = MotionPlan()
#    if req.bimanual:
#      bimanual_config = self.find_bimanual_insertion_configuration(req)
#      if bimanual_config is None:
#        self.logger.logwarn('Failed to find a valid bimanual insertion configuration')
#        return None
#      approach_traj = self.bimanual_trajectory(bimanual_config, req.options)
#      if approach_traj is None:
#        self.logger.logwarn('Failed to plan bimanual motion.')
#        return None
#      robot_names = [config[0] for config in bimanual_config]
#      plan.add_step(MotionPlan.MULTIPLE_ROBOTS, robot_names, approach_traj, wait=True)
#      # Insertion parameters
#      data = dict()
#      keys  = ['body_with_hole', 'hole_index', 'body_to_insert', 'standoff', 'depth', 'num_pins']
#      keys += ['contact_velocity', 'contact_force_threshold', 'contact_force_tracked', 'hole_force_threshold']
#      keys += ['spiral_pitch', 'spiral_velocity', 'max_spiral_radius', 'insert_velocity', 'insert_force_threshold']
#      keys += ['save_force_log']
#      for key in keys:
#        if hasattr(req, key):
#          data[key] = getattr(req, key)
#      use_support = bimanual_config[0][2]
#      data['use_support'] = bool(use_support)
#      if use_support:
#        extra_offset = bimanual_config[0][3]
#        data['standoff'] += float(extra_offset)
#      plan.add_step(MotionPlan.INSERT_PIN, robot_names, data, wait=True)
#    return plan
#
#  def plan_pick(self, req):
#    # Merge default planning options
#    req.options = self.merge_default_options(req.options)
#    # Check specified manipulator is valid
#    manip_tuple = (req.robot_name, req.options.manipulator, req.options.iktype)
#    if not self.change_active_manipulator(*manip_tuple):
#      self.logger.logwarn('Cannot change the active manipulator to (robot, manip, iktype): {0}'.format(manip_tuple))
#      return None
#    # Get OpenRAVE objects we will work with
#    robot = self.env.GetRobot(req.robot_name)
#    body = self.env.GetKinBody(req.body_name)
#    manipulator = robot.GetActiveManipulator()
#    qstart = robot.GetActiveDOFValues()
#    # Set the gripper opening before starting the planning
#    if not np.isclose(req.gripper_opening, 0.0):
#      self.set_gripper_distance(req.robot_name, req.gripper_opening)
#    # Find a valid grasp
#    grasp = req.grasp
#    if np.isclose(grasp.slide_step, 0):
#      slide_deltas = [0]
#    else:
#      if grasp.slide_both_directions:
#        slide_deltas = np.arange(-grasp.max_sliding_distance, grasp.max_sliding_distance, grasp.slide_step)
#        slide_deltas = sorted(slide_deltas, key=abs)
#      else:
#        slide_deltas = np.arange(0, grasp.max_sliding_distance, grasp.slide_step)
#    direction = criros.conversions.from_vector3(grasp.approach_direction)
#    approach_directions = [direction]
#    if grasp.opposite_approach_direction:
#      approach_directions.append(-direction)
#    slide_direction = criros.conversions.from_vector3(grasp.slide_direction)
#    found = False
#    for approachdir,delta in itertools.product(approach_directions, slide_deltas):
#      # approachdir is in the body frame
#      Tbody = body.GetTransform()
#      # approach_direction is in the world frame
#      approach_direction = np.dot(Tbody[:3,:3], approachdir)
#      # Key poses
#      Tgrasp = rave_utils.ComputeGraspTransform(body, grasp.link_index, approachdir, grasp.depth, slide_direction, delta)
#      #~ self.draw_axes(Tgrasp)
#      Tapproach = np.array(Tgrasp)
#      Tapproach[:3,3] -= req.approach_distance*approach_direction
#      retreat_direction = tr.unit_vector(criros.conversions.from_vector3(req.retreat_direction))
#      Tretreat = np.array(Tgrasp)
#      Tretreat[:3,3] += req.retreat_distance*retreat_direction
#      # Check IK solution for the grasp pose
#      qgrasp = self.ik_find_solution(req.robot_name, Tgrasp, req.options)
#      if qgrasp is None:
#        continue
#      # Straight line from grasp -> approach pose
#      waypoints_grasp = []
#      if not np.isclose(0, req.approach_distance):
#        waypoints_grasp = self.waypoints_straight_line(req.robot_name, qgrasp, Tapproach[:3,3], req.options)
#        if waypoints_grasp is None:
#          continue
#        waypoints_grasp.reverse()   # Need to reverse because we want approach -> grasp
#        qapproach = waypoints_grasp[0]
#      else:
#        qapproach = qgrasp
#      # Straight line to the retreat pose
#      if not np.isclose(0, req.retreat_distance):
#        waypoints_retreat = self.waypoints_straight_line(req.robot_name, qgrasp, Tretreat[:3,3], req.options)
#        if waypoints_retreat is None:
#          continue
#      else:
#        waypoints_retreat = [qgrasp] * 2
#      # For the trajectory from qstart to qgrasp we need the waypoints from qstart to qapproach
#      traj_start_approach = self.trajectory_without_postprocessing(req.robot_name, qstart, qapproach, req.options)
#      if traj_start_approach is None:
#        continue
#      waypoints = traj_start_approach.GetAllWaypoints2D().tolist()
#      waypoints += waypoints_grasp
#      traj_grasp = self.trajectory_through_waypoints(req.robot_name, waypoints, req.options)
#      if traj_grasp is None:
#        continue
#      traj_retreat = self.trajectory_through_waypoints(req.robot_name, waypoints_retreat, req.options)
#      if traj_retreat is None:
#        continue
#      found = True
#      break
#    # Populate the plan, if found
#    if found:
#      wait = True
#      plan = MotionPlan()
#      # Need to add 2 extra millimeter for the pins
#      opening = req.gripper_opening + 0.003
#      plan.add_step(MotionPlan.COMMAND_GRIPPER, req.robot_name, opening,
#                                                                wait=True)
#      plan.add_step(MotionPlan.TRAJECTORY,        req.robot_name,
#                                              traj_grasp,     wait=True)
#      plan.add_step(MotionPlan.CLOSE_GRIPPER,     req.robot_name, req.
#                                                  body_name,  wait=True)
#      plan.add_step(MotionPlan.TRAJECTORY,        req.robot_name,
#                                              traj_retreat,   wait=True)
#    else:
#      plan = None
#      self.logger.logdebug('Failed to plan pick: {0}'.format(req.body_name))
#    return plan
#
#  def plan_bimanual_pick(self, req):
#    # Merge default planning options
#    req.options = self.merge_default_options(req.options)
#    # Check specified manipulator is valid
#    for robot_name in req.robot_names:
#      manip_tuple = (robot_name, req.options.manipulator, req.options.iktype)
#      if not self.change_active_manipulator(*manip_tuple):
#        self.logger.logwarn('Cannot change the active manipulator to (robot, manip, iktype): {0}'.format(manip_tuple))
#        return None
#    # Get OpenRAVE objects we will work with
#    body = self.env.GetKinBody(req.body_name)
#    robots = []
#    qstarts = []
#    bimanual_config = []
#    # Compute goal configurations for picking
#    for robot_name, opening, qgrasp in zip(req.robot_names, req.gripper_openings, req.qgrasps):
#      robot = self.env.GetRobot(robot_name)
#      robots.append(robot)
#      qstarts.append(robot.GetActiveDOFValues())
#      # Set the gripper opening
#      if not np.isclose(opening, 0.0):
#        self.set_gripper_distance(robot_name, opening)
#      Tgoal = rave_utils.ComputeTGripper2(body, qgrasp[0], qgrasp)
#      qgoal = self.ik_find_solution(robot_name, Tgoal, req.options)
#      if qgoal is None:
#        self.logger.logerr('Failed to find goal IK for robot: {0}'.format(robot_name))
#        return None
#      bimanual_config.append([robot_name, qgoal])
#    # Plan bimanual trajectory
#    bimanual_traj = self.bimanual_trajectory(bimanual_config, req.options)
#    if bimanual_traj is None:
#      self.logger.logwarn('Failed to plan bimanual picking motion.')
#      return None
#    # Generate a new plan
#    plan = MotionPlan()
#    for robot_name, opening in zip(req.robot_names, req.gripper_openings):
#      plan.add_step(MotionPlan.COMMAND_GRIPPER, robot_name, opening, wait=True)
#    plan.add_step(MotionPlan.MULTIPLE_ROBOTS, req.robot_names, bimanual_traj, wait=True)
#    for robot_name, opening in zip(req.robot_names, req.gripper_openings):
#      plan.add_step(MotionPlan.CLOSE_GRIPPER, robot_name, None, wait=True)
#    return plan
#
#  def plan_to_joints(self, req):
#    robot = self.env.GetRobot(req.robot_name)
#    # Merge default planning options
#    options = self.merge_default_options(req.options)
#    # Check specified manipulator is valid
#    manip_tuple = (req.robot_name, options.manipulator, options.iktype)
#    if not self.change_active_manipulator(*manip_tuple):
#      self.logger.logwarn('Cannot change the active manipulator to (robot, manip, iktype): {0}'.format(manip_tuple))
#      return None
#    # Set start state
#    self.set_start_state(req.robot_name, req.start_state)
#    # Sort the goal state msg
#    goal_state = criros.utils.sorted_joint_state_msg(req.goal_state, self.joint_names[req.robot_name])
#    if len(goal_state.name) != robot.GetActiveDOF():
#      self.logger.logwarn('Invalid number of joints in goal_state: {0}'.format(len(goal_state.name)))
#      return None
#    # Get start and goal joint values
#    qstart = robot.GetActiveDOFValues()
#    qgoal = np.array(goal_state.position)
#    # Plan
#    if np.allclose(qstart, qgoal):
#      # Already at the desired goal. Do nothing
#      return MotionPlan()
#    traj = self.trajectory_without_postprocessing(req.robot_name, qstart,
#                                                            qgoal, options)
#    if traj is None:
#      return None
#    traj = self.trajectory_through_waypoints(req.robot_name, traj.GetAllWaypoints2D(), options)
#    if traj is None:
#      return None
#    plan = MotionPlan()
#    plan.add_step(MotionPlan.TRAJECTORY, req.robot_name, traj, wait=True)
#    return plan
#
#  def plan_to_pose(self, req):
#    robot = self.env.GetRobot(req.robot_name)
#    # Merge default planning options
#    options = self.merge_default_options(req.options)
#    # Check specified manipulator is valid
#    manip_tuple = (req.robot_name, options.manipulator, options.iktype)
#    if not self.change_active_manipulator(*manip_tuple):
#      self.logger.logwarn('Cannot change the active manipulator to (robot, manip, iktype): {0}'.format(manip_tuple))
#      return None
#    # Set start state
#    self.set_start_state(req.robot_name, req.start_state)
#    # Find IK solution
#    Tgoal = criros.conversions.from_pose(req.goal_pose)
#    qgoal = self.ik_find_solution(robot.GetName(), Tgoal, options)
#    if qgoal is None:
#      return None
#    # Plan to joints
#    joints_req = JointGoalRequest()
#    joints_req.robot_name = req.robot_name
#    joints_req.start_state = copy.deepcopy(req.start_state)
#    joints_req.goal_state.name = joints_req.start_state.name
#    joints_req.goal_state.position = qgoal.tolist()
#    return self.plan_to_joints(joints_req)
#
#  def set_obj_pose(self, obj_name, pose):
#    obj = self.env.GetKinBody(obj_name)
#    if self.env_manager is not None:
#      self.env_manager.update_rave_environment()
#    with self.env:
#      obj.SetTransform(criros.conversions.from_pose(pose))
#    return
#
#  def set_start_state(self, robot_name, start_state):
#    robot = self.env.GetRobot(robot_name)
#    # Sort the start state msg
#    start_state = criros.utils.sorted_joint_state_msg(start_state, self.joint_names[robot_name])
#    # If we have a valid start_state, move the robot to the start configuration
#    if len(start_state.name) == robot.GetActiveDOF():
#      qstart = np.array(start_state.position)
#      with self.env:
#        robot.SetActiveDOFValues(qstart)
#    else:
#      if self.env_manager is not None:
#        self.env_manager.update_rave_joints()
#    return True
#
#  def set_gripper_distance(self, robot_name, distance):
#    robot = self.env.GetRobot(robot_name)
#    indices = robot.GetActiveManipulator().GetGripperIndices()
#    # We support only 1 DOF grippers
#    if len(indices) > 1:
#      return False
#    min_distance = self.gripper_parameters['min_gap_distance']
#    max_distance = self.gripper_parameters['max_gap_distance']
#    min_angle = self.gripper_parameters['min_gap_angle']
#    max_angle = self.gripper_parameters['max_gap_angle']
#    xp = [min_distance, max_distance]
#    fp = [min_angle, max_angle]
#    gripper_angle = np.clip(np.interp(distance, xp, fp), min(fp), max(fp))
#    robot.SetDOFValues([gripper_angle], dofindices=indices)
#    return True
#
#  def trajectory_through_waypoints(self, robot_name, waypoints, options):
#    num_points = len(waypoints)
#    if num_points < 2:
#      self.logger.logwarn('At least 2 waypoints are required to generate a trajectory. Given: {0}'.format(num_points))
#      return None
#    robot = self.env.GetRobot(robot_name)
#    # Merge default planning options
#    options = self.merge_default_options(options)
#    # Enable/disable collision checking
#    self.collision.enable_collision_checking(not options.allow_collisions)
#    # Insert waypoints
#    traj = orpy.RaveCreateTrajectory(self.env, '')
#    traj.Init(robot.GetActiveConfigurationSpecification())
#    for i,waypoint in enumerate(waypoints):
#      traj.Insert(i, waypoint)
#    # Apply velocity and acceleration factors
#    robot.SetDOFVelocityLimits(self.velocity_limits[robot_name]*options.velocity_factor)
#    robot.SetDOFAccelerationLimits(self.acceleration_limits[robot_name]*options.acceleration_factor)
#    # Populate planner parameters
#    params = orpy.Planner.PlannerParameters()
#    params.SetRobotActiveJoints(robot)
#    params.SetMaxIterations(options.pp_iterations)
#    params.SetPostProcessing('', '')
#    # Generate the trajectory
#    planner = orpy.RaveCreatePlanner(self.env, options.pp_planner)
#    success = planner.InitPlan(robot, params)
#    if not success:
#      self.logger.logwarn('Failed to plan trajectory through waypoints.')
#      return None
#    status = planner.PlanPath(traj)
#    if status != orpy.PlannerStatus.HasSolution:
#      self.logger.logwarn('Failed to plan trajectory through waypoints.')
#      return None
#    return traj
#
#  def trajectory_without_postprocessing(self, robot_name, qstart, qgoal, options):
#    # Merge default planning options
#    options = self.merge_default_options(options)
#    # Enable/disable collision checking
#    self.collision.enable_collision_checking(not options.allow_collisions)
#    # Set start configuration
#    robot = self.env.GetRobot(robot_name)
#    with self.env:
#      robot.SetActiveDOFValues(qstart)
#    # Apply velocity and acceleration factors
#    robot.SetDOFVelocityLimits(self.velocity_limits[robot_name]*options.velocity_factor)
#    robot.SetDOFAccelerationLimits(self.acceleration_limits[robot_name]*options.acceleration_factor)
#    # Populate planner parameters
#    params = orpy.Planner.PlannerParameters()
#    params.SetRobotActiveJoints(robot)
#    params.SetGoalConfig(qgoal)
#    params.SetMaxIterations(options.max_iterations)
#    params.SetPostProcessing('', '')
#    # Generate the trajectory
#    planner = orpy.RaveCreatePlanner(self.env, options.planner)
#    success = planner.InitPlan(robot, params)
#    if not success:
#      self.logger.logwarn('Goal configuration fails constraints.')
#      return None
#    traj = orpy.RaveCreateTrajectory(self.env, '')
#    status = planner.PlanPath(traj)
#    if status != orpy.PlannerStatus.HasSolution:
#      self.logger.logwarn('Failed to plan trajectory without postprocessing.')
#      return None
#    return traj
#
  def update_link_stats(self, robot_name):
    """
    Opens the robot's C{LinkStatisticsModel} database and updates the robot weights and resolutions.
    """
    robot = self.env.GetRobot(robot_name)
    statsmodel = orpy.databases.linkstatistics.LinkStatisticsModel(robot)
    if statsmodel.load():
      statsmodel.setRobotWeights()
      statsmodel.setRobotResolutions(xyzdelta=0.01)
    else:
      manip_name = robot.GetActiveManipulator().GetName()
      self.logger.logwarn('Link Statistics database was not found for robot: {0}, manipulator: {1}'.format(robot_name, manip_name))
      robot.SetDOFWeights([1]*robot.GetDOF())

#  def visualize_plan(self, plan):
#    # Check we have a valid plan
#    if not self.check_plan_consistency(plan):
#      return False
#    # Visualize
#    for step in plan:
#      self.visualize_step(step)
#    return True
#
#  def visualize_step(self, step, wait=None):
#    # Override waiting for the step
#    if wait is None:
#      wait = step.wait
#    # Get robots from the OpenRAVE environment
#    if step.category in [MotionPlan.CLOSED_CHAIN, MotionPlan.MULTIPLE_ROBOTS, MotionPlan.INSERT_PIN]:
#      robot_names = step.robots
#    else:
#      robot_names = [step.robot]
#    robots = [self.env.GetRobot(name) for name in robot_names]
#    # Visualize
#    robot = robots[0]
#    taskmanip = orpy.interfaces.TaskManipulation(robot)
#    if step.category == MotionPlan.CLOSE_GRIPPER:
#      taskmanip.CloseFingers()
#      robot.WaitForController(0)
#      if step.body_name is not None:
#        body = self.env.GetKinBody(step.body_name)
#        robot.Grab(body)
#    elif step.category == MotionPlan.COMMAND_GRIPPER:
#      self.set_gripper_distance(step.robot, step.command)
#    elif step.category == MotionPlan.INSERT_PIN:
#      body_to_insert = self.env.GetKinBody(step.body_to_insert)
#      body_with_hole = self.env.GetKinBody(step.body_with_hole)
#      # Release the pin
#      robot = robots[1]
#      self.set_gripper_distance(robot.GetName(), 0.025)
#      robot.Release(body_to_insert)
#      # Insert it to the hole in OpenRAVE
#      Tpin = body_to_insert.GetTransform()
#      Thole_rel = self.holes[step.body_with_hole][step.hole_index]
#      Tbody = body_with_hole.GetTransform()
#      Thole = np.dot(Tbody, Thole_rel)
#      Tpin[:3,3] = Thole[:3,3]
#      body_to_insert.SetTransform(Tpin)
#      # Attach it to the robot holding the body_with_hole
#      for robot in self.env.GetRobots():
#        if body_with_hole in robot.GetGrabbed():
#          robot.Grab(body_to_insert)
#          break
#    elif step.category == MotionPlan.MULTIPLE_ROBOTS:
#      robots[0].GetController().SetPath(step.trajectory)
#      robots[1].GetController().SetPath(step.trajectory)
#      if wait:
#        robots[0].WaitForController(0)
#        robots[1].WaitForController(0)
#    elif step.category == MotionPlan.OPEN_GRIPPER:
#      taskmanip.ReleaseFingers()
#      robot.WaitForController(0)
#      robot.ReleaseAllGrabbed()
#    elif step.category == MotionPlan.TRAJECTORY:
#      robot.GetController().SetPath(step.trajectory)
#      if wait:
#        robot.WaitForController(0)
#    elif step.category == MotionPlan.CLOSED_CHAIN:
#      ccplanner = step.planner
#      cctraj = step.trajectory
#      ccplanner.visualize_cctraj(cctraj)
#
#  def waypoints_straight_line(self, robot_name, qstart, goal_point, options):
#    # Enable/disable collision checking
#    self.collision.enable_collision_checking(not options.allow_collisions)
#    # Get initial configuration
#    robot = self.env.GetRobot(robot_name)
#    manipulator = robot.GetActiveManipulator()
#    qinit = robot.GetActiveDOFValues()
#    with self.env:
#      robot.SetActiveDOFValues(qstart)
#    start_pose = manipulator.GetTransformPose()
#    # Sample the XYZ to get to the goal position
#    distance = goal_point - start_pose[-3:]
#    approachdir = tr.unit_vector(distance)
#    norm = tr.vector_norm(distance)
#    if norm > options.max_distance:
#      self.logger.logwarn('Max distance exceeded for straight line generation: {0:.3f}'.format(norm))
#      with self.env:
#        robot.SetActiveDOFValues(qinit)
#      return None
#    num_points = int( np.ceil(norm/options.step_size) )
#    pose = np.array(start_pose)
#    waypoints = [qstart]
#    for i, dist in enumerate( np.linspace(options.step_size, norm, num=num_points, endpoint=True) ):
#      pose[-3:] = start_pose[-3:] + (dist*approachdir)
#      qseed = waypoints[-1]
#      with self.env:
#        robot.SetActiveDOFValues(qseed)
#      qsol = self.ik_find_diff_solution(robot_name, pose, options=options)
#      if qsol is None:
#        self.logger.logwarn('Failed to find IK solution at distance: {0:.3f}'.format(dist))
#        waypoints = None
#        break
#      else:
#        waypoints.append(np.array(qsol))
#    with self.env:
#      robot.SetActiveDOFValues(qinit)
#    return waypoints
