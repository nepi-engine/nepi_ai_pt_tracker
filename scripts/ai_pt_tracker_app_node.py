#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Process Script. 
# 1. Waits for ai detection topic
# 2. Adjust LED level based on target location in image

# Requires the following additional scripts are running
# a)ai_detector_config_script.py
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)



import os
#### ROS namespace setup
NEPI_BASE_NAMESPACE = '/nepi/s2x/'
os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1] # remove to run as automation script
import rospy
import time
import sys
import statistics
import numpy as np
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_msg

from std_msgs.msg import UInt8, Empty, Float32
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import PanTiltLimits, PanTiltPosition, SingleAxisTimedMove, PanTiltStatus, StringArray
from nepi_ros_interfaces.msg import AiTargetingStatus, TargetLocalization, TargetLocalizations, ObjectCount
from nepi_app_ai_pt_tracker.msg import AiPtTrackingStatus, TrackingErrors

#from nepi_app_ai_pt_traker.msg import AiPtTrackerStatus



#########################################
# Node Class
#########################################

class pantilt_object_tracker(object):
  PTX_MAX_TRACK_SPEED_RATIO = 1.0
  PTX_MIN_TRACK_SPEED_RATIO = 0.1
  PTX_OBJ_CENTERED_BUFFER_RATIO = 0.15 # Hysteresis band about center of image for tracking purposes


# Pan and Tilt tracking settings

  PTX_OBJECT_TILT_OFFSET_RATIO = 0.15 # Adjust tilt center to lower or raise the calculated object center
  PTX_OBJ_CENTERED_BUFFER_RATIO = 0.15 # Hysteresis band about center of image for tracking purposes

  DEFAULT_CLASS = "person"
  DEFAULT_SCAN_TIME = 5.0
  DEFAULT_MIN_AREA_RATIO = 0.01 # Filters background targets.
  DEFAULT_SCAN_SPEED_RATIO = 0.5
  DEFAULT_SCAN_TILT_DEG = 0.15
  DEFAULT_SCAN_PAN_DEGS = 80
  DEFAULT_TRACK_SPEED_RATIO = 0.6
  DEFAULT_TRACK_TILT_OFFSET_DEG = 0.5

  MIN_MAX_ERROR_GOAL = [1,20]
  DEFAULT_ERROR_GOAL_DEG = 5

  STATUS_UPDATE_TIME = 1
  PTX_UPDATE_TIME = 1

  MAX_SCAN_TIME = 10
  TRACK_JOG_TIME = 0.5
  
  data_products = ["tracking_image"]

  classifier_running = False
  targeting_running = False

  targeting_status_msg = None

  current_image_topic = ""
  last_image_topic = ""
  ros_message_img = None

  classes_list = []
  target_detected = False

  pt_connected = False
  has_position_feedback = False
  pt_status_msg = None
  pan_scan_direction = 1 # Keep track of current scan direction (1: Positive Limit, -1: Negative Limit)
  last_object_pan_ratio=0
  total_tilt_degs = 90
  current_tilt_ratio = 0.5
  total_pan_degs = 90
  current_pan_ratio = 0.5
  last_pt_device = ""

  pt_status_msg = None

  is_running = False
  is_scanning = False
  is_tracking = False

  pitch_yaw_errors_deg = [0.0,0.0]


  pos_scan_dir = 1
  timed_scan_dir = 1

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "pantilt_object_tracker" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    
    # Message Image to publish when detector not running
    message = "WAITING FOR AI TARGETING TO START"
    cv2_img = nepi_img.create_message_image(message)
    self.ros_message_img = nepi_img.cv2img_to_rosimg(cv2_img)


    # Setup Node Publishers
    self.status_pub = rospy.Publisher("~status", AiPtTrackingStatus, queue_size=1, latch=True)
    self.errors_pub = rospy.Publisher("~errors", TrackingErrors, queue_size=1, latch=True)
    self.image_pub = rospy.Publisher("~tracking_image",Image,queue_size=1, latch = True)
    time.sleep(1)
    self.ros_message_img.header.stamp = nepi_ros.time_now()
    self.image_pub.publish(self.ros_message_img)


    # Class Subscribers
    ## App Setup ########################################################
    app_reset_app_sub = rospy.Subscriber('~reset_app', Empty, self.resetAppCb, queue_size = 10)
    self.initParamServerValues(do_updates=False)

    # App Specific Subscribers
    rospy.Subscriber('~select_class', String, self.setClassCb, queue_size = 10)

    set_image_input_sub = rospy.Subscriber('~use_live_image', Bool, self.setImageLiveCb, queue_size = 10)
    set_image_delay_sub = rospy.Subscriber('~use_last_image', Bool, self.setImageLastCb, queue_size = 10)

    rospy.Subscriber('~select_pantilt', String, self.setPtTopicCb, queue_size = 10)
    rospy.Subscriber("~set_scan_time", Float32, self.settimedScanCb, queue_size = 10)
    rospy.Subscriber("~set_min_area_ratio", Float32, self.setMinAreaCb, queue_size = 10)
    rospy.Subscriber("~set_scan_speed_ratio", Float32, self.setScanSpeedCb, queue_size = 10)
    rospy.Subscriber("~set_scan_tilt_offset", Float32, self.setScanTiltOffsetCb, queue_size = 10)
    rospy.Subscriber("~set_scan_pan_angle", Float32, self.setScanPanAngleCb, queue_size = 10)
    rospy.Subscriber("~set_track_speed_ratio", Float32, self.setTrackSpeedCb, queue_size = 10)
    rospy.Subscriber("~set_track_tilt_offset", Float32, self.setTrackTiltOffsetCb, queue_size = 10)

    rospy.Subscriber("~set_error_goal_deg", Float32, self.setErrorGoalCb, queue_size = 10)

    rospy.Subscriber('~enable_tracker', Bool, self.enableTrackerCb)
  

    # Reset Params



    ## Class subscribers
    # AI Targeting Subscriber Topics

    AI_TARGETING_STATUS_TOPIC = self.base_namespace + "app_ai_targeting/status"
    nepi_msg.publishMsgInfo(self,"Waiting for Targeting Msg: " + AI_TARGETING_STATUS_TOPIC )
    nepi_ros.wait_for_topic(AI_TARGETING_STATUS_TOPIC)
    rospy.Subscriber(AI_TARGETING_STATUS_TOPIC, AiTargetingStatus, self.targetingStatusCb, queue_size = 1)

    AI_FOUND_TARGET_TOPIC = self.base_namespace + "app_ai_targeting/target_count"
    nepi_msg.publishMsgInfo(self,"Waiting for Targeting Msg: " + AI_FOUND_TARGET_TOPIC )
    nepi_ros.wait_for_topic(AI_FOUND_TARGET_TOPIC)
    rospy.Subscriber(AI_FOUND_TARGET_TOPIC, ObjectCount, self.foundTargetCb, queue_size = 1)
   

    AI_TARGET_LOCS_TOPIC = self.base_namespace + "app_ai_targeting/targeting_localizations"
    rospy.Subscriber(AI_TARGET_LOCS_TOPIC, TargetLocalizations, self.ptTrackCb, queue_size = 1)


    #######################
    # Wait for ptx pt_status_msg topic to publish
    pt_status_msg_topic = "/ptx/pt_status_msg"
    nepi_msg.publishMsgInfo(self,"Waiting for topic name: " + pt_status_msg_topic)
    pt_topic=nepi_ros.wait_for_topic(pt_status_msg_topic)
    PTX_NAMESPACE = (pt_topic.rpartition("ptx")[0] + "ptx/")
    nepi_msg.publishMsgInfo(self,"Found ptx namespace: " + PTX_NAMESPACE)
    # PanTilt Status Topics
    PTX_GET_STATUS_TOPIC = PTX_NAMESPACE + "pt_status_msg"
    # PanTilt Control Publish Topics
    PTX_SET_SPEED_RATIO_TOPIC = PTX_NAMESPACE + "set_speed_ratio"
    PTX_GOHOME_TOPIC = PTX_NAMESPACE + "go_home"
    PTX_STOP_TOPIC = PTX_NAMESPACE + "stop_moving"
    PTX_GOTO_PAN_RATIO_TOPIC = PTX_NAMESPACE + "jog_to_yaw_ratio"
    PTX_GOTO_TILT_RATIO_TOPIC = PTX_NAMESPACE + "jog_to_pitch_ratio"


    PTX_JOG_PAN_TOPIC = PTX_NAMESPACE + "jog_timed_yaw"
    PTX_JOG_TILT_TOPIC = PTX_NAMESPACE + "jog_timed_pitch"

    PTX_SET_SOFT_LIMITS_TOPIC = PTX_NAMESPACE + "set_soft_limits"

    ## Create Class Publishers
    self.send_pt_home_pub = rospy.Publisher(PTX_GOHOME_TOPIC, Empty, queue_size=10)
    self.set_pt_speed_ratio_pub = rospy.Publisher(PTX_SET_SPEED_RATIO_TOPIC, Float32, queue_size=10)
    self.set_pt_position_pub = rospy.Publisher(jog_to_position, PanTiltPosition, queue_size=10)
    self.set_pt_pan_ratio_pub = rospy.Publisher(PTX_GOTO_PAN_RATIO_TOPIC, Float32, queue_size=10)
    self.set_pt_tilt_ratio_pub = rospy.Publisher(PTX_GOTO_TILT_RATIO_TOPIC, Float32, queue_size=10)
    self.set_pt_pan_jog_pub = rospy.Publisher(PTX_JOG_PAN_TOPIC, Float32, queue_size=10)
    self.set_pt_tilt_jog_pub = rospy.Publisher(PTX_JOG_TILT_TOPIC, Float32, queue_size=10)

    self.set_pt_soft_limits_pub = rospy.Publisher(PTX_SET_SOFT_LIMITS_TOPIC, PanTiltLimits, queue_size=10)
    self.pt_stop_motion_pub = rospy.Publisher(PTX_STOP_TOPIC, Empty, queue_size=10)

    # Set up ai targeting subscriber
    #################################


    nepi_ros.timer(nepi_ros.duration(self.STATUS_UPDATE_TIME), self.updaterCb)
    scan_time = nepi_ros.get_param(self,"~scan_time",self.init_scan_time)
    rospy.Timer(rospy.Duration(scan_time), self.timedScanCb)
    rospy.Timer(rospy.Duration(self.TRACK_JOG_TIME), self.timedTrackCb)

    ## Start Node Processes
    # Set up the timer that start scanning when no objects are detected
    nepi_msg.publishMsgInfo("Setting up pan/tilt scan check timer")
    rospy.Timer(rospy.Duration(self.PTX_UPDATE_TIME), self.ptScanCb)

    ##############################
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self," Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################

  #######################
  ### App Config Functions

  def resetAppCb(self,msg):
    self.resetApp()

  def resetApp(self):
    nepi_ros.set_param(self,"~tracking_enabled",False)

    nepi_ros.set_param(self,'~use_live_image',True)
    nepi_ros.set_param(self,'~use_last_image',True)

    nepi_ros.set_param(self,"~selected_class",self.DEFAULT_CLASS)

    nepi_ros.set_param(self,"~pt_namespace","")
    nepi_ros.set_param(self,"~scan_time",self.DEFAULT_SCAN_TIME)
    nepi_ros.set_param(self,"~min_ratio",self.DEFAULT_MIN_RATIO)
    nepi_ros.set_param(self,"~scan_speed_ratio",self.DEFAULT_SCAN_SPEED_RATIO)
    nepi_ros.set_param(self,"~scan_tilt_offset",self.DEFAULT_SCAN_TILT_ANGLE)
    nepi_ros.set_param(self,"~scan_pan_angle",self.DEFAULT_SCAN_PAN_DEGS)
    nepi_ros.set_param(self,"~track_speed_ratio",self.DEFAULT_TRACK_SPEED_RATIO)
    nepi_ros.set_param(self,"~track_tilt_offset",self.DEFAULT_TRACK_TILT_OFFSET)
    nepi_ros.set_param(self,"~error_goal",self.DEFAULT_ERROR_GOAL)

    nepi_ros.set_param(self,'~running',  False)
    self.publish_status()

  def saveConfigCb(self, msg):  # Just update Class init values. Saving done by Config IF system
    pass # Left empty for sim, Should update from param server

  def setCurrentAsDefault(self):
    self.initParamServerValues(do_updates = False)

  def updateFromParamServer(self):
    #nepi_msg.publishMsgWarn(self,"Debugging: param_dict = " + str(param_dict))
    #Run any functions that need updating on value change
    # Don't need to run any additional functions
    pass

  def initParamServerValues(self,do_updates = True):
    nepi_msg.publishMsgInfo(self," Setting init values to param values")
    self.init_tracking_enabled = nepi_ros.get_param(self,"~tracking_enabled",False)

    self.init_use_live_image = nepi_ros.get_param(self,'~use_live_image',True)
    self.init_use_last_image = nepi_ros.get_param(self,'~use_last_image',True)

    self.init_sel_class = nepi_ros.get_param(self,"~selected_class",self.DEFAULT_CLASS)

    self.init_pt_namespace = nepi_ros.get_param(self,"~pt_namespace","")
    self.init_scan_time = nepi_ros.get_param(self,"~scan_time",self.DEFAULT_SCAN_TIME)
    self.init_min_area_ratio = nepi_ros.get_param(self,"~min_area_ratio",self.DEFAULT_MIN_AREA_RATIO)
    self.init_scan_speed_ratio = nepi_ros.get_param(self,"~scan_speed_ratio",self.DEFAULT_SCAN_SPEED_RATIO)
    self.init_scan_tilt_offset = nepi_ros.get_param(self,"~scan_tilt_offset",self.DEFAULT_SCAN_TILT_ANGLE)
    self.init_scan_pan_angle = nepi_ros.get_param(self,"~scan_pan_angle",self.DEFAULT_SCAN_PAN_DEGS)
    self.init_track_speed_ratio = nepi_ros.get_param(self,"~track_speed_ratio",self.DEFAULT_TRACK_SPEED_RATIO)
    self.init_track_tilt_offset = nepi_ros.get_param(self,"~track_tilt_offset",self.DEFAULT_TRACK_TILT_OFFSET)

    self.init_error_goal = nepi_ros.get_param(self,"~error_goal",self.DEFAULT_ERROR_GOAL)

    self.resetParamServer(do_updates)

  def resetParamServer(self,do_updates = True):
    nepi_ros.set_param(self,"~tracking_enabled",self.init_tracking_enabled)

    nepi_ros.set_param(self,'~use_live_image',self.init_use_live_image)
    nepi_ros.set_param(self,'~use_last_image',self.init_use_last_image)

    nepi_ros.set_param(self,"~selected_class",self.init_sel_class)

    nepi_ros.set_param(self,"~pt_namespace",self.init_pt_namespace)
    nepi_ros.set_param(self,"~scan_time",self.init_scan_time)
    nepi_ros.set_param(self,"~min_area_ratio",self.init_min_area_ratio)
    nepi_ros.set_param(self,"~scan_speed_ratio",self.init_scan_speed_ratio)
    nepi_ros.set_param(self,"~scan_tilt_offset",self.init_scan_tilt_offset)
    nepi_ros.set_param(self,"~scan_pan_angle",self.init_scan_pan_angle)
    nepi_ros.set_param(self,"~track_speed_ratio",self.init_track_speed_ratio)
    nepi_ros.set_param(self,"~track_tilt_offset", self.init_track_tilt_offset)

    nepi_ros.set_param(self,"~error_goal",self.init_error_goal)

    if do_updates:
        self.updateFromParamServer()
        self.publish_status()


  ###################
  ## Status Publisher
  def publish_status(self):
    status_msg = AiPtTrackerStatus()

    status_msg.tracking_enabled = nepi_ros.get_param(self,"~tracking_enabled",self.init_tracking_enabled)
    
    status_msg.classifier_running = self.classifier_running
    status_msg.targeting_running = self.targeting_running

    status_msg.image_topic = self.current_image_topic
    status_msg.use_live_image = nepi_ros.get_param(self,'~use_live_image',self.init_use_live_image)
    status_msg.use_last_image = nepi_ros.get_param(self,'~use_last_image',self.init_use_last_image)

    status_msg.available_classes_list = sorted(self.classes_list)
    status_msg.selected_class = nepi_ros.get_param(self,"~selected_class",self.init_sel_class)
    status_msg.target_detected = self.target_detected

    status_msg.pantilt_device = nepi_ros.get_param(self,"~pt_namespace",self.init_pt_namespace)
    status_msg.pantilt_connected = self.pt_connected
    status_msg.has_position_feedback = self.has_position_feedback
    if self.pt_status_msg is not None:
            tilt_min_max = [self.pt_status_msg.pitch_min_softstop_deg, self.pt_status_msg.pitch_max_softstop_deg]
            pan_min_max = [self.pt_status_msg.yaw_min_softstop_deg, self.pt_status_msg.yaw_max_softstop_deg]
    else:
            tilt_min_max = [-999,-999]
            pan_min_max = [-999,-999]
    status_msg.max_scan_time_sec = self.MAX_SCAN_TIME
    status_msg.scan_time_sec = nepi_ros.get_param(self,"~scan_time",self.init_scan_time)
    status_msg.min_area_ratio = nepi_ros.get_param(self,"~min_area_ratio",self.init_min_area_ratio)
    status_msg.scan_speed_ratio = nepi_ros.get_param(self,"~scan_speed_ratio",self.init_scan_speed_ratio)
    status_msg.scan_tilt_offset = nepi_ros.get_param(self,"~scan_tilt_offset",self.init_scan_tilt_offset)
    status_msg.scan_pan_angle = nepi_ros.get_param(self,"~scan_pan_angle",self.init_scan_pan_angle)
    status_msg.track_speed_ratio = nepi_ros.get_param(self,"~track_speed_ratio",self.init_track_speed_ratio)
    status_msg.track_tilt_offset = nepi_ros.get_param(self,"~track_tilt_offset", self.init_track_tilt_offset)

    status_msg.error_goal_min_max_deg = self.MIN_MAX_ERROR_GOAL
    status_msg.error_goal_deg = nepi_ros.get_param(self,"~error_goal",self.init_error_goal)

    status_msg.is_scanning = self.is_scanning
    status_msg.is_tracking = self.is_tracking

    self.status_pub.publish(status_msg)



  def updaterCb(self,timer):
    update_status = False
    if targeting_status_msg is not None:
      self.targeting_running = True
      #status_str = str(targeting_status_msg)
      #nepi_msg.publishMsgWarn(self," got ai manager status: " + status_str)
      self.current_image_topic = targeting_status_msg.image_topic
      self.classifier_running = targeting_status_msg.classifier_running
      classes_list = targeting_status_msg.selected_classifier_classes
      if classes_list != self.classes_list:
        self.classes_list = classes_list
        if len(self.classes_list) > 0:
          cmap = plt.get_cmap('viridis')
          color_list = cmap(np.linspace(0, 1, len(self.classes_list))).tolist()
          rgb_list = []
          for color in color_list:
            rgb = []
            for i in range(3):
              rgb.append(int(color[i]*255))
            rgb_list.append(rgb)
          self.class_color_list = rgb_list
          #nepi_msg.publishMsgWarn(self,self.class_color_list)
        #classes_str = str(self.classes_list)
        #nepi_msg.publishMsgWarn(self," got ai manager status: " + classes_str)
        update_status = True
    
      #nepi_msg.publishMsgWarn(self," Got image topics last and current: " + self.last_image_topic + " " + self.current_image_topic)
      if self.classifier_running:
        use_live_image = nepi_ros.get_param(self,'~use_live_image',self.init_use_live_image)
        if (self.last_image_topic != self.current_image_topic) or (self.image_sub == None and self.current_image_topic != "None") or self.reset_image_topic == True:
          self.reset_image_topic = False
          image_topic = ""
          if use_live_image:
            image_topic = nepi_ros.find_topic(self.current_image_topic)
          if image_topic == "":
            source_topic = AI_MGR_STATUS_SERVICE_NAME = self.ai_mgr_namespace  + "/source_image"
            image_topic = nepi_ros.find_topic(source_topic)
          nepi_msg.publishMsgInfo(self," Got detect image update topic update : " + image_topic)
          update_status = True
          if image_topic != "":
            if self.image_sub != None:
              nepi_msg.publishMsgWarn(self," Unsubscribing to Image topic : " + image_topic)
              self.image_sub.unregister()
              time.sleep(1)
              self.image_sub = None
            nepi_msg.publishMsgInfo(self," Subscribing to Image topic : " + image_topic)
            self.image_sub = rospy.Subscriber(image_topic, Image, self.alertsImageCb, queue_size = 1)
            time.sleep(1)
            if self.image_pub is None:
              #nepi_msg.publishMsgWarn(self," Creating Image publisher ")
              self.image_pub = rospy.Publisher("~image",Image,queue_size=1)
              time.sleep(1)
            self.alerts_running = True
            update_status = True
      elif self.classifier_running == False or self.current_image_topic == "None" or self.current_image_topic == "":  # Turn off alerts subscribers and reset last image topic
        if self.image_sub != None:
          nepi_msg.publishMsgWarn(self," Unsubscribing to Image topic : " + self.current_image_topic)
          self.image_sub.unregister()
          self.image_sub = None
        update_status = True
        time.sleep(1)
      # Publish warning image if not running
      if self.classifier_running == False or self.image_sub == None:
        self.ros_message_img.header.stamp = nepi_ros.time_now()
        self.image_pub.publish(self.ros_message_img)
      # Save last image topic for next check
      self.last_image_topic = self.current_image_topic
  
    sel_pt = nepi_ros.get_param(self,'~pt_namespace',  self.init_pt_namespace)
    if sel_pt == "None":
      nepi_ros.set_param(self,"~tracking_enabled",False)
      self.pt_connected = False
      self.pitch_yaw_errors_deg = [0,0]
      self.pt_status_msg = None

    errors_msg = TrackingErrors()
    errors_msg.pitch_deg = self.pitch_yaw_errors_deg[0]
    errors_msg.yaw_deg = self.pitch_yaw_errors_deg[1]
    self.errors_pub.publish(errors_msg)
    if update_status == True:
      self.publish_status()
    
    

  #######################
  ### Node Callbacks


  def enableTrackerCb(self,msg):
    current_enable = nepi_ros.get_param(self,"~tracking_enabled",self.init_tracking_enabled)
    enable = msg.data
    if current_enable != enable:
      nepi_ros.set_param(self,"~tracking_enabled",enable)
    self.publish_status()


  def setImageLiveCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    live = msg.data
    current_live = nepi_ros.get_param(self,'~use_live_image',self.init_use_live_image)
    if live != current_live:
      nepi_ros.set_param(self,'~use_live_image',live)
      self.reset_image_topic = True # Will force resubscribe later
    self.publish_status()

  def setImageLastCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    use_last = msg.data
    nepi_ros.set_param(self,'~use_last_image',use_last)
    self.publish_status()


  def setClassCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    selected_class = msg.data
    if selected_class in self.classes_list:
      nepi_ros.set_param(self,'~selected_class',  selected_class)
    self.publish_status()

  def setPtTopicCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    pt_topic = msg.data
    nepi_ros.set_param(self,'~pt_namespace',  pt_topic)
    self.publish_status()


  def settimedScanCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    val = msg.data
    if val > 0 and val <= MAX_SCAN_TIME:
      nepi_ros.set_param(self,'~scan_time',  val)
    self.publish_status()

  def setMinAreaCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    val = msg.data
    if val >= 0 and val <= 1:
      nepi_ros.set_param(self,'~min_area_ratio',  val)
    self.publish_status()

  def setScanSpeedCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    val = msg.data
    if val >= 0 and val <= 1:
      nepi_ros.set_param(self,'~scan_speed_ratio',  val)
    self.publish_status()

  def setScanTiltOffsetCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    val = msg.data
    if self.pt_status_msg is not None:
      tilt_min = self.pt_status_msg.pitch_min_softstop_deg
      tilt_max = self.pt_status_msg.pitch_max_softstop_deg
      if val >= tilt_min and val <= tilt_max:
        nepi_ros.set_param(self,'~scan_tilt_offset',  val)
    self.publish_status()

  def setScanPanAngleCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    val = msg.data
    if self.pt_status_msg is not None:
      pan_to_min = -val/2
      pan_to_max = val/2
      pan_min = self.pt_status_msg.yaw_min_softstop_deg
      pan_max = self.pt_status_msg.yaw_max_softstop_deg
    if val >= 10:
      if pan_to_min < pan_min or pan_to_max > pan_max:
        pan_angle = 2 * max([abs(pan_min),abs(pan_max)])
      else:
        pan_angle = val
      nepi_ros.set_param(self,'~scan_pan_angle',  val)
    self.publish_status()


  def setTrackSpeedCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    val = msg.data
    if val >= 0 and val <= 1:
      nepi_ros.set_param(self,'~track_speed_ratio',  val)
    self.publish_status()

  def setTrackTiltOffsetCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    val = msg.data
    if val >= -30 and val <= 30:
      nepi_ros.set_param(self,'~track_tilt_offset',  val)
    self.publish_status()

  def setErrorGoalCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    val = msg.data
    if val < self.MIN_MAX_ERROR_GOAL[0]:
      val = self.MIN_MAX_ERROR_GOAL[0]
    if val > self.MIN_MAX_ERROR_GOAL[1]:
      val = self.MIN_MAX_ERROR_GOAL[1]
    nepi_ros.set_param(self,'~error_goal',  val)
    self.publish_status()
  #######################
  ### PT Callbacks

  ### Simple callback to get pt pt_status_msg info
  def pt_status_msg_callback(self,pt_status_msg):
    # This is just to get the current pt positions
    self.pt_status_msg = pt_status_msg
    self.has_position_feedback = pt_status_msg.has_position_feedback
    val = nepi_ros.get_param(self,'~scan_pan_angle',  val)
    if self.pt_status_msg is not None:
      pan_to_min = -val/2
      pan_to_max = val/2
      pan_min = self.pt_status_msg.yaw_min_softstop_deg
      pan_max = self.pt_status_msg.yaw_max_softstop_deg
    if val >= 10:
      if pan_to_min < pan_min or pan_to_max > pan_max:
        pan_angle = 2 * min([abs(pan_min),abs(pan_max)])
      else:
        pan_angle = val
      nepi_ros.set_param(self,'~scan_pan_angle',  val)

  def timedScanCb(self,timer):
    current_dir = self.timed_scan_dir
    self.timed_scan_dir = -1 * current_dir
    scan_time = nepi_ros.get_param(self,"~scan_time",self.init_scan_time)
    if self.pt_connected == True:
      if self.has_position_feedback == False & self.is_scanning == True:
        pan_jog_msg = SingleAxisTimedMove()
        pan_jog_msg.direction = self.timed_scan_dir
        pan_jog_msg.duration_s = scan_time
        self.set_pt_pan_jog_pub.publish(pan_jog_msg)


  def timedTrackCb(self,timer):
    tilt_error = self.pitch_yaw_errors_deg[0]
    pan_error = self.pitch_yaw_errors_deg[1]
    error_goal = nepi_ros.get_param(self,"~error_goal",self.init_error_goal)
    if self.pt_connected == True:
      if self.has_position_feedback == False & self.is_tracking == True:
        if abs(tilt_error) < error_goal and abs(pan_error) < error_goal
          self.pt_stop_motion_pub(Empty)
        else:
          if abs(tilt_error) > error_goal:
            tilt_jog_msg = SingleAxisTimedMove()
            tilt_jog_msg.direction =  np.sign(tilt_error)
            tilt_jog_msg.duration_s = self.TRACK_JOG_TIME
            self.set_pt_tilt_jog_pub.publish(tilt_jog_msg)
          if abs(pan_error) > error_goal:
            pan_jog_msg = SingleAxisTimedMove()
            pan_jog_msg.direction =  np.sign(pan_error)
            pan_jog_msg.duration_s = self.TRACK_JOG_TIME
            self.set_pt_pan_jog_pub.publish(pan_jog_msg)
    
  ### Setup a regular background scan process based on timer callback
  def ptScanCb(self,timer):
    tracking_enabled = nepi_ros.get_param(self,"~tracking_enabled",self.init_tracking_enabled)
    if tracking_enabled == True:
      scan_speed_ratio = nepi_ros.get_param(self,"~scan_speed_ratio",self.init_scan_speed_ratio)
      scan_tilt_offset = nepi_ros.get_param(self,"~scan_tilt_offset",self.init_scan_tilt_offset)
      scan_pan_angle = nepi_ros.get_param(self,"~scan_pan_angle",self.init_scan_pan_angle)
      pan_to_max = scan_pan_angle / 2
      pan_to_min = -1 * scan_pan_angle / 2
      if self.target_detected == False: # if not tracking, return to scan mode
        self.is_scanning = True
        self.is_tracking = False
        self.set_pt_speed_ratio_pub.publish(scan_speed_ratio)
        if self.has_position_feedback = False:
          pass # Managed by seperate timer callback
        else:
          current_dir = self.pos_scan_dir
          if self.pt_status_msg is not None:
            tilt_min = self.pt_status_msg.pitch_min_softstop_deg
            tilt_max = self.pt_status_msg.pitch_max_softstop_deg
            tilt_cur = self.pt_status_msg.pitch_now_deg
            tilt_goal = self.pt_status_msg.pitch_goal_deg
            pan_min = self.pt_status_msg.yaw_min_softstop_deg
            pan_max = self.pt_status_msg.yaw_max_softstop_deg
            pan_cur = self.pt_status_msg.pitch_now_deg
            pan_goal = self.pt_status_msg.pitch_goal_deg
            # Check tilt limits
            if scan_tilt_offset < tilt_min:
              scan_tilt_offset = tilt_min
            if scan_tilt_offset > tilt_max:
              scan_tilt_offset = tilt_max
            nepi_ros.set_param(self,"~scan_tilt_offset",scan_tilt_offset)
            # Check pan limits
            if scan_pan_angle < 10:
              scan_pan_angle = 10
            if pan_to_min < pan_min or pan_to_max > pan_max:
              scan_pan_angle = 2 * min([abs(pan_min),abs(pan_max)])
            nepi_ros.set_param(self,"~scan_pan_angle",scan_pan_angle)
            scan_pan_goal = scan_pan_angle / 2
            pan_to_max = scan_pan_angle / 2
            pan_to_min = -1 * scan_pan_angle / 2
            # Check if scan dir change needed
            
            if (pan_cur < pan_min + 5) and current_dir != -1:
              pan_tilt_pos_msg = PanTiltPosition()
              pan_tilt_pos_msg.yaw_deg = pan_to_min
              pan_tilt_pos_msg.pitch_deg = scan_tilt_offset
              self.set_pt_position_pub.publish(pan_tilt_pos_msg)
              self.pos_scan_dir = -1
              self.publish_status()
    
            if (pan_cur > pan_min - 5) and current_dir != 1:
              pan_tilt_pos_msg = PanTiltPosition()
              pan_tilt_pos_msg.yaw_deg = pan_to_max
              pan_tilt_pos_msg.pitch_deg = scan_tilt_offset
              self.set_pt_position_pub.publish(pan_tilt_pos_msg)
              self.pos_scan_dir = 1
              self.publish_status()


  # Action upon detection of object of interest
  def ptTrackCb(self,target_locs_msg):
    self.target_locs_msg = target_locs_msg
    tracking_enabled = nepi_ros.get_param(self,"~tracking_enabled",self.init_tracking_enabled)
    if tracking_enabled == False:
      self.target_detected=False
    elif self.pt_connected == True:
      error_goal = nepi_ros.get_param(self,"~error_goal",self.init_error_goal)
      selected_class = nepi_ros.get_param(self,"~selected_class",self.init_sel_class)
      min_area_ratio =  nepi_ros.get_param(self,"~min_area_ratio",self.init_min_area_ratio)
      track_speed_ratio = nepi_ros.get_param(self,"~track_speed_ratio",self.init_track_speed_ratio)
      track_tilt_offset = nepi_ros.get_param(self,"~track_tilt_offset", self.init_track_tilt_offset)
      #nepi_msg.publishMsgInfo("Entering Detection Callback")
      # Iterate over all of the objects reported by the detector and return center of largest box in degrees relative to img center
      largest_box_area_ratio=0 # Initialize largest box area
      for target_loc in target_locs_msg.target_localizations:
        # Check for the object of interest and take appropriate actions
        if target_loc.Class == selected_class:
          # Check if largest box
          box_area_ratio = target_loc.area_ratio
          if box_area_ratio > largest_box_area_ratio:
            largest_box_area_ratio=box_area_ratio
            largest_target=target_loc
      if largest_box_area_ratio > min_area_ratio:
        self.target_detected = True
        self.is_scanning = False
        self.is_tracking = True
        self.set_pt_speed_ratio_pub.publish(track_speed_ratio)
        # publish error and make change
        if self.pt_status_msg is not None:
          tilt_min = self.pt_status_msg.pitch_min_softstop_deg
          tilt_max = self.pt_status_msg.pitch_max_softstop_deg
          tilt_cur = self.pt_status_msg.pitch_now_deg
          tilt_goal = self.pt_status_msg.pitch_goal_deg
          pan_min = self.pt_status_msg.yaw_min_softstop_deg
          pan_max = self.pt_status_msg.yaw_max_softstop_deg
          pan_cur = self.pt_status_msg.pitch_now_deg
          pan_goal = self.pt_status_msg.pitch_goal_deg

          pan_error = largest_target.azimuth_deg
          tilt_error = largest_target.elevation_deg
          self.pitch_yaw_errors_deg = [pan_error,tilt_error]

          if self.has_position_feedback = False:
            pass # Managed by seperate timer callback
          else:
            if abs(tilt_error) < error_goal and abs(pan_error) < error_goal
              self.pt_stop_motion_pub(Empty)
            else:
              # Set pan angle goal
              if pan_error > error_goal:
                pan_to_goal = pan_cur + pan_error
              else:
                pan_to_goal = pan_cur
              if pan_to_goal < pan_min:
                pan_to_goal = pan_min
              if pan_to_goal > pan_max:
                pan_to_goal = pan_max
              # Set tilt angle goal
              if tilt_error > error_goal:
                tilt_to_goal = tilt_cur + tilt_error
              else:
                tilt_to_goal = tilt_cur
              if tilt_to_goal < tilt_min:
                tilt_to_goal = tilt_min
              if tilt_to_goal > tilt_max:
                tilt_to_goal = tilt_max
              # Send angle goal
              pt_pos_msg = PanTiltPosition()
              pt_pos_msg.yaw_deg = pan_to_goal
              pt_pos_msg.pitch_deg = tilt_to_goal
              self.set_pt_position_pub.publish(pt_pos_msg)   
      else:
        # Object of interest not detected, so reset target_detected
        self.target_detected=False  # will start scan mode on next timer event
        self.pitch_yaw_errors_deg = [0,0]

  def foundTargetCb(self,found_obj_msg):
    # Must reset target_detected in the event of no objects to restart scan mode
    if found_obj_msg.count == 0:
      #nepi_msg.publishMsgInfo("No objects found")
      self.target_detected=False

  def targetingStatusCb(self,targeting_status_msg)
    self.targeting_status_msg = targeting_status_msg


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    global led_intensity_pub
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")



#########################################
# Main
#########################################
if __name__ == '__main__':
  pantilt_object_tracker()


