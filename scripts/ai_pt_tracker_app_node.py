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

import time
import sys
import rospy
import statistics
import numpy as np
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_msg

from std_msgs.msg import UInt8, Empty, Float32
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import PanTiltLimits, PanTiltPosition, PanTiltStatus, StringArray
from darknet_ros_msgs.msg import TargetLocalization, TargetLocalizations, ObjectCount

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

  DEFAULT_TARGET = "person"
  DEFAULT_MIN_AREA_RATIO = 0.01 # Filters background targets.
  DEFAULT_SCAN_SPEED_RATIO = 0.5
  DEFAULT_SCAN_TILT_DEG = 0.15
  DEFAULT_SCAN_PAN_DEGS = 80
  DEFAULT_TRACK_SPEED_RATIO = 0.6
  DEFAULT_TRACK_TILT_OFFSET_DEG = 0.5

  STATUS_UPDATE_TIME = 1
  PTX_UPDATE_TIME = 1
  
  classifier_connected = False
  classes_list = []
  current_image_topic = ""
  current_classifier = ""
  current_classifier_state = "Stopped"
  classifier_running = False
  last_classifier = ""
  last_image_topic = ""


  target_detected = False


  pt_connected = False
  pt_status_msg = None
  pan_scan_direction = 1 # Keep track of current scan direction (1: Positive Limit, -1: Negative Limit)
  last_object_pan_ratio=0
  total_tilt_degs = 90
  current_tilt_ratio = 0.5
  total_pan_degs = 90
  current_pan_ratio = 0.5
  last_pt_device = ""

  is_running = False
  is_scanning = False
  is_tracking = False
  has_position_feedback = False


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
    
    # Setup Node Publishers
    #self.status_pub = rospy.Publisher("~status", AiPtTrackingStatus, queue_size=1, latch=True)

    # Class Subscribers
    ## App Setup ########################################################
    app_reset_app_sub = rospy.Subscriber('~reset_app', Empty, self.resetAppCb, queue_size = 10)
    self.initParamServerValues(do_updates=False)

    # App Specific Subscribers
    rospy.Subscriber('~set_pantilt_device', String, self.setPtTopicCb, queue_size = 10)
    rospy.Subscriber('~set_target_class', String, self.setTargetClassCb, queue_size = 10)

    rospy.Subscriber("~set_min_area_ratio", Float32, self.setMinAreaCb, queue_size = 10)
    rospy.Subscriber("~set_scan_speed_ratio", Float32, self.setScanSpeedCb, queue_size = 10)
    rospy.Subscriber("~set_scan_tilt_offset", Float32, self.setScanTiltOffsetCb, queue_size = 10)
    rospy.Subscriber("~set_scan_pan_angle", Float32, self.setScanPanAngleCb, queue_size = 10)
    rospy.Subscriber("~set_track_speed_ratio", Float32, self.setTrackSpeedCb, queue_size = 10)
    rospy.Subscriber("~set_track_tilt_offset", Float32, self.setTrackTiltOffsetCb, queue_size = 10)

    rospy.Subscriber('~start_pub', Empty, self.startTrackerCb)
    rospy.Subscriber('~stop_pub', Empty, self.stopTrackerCb)
  

    # Reset Params



    ## Class subscribers
    # AI Detector Subscriber Topics
    AI_FOUND_TARGET_TOPIC = self.base_namespace + "app_ai_targeting/target_count"
    nepi_msg.publishMsgInfo(self,"Waiting for Targeting Msg: " + AI_FOUND_TARGET_TOPIC )
    nepi_ros.wait_for_topic(AI_FOUND_TARGET_TOPIC)
    rospy.Subscriber(AAI_TARGET_LOCS_TOPIC, TargetLocalizations, self.targetLocsCb, queue_size = 1)

    AI_TARGET_LOCS_TOPIC = self.base_namespace + "app_ai_targeting/targeting_localizations"
    rospy.Subscriber(AI_FOUND_TARGET_TOPIC, ObjectCount, self.foundTargetCb, queue_size = 1)


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
    PTX_SET_SOFT_LIMITS_TOPIC = PTX_NAMESPACE + "set_soft_limits"

    ## Create Class Publishers
    self.send_pt_home_pub = rospy.Publisher(PTX_GOHOME_TOPIC, Empty, queue_size=10)
    self.set_pt_speed_ratio_pub = rospy.Publisher(PTX_SET_SPEED_RATIO_TOPIC, Float32, queue_size=10)
    self.set_pt_pan_ratio_pub = rospy.Publisher(PTX_GOTO_PAN_RATIO_TOPIC, Float32, queue_size=10)
    self.set_pt_tilt_ratio_pub = rospy.Publisher(PTX_GOTO_TILT_RATIO_TOPIC, Float32, queue_size=10)
    self.set_pt_soft_limits_pub = rospy.Publisher(PTX_SET_SOFT_LIMITS_TOPIC, PanTiltLimits, queue_size=10)
    self.pt_stop_motion_pub = rospy.Publisher(PTX_STOP_TOPIC, Empty, queue_size=10)

    # Set up ai targeting subscriber
    #################################


    nepi_ros.timer(nepi_ros.duration(self.STATUS_UPDATE_TIME), self.updaterCb)

    ## Start Node Processes
    # Set up the timer that start scanning when no objects are detected
    nepi_msg.publishMsgInfo("Setting up pan/tilt scan check timer")
    rospy.Timer(rospy.Duration(self.PTX_UPDATE_TIME), self.pt_scan_timer_callback)

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
    nepi_ros.set_param(self,"~pt_namespace","")
    nepi_ros.set_param(self,"~target_name",self.DEFAULT_TARGET)
    nepi_ros.set_param(self,"~min_ratio",self.DEFAULT_MIN_RATIO)
    nepi_ros.set_param(self,"~scan_speed_ratio",self.DEFAULT_SCAN_SPEED_RATIO)
    nepi_ros.set_param(self,"~scan_tilt_offset",self.DEFAULT_SCAN_TILT_ANGLE)
    nepi_ros.set_param(self,"~scan_pan_angle",self.DEFAULT_SCAN_PAN_DEGS)
    nepi_ros.set_param(self,"~track_speed_ratio",self.DEFAULT_TRACK_SPEED_RATIO)
    nepi_ros.set_param(self,"~track_tilt_offset",self.DEFAULT_TRACK_TILT_OFFSET)

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
    self.init_pt_namespace = nepi_ros.get_param(self,"~pt_namespace","")
    self.init_target_name = nepi_ros.get_param(self,"~target_name",self.DEFAULT_TARGET)
    self.init_min_area_ratio = nepi_ros.get_param(self,"~min_area_ratio",self.DEFAULT_MIN_AREA_RATIO)
    self.init_scan_speed_ratio = nepi_ros.get_param(self,"~scan_speed_ratio",self.DEFAULT_SCAN_SPEED_RATIO)
    self.init_scan_tilt_offset = nepi_ros.get_param(self,"~scan_tilt_offset",self.DEFAULT_SCAN_TILT_ANGLE)
    self.init_scan_pan_angle = nepi_ros.get_param(self,"~scan_pan_angle",self.DEFAULT_SCAN_PAN_DEGS)
    self.init_track_speed_ratio = nepi_ros.get_param(self,"~track_speed_ratio",self.DEFAULT_TRACK_SPEED_RATIO)
    self.init_track_tilt_offset = nepi_ros.get_param(self,"~track_tilt_offset",self.DEFAULT_TRACK_TILT_OFFSET)
  
    self.init_running =  nepi_ros.get_param(self,'~running',  False)
    self.resetParamServer(do_updates)

  def resetParamServer(self,do_updates = True):
    nepi_ros.set_param(self,"~pt_namespace",self.init_pt_namespace)
    nepi_ros.set_param(self,"~target_name",self.init_target_name)

    nepi_ros.set_param(self,"~min_area_ratio",self.init_min_area_ratio)
    nepi_ros.set_param(self,"~scan_speed_ratio",self.init_scan_speed_ratio)
    nepi_ros.set_param(self,"~scan_tilt_offset",self.init_scan_tilt_offset)
    nepi_ros.set_param(self,"~scan_pan_angle",self.init_scan_pan_angle)
    nepi_ros.set_param(self,"~track_speed_ratio",self.init_track_speed_ratio)
    nepi_ros.set_param(self,"~track_tilt_offset", self.init_track_tilt_offset)

    nepi_ros.set_param(self,'~running',  self.init_running)

      if do_updates:
          self.updateFromParamServer()
          self.publish_status()


  ###################
  ## Status Publisher
  def publish_status(self):
    status_msg = AiPtTrackerStatus()

    status_msg.classifier_running = self.classifier_running
    status_msg.classifier_connected = self.classifier_connected

    status_msg.target_class = nepi_ros.get_param(self,"~target_name",self.init_target_name)
    status_msg.target_detected = self.target_detected

    status_msg.is_running = self.is_running
    status_msg.is_scanning = self.is_scanning
    status_msg.is_tracking = self.is_tracking

    status_msg.pantilt_connected = self.pantilt_connected
    status_msg.has_position_feedback = self.has_position_feedback

    status_msg.pantilt_device = nepi_ros.get_param(self,"~pt_namespace",self.init_pt_namespace)


    status_msg.min_area_ratio = nepi_ros.get_param(self,"~min_area_ratio",self.init_min_area_ratio)
    status_msg.scan_speed_ratio = nepi_ros.get_param(self,"~scan_speed_ratio",self.init_scan_speed_ratio)
    status_msg.scan_tilt_offset = nepi_ros.get_param(self,"~scan_tilt_offset",self.init_scan_tilt_offset)
    status_msg.scan_pan_angle = nepi_ros.get_param(self,"~scan_pan_angle",self.init_scan_pan_angle)
    status_msg.track_speed_ratio = nepi_ros.get_param(self,"~track_speed_ratio",self.init_track_speed_ratio)
    status_msg.track_tilt_offset = nepi_ros.get_param(self,"~track_tilt_offset", self.init_track_tilt_offset)

    self.status_pub.publish(status_msg)



  def updaterCb(self,timer):
    update_status = False
    current_timestamp = nepi_ros.get_rostime()
    # Update Classifier Info
    try:
      ai_mgr_status_response = self.get_ai_mgr_status_service()
      #nepi_msg.publishMsgInfo(self," Got classifier status  " + str(ai_mgr_status_response))
    except Exception as e:
      nepi_msg.publishMsgWarn(self,"Failed to call AI MGR STATUS service" + str(e))
      return
    #status_str = str(ai_mgr_status_response)
    #nepi_msg.publishMsgWarn(self," got ai manager status: " + status_str)
    self.current_image_topic = ai_mgr_status_response.selected_img_topic
    self.current_classifier = ai_mgr_status_response.selected_classifier
    self.current_classifier_state = ai_mgr_status_response.classifier_state
    self.classifier_running = self.current_classifier_state == "Running"
    classes_list = ai_mgr_status_response.selected_classifier_classes
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
  
    last_classifier = self.last_classifier
    if last_classifier != self.current_classifier and self.current_classifier != "None":
      update_status = True
    self.last_classifier = self.current_classifier
    self.last_image_topic = self.current_image_topic
    #nepi_msg.publishMsgWarn(self," Got image topics last and current: " + self.last_image_topic + " " + self.current_image_topic)
    if self.classifier_running:
      self.classifier_connected = True
    else:
      self.classifier_connected = False
      ### Setup PT interface if needed


    current_pt_device = nepi_ros.set_param(self,"~pt_namespace",self.init_pt_namespace)
    if self.last_pt_device != current_pt_device:
      if self.pantilt_connected == True:
        self.send_pt_home_pub.unregister()
        self.set_pt_speed_ratio_pub.unregister()
        self.set_pt_pan_ratio_pub.unregister()
        self.set_pt_tilt_ratio_pub.unregister()
        self.set_pt_soft_limits_pub.unregister()
        self.pt_stop_motion_pub.unregister()
        if self.pt_sub != None:
          self.pt_sub.unregister()
          time.sleep(1)
          self.pt_sub = None
        time.sleep(1)
      pt_status_topic = os.path.join(current_pt_device,"/ptx/pt_status_msg")
      nepi_msg.publishMsgInfo(self,"Looking for PT device status msg on topic: " + pt_status_topic)
      pt_topic=nepi_ros.find_topic(pt_pt_status_msg_topic)
      if pt_topic == "":
        nepi_msg.publishMsgWarn(self,"No PT device status found on topic: " + pt_status_topic)
      else:
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
        PTX_SET_SOFT_LIMITS_TOPIC = PTX_NAMESPACE + "set_soft_limits"

        ## Create Class Publishers
        self.send_pt_home_pub = rospy.Publisher(PTX_GOHOME_TOPIC, Empty, queue_size=10)
        self.set_pt_speed_ratio_pub = rospy.Publisher(PTX_SET_SPEED_RATIO_TOPIC, Float32, queue_size=10)
        self.set_pt_pan_ratio_pub = rospy.Publisher(PTX_GOTO_PAN_RATIO_TOPIC, Float32, queue_size=10)
        self.set_pt_tilt_ratio_pub = rospy.Publisher(PTX_GOTO_TILT_RATIO_TOPIC, Float32, queue_size=10)
        self.set_pt_soft_limits_pub = rospy.Publisher(PTX_SET_SOFT_LIMITS_TOPIC, PanTiltLimits, queue_size=10)
        self.pt_stop_motion_pub = rospy.Publisher(PTX_STOP_TOPIC, Empty, queue_size=10)

        # Start PT Status Callback
        nepi_msg.publishMsgInfo("Starting Pan Tilt Stutus callback")
        self.pt_sub = rospy.Subscriber(PTX_GET_STATUS_TOPIC, PanTiltStatus, self.pt_status_msg_callback)  

        self.pantilt_connected = True
        update_status = True
    self.last_pt_device = current_pt_device


    if update_status == True:
      self.publish_status()

  #######################
  ### Node Callbacks


  def startTrackerCb(self,msg):
    sefl.startTracker()
    self.publish_status()

  def startTracker(self):
    if self.classifier_connected and self.pantilt_connected:
      self.is_running = True
      nepi_ros.set_param(self,'~running',  True)
   
  def stopTrackerCb(self,msg):
    sefl.stopTracker()
    self.publish_status()

  def startTracker(self):
    self.is_running = False
    nepi_ros.set_param(self,'~running',  False)


  def setPtTopicCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    pt_topic = msg.data
    nepi_ros.set_param(self,'~pt_namespace',  pt_topic)
    self.publish_status()

  def setTargetClassCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    target_class = msg.data
    if target_class in self.current_classifier_classes:
      nepi_ros.set_param(self,'~target_class',  fov)
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
    if val >= -30 and val <= 30:
      nepi_ros.set_param(self,'~scan_tilt_offset',  val)
    self.publish_status()

  def setScanPanAngleCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    val = msg.data
    if val >= 10 and val <= 180:
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


  #######################
  ### PT Callbacks

  ### Simple callback to get pt pt_status_msg info
  def pt_status_msg_callback(self,PanTiltStatus):
    # This is just to get the current pt positions
    self.pt_status_msg = PanTiltStatus
    self.total_tilt_degs = self.pt_status_msg.pitch_max_softstop_deg - self.pt_status_msg.pitch_min_softstop_deg
    tilt_ratio = 0.5 + self.pt_status_msg.pitch_now_deg / (self.total_tilt_degs)
    if self.pt_status_msg.reverse_pitch_control:
      self.current_tilt_ratio = 1 - tilt_ratio
    self.total_pan_degs = self.pt_status_msg.yaw_max_softstop_deg - self.pt_status_msg.yaw_min_softstop_deg
    pan_ratio = 0.5 + self.pt_status_msg.yaw_now_deg / (self.total_pan_degs)
    if self.pt_status_msg.reverse_yaw_control:
      self.current_pan_ratio = 1 - pan_ratio
    
  ### Setup a regular background scan process based on timer callback
  def pt_scan_timer_callback(self,timer):
    if self.is_running == True:
      target_name = nepi_ros.get_param(self,"~target_name",self.init_target_name)
      min_area_ratio =  nepi_ros.get_param(self,"~min_area_ratio",self.init_min_area_ratio)
      scan_speed_ratio = nepi_ros.get_param(self,"~scan_speed_ratio",self.init_scan_speed_ratio)
      scan_tilt_offset = nepi_ros.get_param(self,"~scan_tilt_offset",self.init_scan_tilt_offset)
      scan_pan_angle = nepi_ros.get_param(self,"~scan_pan_angle",self.init_scan_pan_angle)
      track_speed_ratio = nepi_ros.get_param(self,"~track_speed_ratio",self.init_track_speed_ratio)
      track_tilt_offset = nepi_ros.get_param(self,"~track_tilt_offset", self.init_track_tilt_offset)

      pan_scan_limit = scan_pan_angle / float(2)

      # Called periodically no matter what as a Timer object callback
      if not self.target_detected: # if not tracking, return to scan mode
        self.is_scanning = True
        self.is_tracking = False
        #nepi_msg.publishMsgInfo("No Targets Found, Entering Scan Mode")
        if self.pt_status_msg.yaw_now_deg > pan_scan_limit:
          nepi_msg.publishMsgInfo("Soft Pan Limit Reached, Reversing Scan Direction")
          if self.pt_status_msg.reverse_yaw_control == False:
            self.pan_scan_direction = -1
          else:
            self.pan_scan_direction = 1
        elif self.pt_status_msg.yaw_now_deg < (-1 * pan_scan_limit):
          nepi_msg.publishMsgInfo("Soft Pan Limit Reached, Reversing Scan Direction")
          if self.pt_status_msg.reverse_yaw_control == False:
            self.pan_scan_direction = 1
          else:
            self.pan_scan_direction = -1
        pan_ratio = (self.pan_scan_direction + 1) /2

        tilt_offset_ratio = scan_tilt_offset
        if self.pt_status_msg.reverse_pitch_control == False:
          tilt_offset_ratio = - tilt_offset_ratio
        tilt_ratio = 0.5 + tilt_offset_ratio
        speed_ratio = scan_speed_ratio
        nepi_msg.publishMsgInfo("Current pan_scan_to_dir: " + "%.2f" % (pan_ratio))
        nepi_msg.publishMsgInfo("Current tilt_scan_to_dir: " + "%.2f" % (tilt_ratio))
        self.set_pt_speed_ratio_pub.publish(speed_ratio)
        self.set_pt_pan_ratio_pub.publish(pan_ratio)
        self.set_pt_tilt_ratio_pub.publish(tilt_ratio)



  # Action upon detection of object of interest
  def targetLocsCb(self,target_locs_msg):
    if self.is_running == True:
      target_name = nepi_ros.get_param(self,"~target_name",self.init_target_name)
      min_area_ratio =  nepi_ros.get_param(self,"~min_area_ratio",self.init_min_area_ratio)
      scan_speed_ratio = nepi_ros.get_param(self,"~scan_speed_ratio",self.init_scan_speed_ratio)
      scan_tilt_offset = nepi_ros.get_param(self,"~scan_tilt_offset",self.init_scan_tilt_offset)
      scan_pan_angle = nepi_ros.get_param(self,"~scan_pan_angle",self.init_scan_pan_angle)
      track_speed_ratio = nepi_ros.get_param(self,"~track_speed_ratio",self.init_track_speed_ratio)
      track_tilt_offset = nepi_ros.get_param(self,"~track_tilt_offset", self.init_track_tilt_offset)


      #nepi_msg.publishMsgInfo("Entering Detection Callback")
      # Iterate over all of the objects reported by the detector and return center of largest box in degrees relative to img center
      largest_box_area_ratio=0 # Initialize largest box area
      for target_loc in target_locs_msg.target_localizations:
        # Check for the object of interest and take appropriate actions
        if target_loc.Class == target_name:
          # Check if largest box
          box_area_ratio = target_loc.area_ratio
          if box_area_ratio > largest_box_area_ratio:
            largest_box_area_ratio=box_area_ratio
            largest_box=box
      if largest_box_area_ratio > min_area_ratio:
        toi = largest_box
        self.target_detected = True
        self.is_scanning = False
        self.is_tracking = True
        # Calculate the box center in image ratio terms
        object_loc_y_pix = toi.ymin + ((toi.ymax - toi.ymin)  / 2) 
        object_loc_x_pix = toi.xmin + ((toi.xmax - toi.xmin)  / 2)
        object_loc_y_ratio = float(object_loc_y_pix) / self.img_height - track_tilt_offset
        object_loc_x_ratio = float(object_loc_x_pix) / self.img_width
        object_error_y_ratio = (object_loc_y_ratio - 0.5)  
        object_error_x_ratio = (object_loc_x_ratio - 0.5) 
        #nepi_msg.publishMsgInfo("Object Detection Center Error Ratios  x: " + "%.2f" % (object_error_x_ratio) + " y: " + "%.2f" % (object_error_y_ratio))
        # Call the tracking algorithm
        self.pt_track_box(object_error_x_ratio, object_error_y_ratio)
      else:
        # Object of interest not detected, so reset target_detected
        self.target_detected=False  # will start scan mode on next timer event
    
    def foundTargetCb(self,found_obj_msg):
      # Must reset target_detected in the event of no objects to restart scan mode
      if found_obj_msg.count == 0:
        #nepi_msg.publishMsgInfo("No objects found")
        self.target_detected=False

  ### Track box process based on current box center relative ratio of image
  def pt_track_box(self,object_error_x_ratio, object_error_y_ratio):
    target_name = nepi_ros.get_param(self,"~target_name",self.init_target_name)
    min_area_ratio =  nepi_ros.get_param(self,"~min_area_ratio",self.init_min_area_ratio)
    scan_speed_ratio = nepi_ros.get_param(self,"~scan_speed_ratio",self.init_scan_speed_ratio)
    scan_tilt_offset = nepi_ros.get_param(self,"~scan_tilt_offset",self.init_scan_tilt_offset)
    scan_pan_angle = nepi_ros.get_param(self,"~scan_pan_angle",self.init_scan_pan_angle)
    track_speed_ratio = nepi_ros.get_param(self,"~track_speed_ratio",self.init_track_speed_ratio)
    track_tilt_offset = nepi_ros.get_param(self,"~track_tilt_offset", self.init_track_tilt_offset)
    #nepi_msg.publishMsgInfo("Entering Track Callback, Object Detection Value: " + str(self.target_detected)) 
    if self.target_detected:
      #nepi_msg.publishMsgInfo("Target Found, Entering Track Mode")
      # Simple bang/bang positional control with hysteresis band and error-proportional speed control
      # First check if we are close enough to center in either dimension to stop motion: Hysteresis band
      # Adjust the vertical box error to better center on target
      nepi_msg.publishMsgInfo("Object Detection Error Ratios pan: " + "%.2f" % (object_error_x_ratio) + " tilt: " + "%.2f" % (object_error_y_ratio))
      if (abs(object_error_y_ratio) <= self.PTX_OBJ_CENTERED_BUFFER_RATIO ) and \
         (abs(object_error_x_ratio) <= self.PTX_OBJ_CENTERED_BUFFER_RATIO ):
        #nepi_msg.publishMsgInfo("Object is centered in frame: Stopping any p/t motion") 
        self.pt_stop_motion_pub.publish()
      else:
        #nepi_msg.publishMsgInfo("Object not centered in frame")
        # Now set the speed proportional to average error
        speed_control_value = track_speed_ratio + \
                              (self.PTX_MAX_TRACK_SPEED_RATIO-self.PTX_MIN_TRACK_SPEED_RATIO) * max(abs(object_error_x_ratio),abs(object_error_y_ratio))
        #nepi_msg.publishMsgInfo("Current track speed ratio: " + "%.2f" % (speed_control_value))
        self.set_pt_speed_ratio_pub.publish(speed_control_value)
        # Per-axis adjustment
        self.move_pan_rel_ratio(object_error_x_ratio)
        self.move_tilt_rel_ratio(object_error_y_ratio)
        # set next scan in direction of last detection
        if self.pt_status_msg.reverse_yaw_control == False:
          self.pan_scan_direction = - np.sign(object_error_x_ratio)
        else:
          self.pan_scan_direction = np.sign(object_error_x_ratio)
        #nepi_msg.publishMsgInfo("X Error: " + "%.2f" % (object_error_x_ratio))
        #nepi_msg.publishMsgInfo("New Scan Dir: " + str(self.pan_scan_direction))



  def move_pan_rel_ratio(self,pan_rel_ratio):
    nepi_msg.publishMsgInfo("Pan Track Info")
    nepi_msg.publishMsgInfo(str(self.current_pan_ratio))
    nepi_msg.publishMsgInfo(str(pan_rel_ratio))
    if self.pt_status_msg.reverse_yaw_control == False:
      pan_ratio = self.current_pan_ratio - pan_rel_ratio
    else:
      pan_ratio = self.current_pan_ratio + pan_rel_ratio
    nepi_msg.publishMsgInfo(str(pan_ratio))
    if pan_ratio < 0.0:
      pan_ratio = 0
    elif pan_ratio > 1.0:
      pan_ratio = 1
    if not rospy.is_shutdown():
      nepi_msg.publishMsgInfo("Current pan_to_ratio: " + "%.2f" % (pan_ratio))
      self.set_pt_pan_ratio_pub.publish(pan_ratio)

  def move_tilt_rel_ratio(self,tilt_rel_ratio):
    #nepi_msg.publishMsgInfo("Tilt Track Info")
    #nepi_msg.publishMsgInfo(self.current_tilt_ratio)
    #nepi_msg.publishMsgInfo(tilt_rel_ratio)
    if self.pt_status_msg.reverse_pitch_control == True:
      tilt_ratio = self.current_tilt_ratio - tilt_rel_ratio
    else:
      tilt_ratio = self.current_tilt_ratio + tilt_rel_ratio
    #nepi_msg.publishMsgInfo(str(tilt_ratio))
    if tilt_ratio < 0:
      tilt_ratio = 0
    elif tilt_ratio > 1:
      tilt_ratio = 1
    if not rospy.is_shutdown():
      nepi_msg.publishMsgInfo("Current tilt_to_ratio: " + "%.2f" % (tilt_ratio))
      self.set_pt_tilt_ratio_pub.publish(tilt_ratio)


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


