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
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1] # remove to run as automation script
import rospy
import time
import sys
import copy
import threading
import statistics
import numpy as np
import cv2
from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_save
from nepi_edge_sdk_base import nepi_img

from std_msgs.msg import Bool, UInt8, Empty, Float32, String
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import PanTiltLimits, PanTiltPosition, SingleAxisTimedMove, PanTiltStatus, StringArray

from nepi_ros_interfaces.msg import BoundingBox, BoundingBoxes, ObjectCount, RangeWindow

from nepi_ros_interfaces.srv import ImageClassifierStatusQuery, ImageClassifierStatusQueryRequest, PTXCapabilitiesQuery


from nepi_app_ai_targeting.msg import AiTargetingStatus
from nepi_app_ai_pt_tracker.msg import AiPtTrackerStatus , TrackingErrors

from nepi_edge_sdk_base.save_data_if import SaveDataIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF


#########################################
# Node Class
#########################################

class pantiltTargetTrackerApp(object):
  AI_MANAGER_NODE_NAME = "ai_detector_mgr"

  UDATE_PROCESS_DELAY = 1
  SCAN_TRACK_PROCESS_DELAY = 0.2
  IMG_PUB_PROCESS_DELAY = 0.2

  PTX_MAX_TRACK_SPEED_RATIO = 1.0
  PTX_MIN_TRACK_SPEED_RATIO = 0.1
  PTX_OBJ_CENTERED_BUFFER_RATIO = 0.15 # Hysteresis band about center of image for tracking purposes


# Pan and Tilt tracking settings

  FACTORY_FOV_VERT_DEG=70 # Camera Vertical Field of View (FOV)
  FACTORY_FOV_HORZ_DEG=110 # Camera Horizontal Field of View (FOV)


  FACTORY_CLASS = "None"
  FACTORY_SCAN_TIME = 5.0
  FACTORY_MIN_AREA_RATIO = 0.05 # Filters background targets.
  FACTORY_SCAN_SPEED_RATIO = 0.6
  FACTORY_SCAN_TILT_DEG = -15

  FACTORY_MIN_MAX_PAN_ANGLES = [-60,60]
  FACTORY_MIN_MAX_TILT_ANGLES = [-30,30]


  FACTORY_TRACK_SPEED_RATIO = 0.6
  FACTORY_TRACK_TILT_OFFSET_DEG = -5

  MIN_MAX_ERROR_GOAL = [1,20]
  FACTORY_ERROR_GOAL_DEG = 10

  MAX_SCAN_TIME = 10
  TRACK_JOG_TIME = 0.5

  PTX_UPDATE_TIME = 1

  LOST_TARGET_COUNT_LIMIT = 5

  data_products = ["tracking_image"]

  classifier_running = False

  targeting_status_msg = None

  current_image_topic = ""
  last_image_topic = ""

  image_sub = None
  img_width = 0
  img_height = 0 

  classes_list = []
  target_detected = False

  pt_namespace = "None"
  pt_connected = False
  has_position_feedback = False
  has_adjustable_speed = False
  pt_status_msg = None
  last_sel_pt = "None"
  pt_status_msg = None
  current_scan_dir = 1
  
  pt_status_topic = ""
  pt_status_sub = None
  send_pt_home_pub = None
  set_pt_speed_ratio_pub  = None
  set_pt_position_pub  = None
  set_pt_pan_ratio_pub  = None
  set_pt_tilt_ratio_pub  = None
  set_pt_pan_jog_pub  = None
  set_pt_tilt_jog_pub  = None
  set_pt_soft_limits_pub  = None
  pt_stop_motion_pub  = None


  class_selected = False
  selected_class = "None"
  last_track_dir = -1
  last_pan_pos = 0
  is_moving = False

  no_object_count = 0
  lost_target_count = 0

  reset_image_topic = False
  app_enabled = False
  app_msg = "App not enabled"

  img_acquire = False
  img_msg = None
  img_lock = threading.Lock()

  target_box_acquire = False
  target_box = None
  target_box_lock = threading.Lock()

  is_scanning = False
  is_tracking = False

  pitch_yaw_errors_deg = [0.0,0.0]

  img_has_subs = False

  last_app_enabled = False
  #######################
  ### Node Initialization
  FACTORY_NODE_NAME = "app_ai_pt_tracker" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.FACTORY_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    self.ai_mgr_namespace = os.path.join(self.base_namespace, self.AI_MANAGER_NODE_NAME)

    self.initParamServerValues(do_updates = False)
    self.resetParamServer(do_updates = False)


    # Setup Node Publishers
    self.status_pub = rospy.Publisher("~status", AiPtTrackerStatus, queue_size=1, latch=True)
    self.errors_pub = rospy.Publisher("~errors", TrackingErrors, queue_size=1, latch=True)
    self.image_pub = rospy.Publisher("~tracking_image",Image,queue_size=1, latch = True)
    time.sleep(1)


    # Message Image to publish when detector not running
    message = "APP NOT ENABLED"
    cv2_img = nepi_img.create_message_image(message)
    self.app_ne_img = nepi_img.cv2img_to_rosimg(cv2_img)
    self.app_ne_img.header.stamp = nepi_ros.time_now()
    self.image_pub.publish(self.app_ne_img)

    message = "WAITING FOR AI DETECTOR TO START"
    cv2_img = nepi_img.create_message_image(message)
    self.classifier_nr_img = nepi_img.cv2img_to_rosimg(cv2_img)


    # Set up save data and save config services ########################################################
    factory_data_rates= {}
    for d in self.data_products:
        factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
    if 'tracking_image' in self.data_products:
        factory_data_rates['tracking_image'] = [1.0, 0.0, 100.0] 
    self.save_data_if = SaveDataIF(data_product_names = self.data_products, factory_data_rate_dict = factory_data_rates)
    # Temp Fix until added as NEPI ROS Node
    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)

    # Class Subscribers
    ## App Setup ########################################################
    app_reset_app_sub = rospy.Subscriber('~reset_app', Empty, self.resetAppCb, queue_size = 10)
    self.initParamServerValues(do_updates=False)

    # App Specific Subscribers
    rospy.Subscriber('~publish_status', Empty, self.pubStatusCb, queue_size = 10)
    rospy.Subscriber('~enable_app', Bool, self.appEnableCb, queue_size = 10)

    rospy.Subscriber("~set_image_fov_vert", Float32, self.setVertFovCb, queue_size = 10)
    rospy.Subscriber("~set_image_fov_horz", Float32, self.setHorzFovCb, queue_size = 10)

    rospy.Subscriber('~select_class', String, self.setClassCb, queue_size = 10)

    rospy.Subscriber('~select_pantilt', String, self.setPtTopicCb, queue_size = 10)
    rospy.Subscriber("~set_scan_time", Float32, self.setTimedScanCb, queue_size = 10)
    rospy.Subscriber("~set_min_area_ratio", Float32, self.setMinAreaCb, queue_size = 10)
    rospy.Subscriber("~set_scan_speed_ratio", Float32, self.setScanSpeedCb, queue_size = 10)
    rospy.Subscriber("~set_scan_tilt_offset", Float32, self.setScanTiltOffsetCb, queue_size = 10)
    rospy.Subscriber('~set_min_max_pan_angles', RangeWindow, self.setMinMaxPanCb, queue_size = 10)
    rospy.Subscriber('~set_min_max_tilt_angles', RangeWindow, self.setMinMaxTiltCb, queue_size = 10)
    rospy.Subscriber("~set_track_speed_ratio", Float32, self.setTrackSpeedCb, queue_size = 10)
    rospy.Subscriber("~set_track_tilt_offset", Float32, self.setTrackTiltOffsetCb, queue_size = 10)

    rospy.Subscriber("~set_error_goal_deg", Float32, self.setErrorGoalCb, queue_size = 10)


    # Get AI Manager Service Call
    AI_MGR_STATUS_SERVICE_NAME = self.ai_mgr_namespace  + "/img_classifier_status_query"
    self.get_ai_mgr_status_service = rospy.ServiceProxy(AI_MGR_STATUS_SERVICE_NAME, ImageClassifierStatusQuery)
    # Start AI Manager Subscribers
    FOUND_OBJECT_TOPIC = self.ai_mgr_namespace  + "/found_object"
    rospy.Subscriber(FOUND_OBJECT_TOPIC, ObjectCount, self.foundObjectCb, queue_size = 1)
    BOUNDING_BOXES_TOPIC = self.ai_mgr_namespace  + "/bounding_boxes"
    rospy.Subscriber(BOUNDING_BOXES_TOPIC, BoundingBoxes, self.objectDetectedCb, queue_size = 1)
    time.sleep(1)    


    # Set up ai targeting subscriber
    #################################
    nepi_ros.timer(nepi_ros.duration(self.UDATE_PROCESS_DELAY), self.updaterCb)
    nepi_ros.timer(nepi_ros.duration(self.SCAN_TRACK_PROCESS_DELAY), self.scanTrackCb)
    nepi_ros.timer(nepi_ros.duration(self.IMG_PUB_PROCESS_DELAY), self.imagePubCb)



    ## Start Node Processes
    # Set up the timer that start scanning when no objects are detected
    nepi_msg.publishMsgInfo(self,"Setting up scan check timer")
    #rospy.Timer(rospy.Duration(self.PTX_UPDATE_TIME), self.ptScanCb)

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


    nepi_ros.set_param(self,'~image_fov_vert',  self.FACTORY_FOV_VERT_DEG)
    nepi_ros.set_param(self,'~image_fov_horz', self.FACTORY_FOV_HORZ_DEG)

    nepi_ros.set_param(self,'~last_classifier', "")
    nepi_ros.set_param(self,"~selected_class",self.FACTORY_CLASS)

    nepi_ros.set_param(self,"~pt_namespace","None")
    nepi_ros.set_param(self,"~scan_time",self.FACTORY_SCAN_TIME)
    nepi_ros.set_param(self,"~min_ratio",self.FACTORY_MIN_AREA_RATIO)
    nepi_ros.set_param(self,"~scan_speed_ratio",self.FACTORY_SCAN_SPEED_RATIO)
    nepi_ros.set_param(self,"~scan_tilt_offset",self.FACTORY_SCAN_TILT_DEG)

    nepi_ros.set_param(self,"~min_pan_angle",self.FACTORY_MIN_MAX_PAN_ANGLES[0])
    nepi_ros.set_param(self,"~max_pan_angle",self.FACTORY_MIN_MAX_PAN_ANGLES[1])

    nepi_ros.set_param(self,"~min_tilt_angle",self.FACTORY_MIN_MAX_TILT_ANGLES[0])
    nepi_ros.set_param(self,"~max_tilt_angle",self.FACTORY_MIN_MAX_TILT_ANGLES[1])

    nepi_ros.set_param(self,"~track_speed_ratio",self.FACTORY_TRACK_SPEED_RATIO)
    nepi_ros.set_param(self,"~track_tilt_offset",self.FACTORY_TRACK_TILT_OFFSET_DEG)
    nepi_ros.set_param(self,"~error_goal",self.FACTORY_ERROR_GOAL_DEG)
    
    self.last_image_topic = ""
    self.last_sel_pt = ""
    nepi_ros.set_param(self,"~app_enabled",False)
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

    self.init_image_fov_vert = nepi_ros.get_param(self,'~image_fov_vert',  self.FACTORY_FOV_VERT_DEG)
    self.init_image_fov_horz = nepi_ros.get_param(self,'~image_fov_horz', self.FACTORY_FOV_HORZ_DEG)

    self.init_last_classifier = nepi_ros.get_param(self,"~last_classifier", "")
    self.init_sel_class = nepi_ros.get_param(self,"~selected_class",self.FACTORY_CLASS)

    self.init_pt_namespace = nepi_ros.get_param(self,"~pt_namespace","")
    self.init_scan_time = nepi_ros.get_param(self,"~scan_time",self.FACTORY_SCAN_TIME)
    self.init_min_area_ratio = nepi_ros.get_param(self,"~min_area_ratio",self.FACTORY_MIN_AREA_RATIO)
    self.init_scan_speed_ratio = nepi_ros.get_param(self,"~scan_speed_ratio",self.FACTORY_SCAN_SPEED_RATIO)
    self.init_scan_tilt_offset = nepi_ros.get_param(self,"~scan_tilt_offset",self.FACTORY_SCAN_TILT_DEG)
  
    self.init_min_pan = nepi_ros.get_param(self,"~min_pan_angle",self.FACTORY_MIN_MAX_PAN_ANGLES[0])
    self.init_max_pan = nepi_ros.get_param(self,"~max_pan_angle",self.FACTORY_MIN_MAX_PAN_ANGLES[1])

    self.init_min_tilt = nepi_ros.get_param(self,"~min_tilt_angle",self.FACTORY_MIN_MAX_TILT_ANGLES[0])
    self.init_max_tilt = nepi_ros.get_param(self,"~max_tilt_angle",self.FACTORY_MIN_MAX_TILT_ANGLES[1])


    self.init_track_speed_ratio = nepi_ros.get_param(self,"~track_speed_ratio",self.FACTORY_TRACK_SPEED_RATIO)
    self.init_track_tilt_offset = nepi_ros.get_param(self,"~track_tilt_offset",self.FACTORY_TRACK_TILT_OFFSET_DEG)

    self.init_error_goal = nepi_ros.get_param(self,"~error_goal",self.FACTORY_ERROR_GOAL_DEG)

    self.init_app_enabled = nepi_ros.get_param(self,"~app_enabled",False)

    self.resetParamServer(do_updates)



  def resetParamServer(self,do_updates = True):


    nepi_ros.set_param(self,'~image_fov_vert',  self.init_image_fov_vert)
    nepi_ros.set_param(self,'~image_fov_horz', self.init_image_fov_horz)

    nepi_ros.set_param(self,'~last_classiier', self.init_last_classifier)
    nepi_ros.set_param(self,"~selected_class",self.init_sel_class)

    nepi_ros.set_param(self,"~pt_namespace",self.init_pt_namespace)
    nepi_ros.set_param(self,"~scan_time",self.init_scan_time)
    nepi_ros.set_param(self,"~min_area_ratio",self.init_min_area_ratio)
    nepi_ros.set_param(self,"~scan_speed_ratio",self.init_scan_speed_ratio)
    nepi_ros.set_param(self,"~scan_tilt_offset",self.init_scan_tilt_offset)

    nepi_ros.set_param(self,"~min_pan_angle",self.init_min_pan)
    nepi_ros.set_param(self,"~max_pan_angle",self.init_max_pan)

    nepi_ros.set_param(self,"~min_tilt_angle",self.init_min_tilt)
    nepi_ros.set_param(self,"~max_tilt_angle",self.init_max_tilt)

    nepi_ros.set_param(self,"~track_speed_ratio",self.init_track_speed_ratio)
    nepi_ros.set_param(self,"~track_tilt_offset", self.init_track_tilt_offset)

    nepi_ros.set_param(self,"~error_goal",self.init_error_goal)
    nepi_ros.set_param(self,'~app_enabled',self.init_app_enabled)
    if do_updates:
        self.updateFromParamServer()
        self.publish_status()


  ###################
  ## Status Publisher
  def publish_status(self):
    status_msg = AiPtTrackerStatus()

    status_msg.app_enabled = nepi_ros.get_param(self,'~app_enabled',self.init_app_enabled)
    status_msg.app_msg = self.app_msg
    
    status_msg.image_topic = self.current_image_topic
    status_msg.image_fov_vert_degs = nepi_ros.get_param(self,'~image_fov_vert',  self.init_image_fov_vert)
    status_msg.image_fov_horz_degs = nepi_ros.get_param(self,'~image_fov_horz', self.init_image_fov_horz)


    status_msg.classifier_running = self.classifier_running

    status_msg.available_classes_list = sorted(self.classes_list)
    status_msg.selected_class = self.selected_class 
    status_msg.target_detected = self.target_detected

    status_msg.pantilt_device = self.pt_namespace
    status_msg.pantilt_connected = self.pt_connected
    status_msg.has_position_feedback = self.has_position_feedback
    status_msg.has_adjustable_speed = self.has_adjustable_speed

    if self.pt_status_msg is not None:
      pan_min = self.pt_status_msg.yaw_min_softstop_deg
      pan_max = self.pt_status_msg.yaw_max_softstop_deg
      status_msg.pan_min_max_deg = [pan_min,pan_max]

      tilt_min = self.pt_status_msg.pitch_min_softstop_deg
      tilt_max = self.pt_status_msg.pitch_max_softstop_deg
      status_msg.tilt_min_max_deg = [tilt_min,tilt_max]
    else:
      status_msg.pan_min_max_deg = [-180,180]
      status_msg.tilt_min_max_deg = [-180,180]


    min_pan = nepi_ros.get_param(self,"~min_pan_angle",self.init_min_pan)
    max_pan = nepi_ros.get_param(self,"~max_pan_angle",self.init_max_pan)
    status_msg.set_pan_min_max_deg = [min_pan,max_pan]

    min_tilt = nepi_ros.get_param(self,"~min_tilt_angle",self.init_min_tilt)
    max_tilt = nepi_ros.get_param(self,"~max_tilt_angle",self.init_max_tilt)
    status_msg.set_tilt_min_max_deg = [min_tilt,max_tilt]
    

    status_msg.max_scan_time_sec = self.MAX_SCAN_TIME
    status_msg.scan_time_sec = nepi_ros.get_param(self,"~scan_time",self.init_scan_time)
    status_msg.min_area_ratio = nepi_ros.get_param(self,"~min_area_ratio",self.init_min_area_ratio)
    status_msg.scan_speed_ratio = nepi_ros.get_param(self,"~scan_speed_ratio",self.init_scan_speed_ratio)
    status_msg.scan_tilt_offset = nepi_ros.get_param(self,"~scan_tilt_offset",self.init_scan_tilt_offset)

    status_msg.track_speed_ratio = nepi_ros.get_param(self,"~track_speed_ratio",self.init_track_speed_ratio)
    status_msg.track_tilt_offset = nepi_ros.get_param(self,"~track_tilt_offset", self.init_track_tilt_offset)

    status_msg.error_goal_min_max_deg = self.MIN_MAX_ERROR_GOAL
    status_msg.error_goal_deg = nepi_ros.get_param(self,"~error_goal",self.init_error_goal)

    status_msg.is_scanning = self.is_scanning
    status_msg.is_tracking = self.is_tracking

    self.status_pub.publish(status_msg)



  def updaterCb(self,timer):
    # Save last image topic for next check
    self.last_image_topic = self.current_image_topic
    ############## DEBUG
    #nepi_ros.set_param(self,"~app_enabled",True)
    #nepi_ros.set_param(self,"~pt_namespace","/nepi/s2x/iqr_pan_tilt/ptx")
    #nepi_ros.set_param(self,"~selected_class","person")
    ############## DEBUG

    update_status = False
    app_enabled = nepi_ros.get_param(self,"~app_enabled", self.init_app_enabled)

    app_msg = ""
    #nepi_msg.publishMsgWarn(self," Running app update process with app enabled: " + str(app_enabled))
    if app_enabled == False:
      app_msg += "App not enabled"
      self.target_detected=False
      self.current_image_topic = "None"
      if self.image_sub is not None:
        nepi_msg.publishMsgWarn(self," App Disabled, Unsubscribing from Image topic : " + self.last_image_topic)
        self.image_sub.unregister()
        time.sleep(1)
        self.image_sub = None
    elif self.last_app_enabled != app_enabled:
      update_status = True
    self.last_app_enabled = app_enabled
    
    # Setup PT subscribers and Publishers if needed
    sel_pt = nepi_ros.get_param(self,'~pt_namespace',  self.init_pt_namespace)
    #nepi_msg.publishMsgWarn(self," Selected PT: " + str(sel_pt))
    #nepi_msg.publishMsgWarn(self," Last Selected PT: " + str(self.last_sel_pt))
    pt_needs_update = self.last_sel_pt != sel_pt
    self.last_sel_pt = sel_pt
    if pt_needs_update and sel_pt != "":
      if sel_pt == "None" and self.pt_status_sub is not None:
        self.removePtSubs()
      elif app_enabled == True and sel_pt != "None":
        self.setupPtSubs(sel_pt)
        update_status = True
      else:
        self.last_sel_pt = ""
    # Check if PT still there
    if self.pt_connected == True:
      if self.pt_status_topic != "":
        pt_status_topic=nepi_ros.find_topic(self.pt_status_topic)
        if pt_status_topic == "":
          nepi_msg.publishMsgWarn(self,"PT lost: " + self.pt_status_topic)
          self.removePtSubs()
          update_status = True
    if self.pt_connected == True:
      app_msg += ", PanTilt connected"
    else:
      app_msg += ", PanTilt not connected"


    # Update classifier info
    ai_mgr_status_response = None
    try:
      ai_mgr_status_response = self.get_ai_mgr_status_service()
      #nepi_msg.publishMsgInfo(self," Got classifier status  " + str(ai_mgr_status_response))
    except Exception as e:
      ai_mgr_status_response = None
      nepi_msg.publishMsgWarn(self,"Failed to call AI MGR STATUS service" + str(e))
      self.classifier_running = False
      nepi_ros.set_param(self,'~last_classiier', "")
      #app_msg += ", AI Detector not connected"
    if ai_mgr_status_response != None:
      #app_msg += ", AI Detector connected"
      status_str = str(ai_mgr_status_response)
      #nepi_msg.publishMsgWarn(self," got ai manager status: " + status_str)
      self.current_image_topic = ai_mgr_status_response.selected_img_topic
      self.current_classifier = ai_mgr_status_response.selected_classifier
      self.current_classifier_state = ai_mgr_status_response.classifier_state
      self.classifier_running = self.current_classifier_state == "Running"
      classes_list = ai_mgr_status_response.selected_classifier_classes
      if classes_list != self.classes_list:
        self.classes_list = classes_list
        #classes_str = str(self.classes_list)
        #nepi_msg.publishMsgWarn(self," got ai manager status: " + classes_str)
        update_status = True
      selected_class = nepi_ros.get_param(self,'~selected_class', self.init_sel_class)
      last_classifier = nepi_ros.get_param(self,'~last_classiier', self.init_last_classifier)
      if last_classifier != self.current_classifier and self.current_classifier != "None":
        selected_class = "None" # Reset classes to all on new classifier
        update_status = True
      self.selected_class = selected_class
      #nepi_ros.set_param(self,'~selected_class', selected_class)
      nepi_ros.set_param(self,'~last_classiier', self.current_classifier)
      #nepi_msg.publishMsgWarn(self," Got image topics last and current: " + self.last_image_topic + " " + self.current_image_topic)

      # Update Image Topic Subscriber
      if self.classifier_running == False:
        app_msg += ", Classifier not running"
        self.target_detected=False
        self.current_image_topic = "None"
      else:
        app_msg += ", Classifier running"

      # Update image topic subs
      #nepi_msg.publishMsgWarn(self," Current Image Topic: " + self.current_image_topic)
      #nepi_msg.publishMsgWarn(self," Last Image Topic: " + self.last_image_topic)
      if (self.last_image_topic != self.current_image_topic) or (self.image_sub == None and self.current_image_topic != "None") or self.reset_image_topic == True:
        self.reset_image_topic = False
        image_topic = nepi_ros.find_topic(self.current_image_topic)
        if image_topic == "":
          nepi_msg.publishMsgWarn(self," Could not find image update topic: " + self.current_image_topic)
        elif app_enabled == True and image_topic != "None":
          nepi_msg.publishMsgInfo(self," Found detect image update topic : " + image_topic)
          update_status = True
          if self.image_sub != None:
            nepi_msg.publishMsgWarn(self," Unsubscribing to Image topic : " + self.last_image_topic)
            self.image_sub.unregister()
            time.sleep(1)
            self.image_sub = None
          nepi_msg.publishMsgInfo(self," Subscribing to Image topic : " + image_topic)
          self.image_sub = rospy.Subscriber(image_topic, Image, self.imageCb, queue_size = 1)
        else:
          self.last_image_topic = ""

        if self.current_image_topic == "None" or self.current_image_topic == "":  # Reset last image topic
          if self.image_sub != None:
            nepi_msg.publishMsgWarn(self," Unsubscribing to Image topic : " + self.current_image_topic)
            self.image_sub.unregister()
            time.sleep(1)
            self.image_sub = None
            update_status = True
            time.sleep(1)
    # Check for img subscribers
    if self.image_sub is not None:
      self.img_has_subs = (self.image_sub.get_num_connections() > 0)

    # Check class selection
    class_sel = False
    #nepi_msg.publishMsgWarn(self," sel class: " + sel_class)
    if len(self.classes_list) > 0:
      if self.selected_class  in self.classes_list:
        class_sel = True
      if class_sel == False:
        app_msg += ", Target not selected"
      else:
        app_msg += ", Target selected"
    self.class_selected = class_sel


    # Update status app msg
    self.app_msg = app_msg
    # Publish status if needed
    if update_status == True:
      #nepi_msg.publishMsgInfo(self," App update process msg: " + app_msg)
      self.publish_status()
    
  

  def setupPtSubs(self,pt_namespace):
    if pt_namespace is not None:
      if pt_namespace != "None" and pt_namespace != "":
        pt_status_topic = os.path.join(pt_namespace,"/ptx/status")
        nepi_msg.publishMsgInfo(self,"Waiting for topic name: " + pt_status_topic)
        self.pt_status_topic=nepi_ros.find_topic(pt_status_topic)
        if self.pt_status_sub is not None:
          self.removePtSubs()
        if self.pt_status_topic != "":
          nepi_msg.publishMsgInfo(self,"Found ptx status topic: " + self.pt_status_topic)
          ptx_namespace = self.pt_status_topic.replace("status","")
          nepi_msg.publishMsgInfo(self,"Found ptx namespace: " + ptx_namespace)
          self.pt_namespace = ptx_namespace.split("/ptx")[0]
          # PanTilt Status Topics
          # PanTilt Control Publish Topics
          PTX_SET_SPEED_RATIO_TOPIC = ptx_namespace + "set_speed_ratio"
          PTX_GOHOME_TOPIC = ptx_namespace + "go_home"
          PTX_STOP_TOPIC = ptx_namespace + "stop_moving"
          PTX_GOTO_PAN_RATIO_TOPIC = ptx_namespace + "jog_to_yaw_ratio"
          PTX_GOTO_TILT_RATIO_TOPIC = ptx_namespace + "jog_to_pitch_ratio"
          PTX_JOG_PAN_TOPIC = ptx_namespace + "jog_timed_yaw"
          PTX_JOG_TILT_TOPIC = ptx_namespace + "jog_timed_pitch"
          PTX_JOG_POSITION_TOPIC = ptx_namespace + "jog_to_position"
          PTX_SET_SOFT_LIMITS_TOPIC = ptx_namespace + "set_soft_limits"

          ## Get PTX capabilities info
          ptx_capabilities_service_topic = ptx_namespace + "capabilities_query"
          try:
            ptx_caps_service = rospy.ServiceProxy(ptx_capabilities_service_topic, PTXCapabilitiesQuery)
            time.sleep(1)
            ptx_caps = ptx_caps_service()
            self.has_position_feedback = ptx_caps.absolute_positioning
            self.has_adjustable_speed =  ptx_caps.adjustable_speed
          except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to call PTX capabilities service: " + ptx_capabilities_service_topic + " " + str(e))
            self.has_position_feedback = False
            self.has_adjustable_speed =  False


          ## Create Class Subscribers
          nepi_msg.publishMsgInfo(self,"Subscribing to PTX Status Msg: " + self.pt_status_topic)

          ## Create Class Publishers
          self.send_pt_home_pub = rospy.Publisher(PTX_GOHOME_TOPIC, Empty, queue_size=10)
          self.set_pt_speed_ratio_pub = rospy.Publisher(PTX_SET_SPEED_RATIO_TOPIC, Float32, queue_size=10)
          self.set_pt_position_pub = rospy.Publisher(PTX_JOG_POSITION_TOPIC, PanTiltPosition, queue_size=10)
          self.set_pt_pan_ratio_pub = rospy.Publisher(PTX_GOTO_PAN_RATIO_TOPIC, Float32, queue_size=10)
          self.set_pt_tilt_ratio_pub = rospy.Publisher(PTX_GOTO_TILT_RATIO_TOPIC, Float32, queue_size=10)
          self.set_pt_pan_jog_pub = rospy.Publisher(PTX_JOG_PAN_TOPIC, SingleAxisTimedMove, queue_size=10)
          self.set_pt_tilt_jog_pub = rospy.Publisher(PTX_JOG_TILT_TOPIC, SingleAxisTimedMove, queue_size=10)
          self.set_pt_soft_limits_pub = rospy.Publisher(PTX_SET_SOFT_LIMITS_TOPIC, PanTiltLimits, queue_size=10)
          self.pt_stop_motion_pub = rospy.Publisher(PTX_STOP_TOPIC, Empty, queue_size=10)
          time.sleep(1)
          self.pt_status_sub = rospy.Subscriber(self.pt_status_topic, PanTiltStatus, self.ptStatusCb, queue_size = 1)

      else:
        self.pt_namespace = "None"
        self.pt_connected = False
        self.pitch_yaw_errors_deg = [0,0]
    else:
      self.pt_namespace = "None"
      self.pt_connected = False
      self.pitch_yaw_errors_deg = [0,0]
      

  def removePtSubs(self):
    if self.pt_status_sub is not None:
      ## Create Class Subscribers
      nepi_msg.publishMsgInfo(self,"Unsubscribing to PTX Status Msg: " + self.pt_status_topic)
      self.pt_status_sub.unregister()
      self.send_pt_home_pub.unregister()
      self.set_pt_speed_ratio_pub.unregister()
      self.set_pt_position_pub.unregister()
      self.set_pt_pan_ratio_pub.unregister()
      self.set_pt_tilt_ratio_pub.unregister()
      self.set_pt_pan_jog_pub.unregister()
      self.set_pt_tilt_jog_pub.unregister()
      self.set_pt_soft_limits_pub.unregister()
      self.pt_stop_motion_pub.unregister()

      time.sleep(1)

      self.pt_status_sub = None
      self.send_pt_home_pub = None
      self.set_pt_speed_ratio_pub = None
      self.set_pt_position_pub = None
      self.set_pt_pan_ratio_pub = None
      self.set_pt_tilt_ratio_pub = None
      self.set_pt_pan_jog_pub = None
      self.set_pt_tilt_jog_pub = None
      self.set_pt_soft_limits_pub = None
      self.pt_stop_motion_pub = None


    self.has_position_feedback = False
    self.has_adjustable_speed =  False

    self.pt_namespace = "None"
    self.pt_connected = False
    self.pitch_yaw_errors_deg = [0,0]

  #######################
  ### Node Callbacks

  def pubStatusCb(self,msg):
    self.publish_status()

  def appEnableCb(self,msg):
    #nepi_msg.publishMsgInfo(self,msg)
    val = msg.data
    nepi_ros.set_param(self,'~app_enabled',val)
    self.publish_status()



  def setClassCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    selected_class = msg.data
    if selected_class in self.classes_list or selected_class == "None":
      nepi_ros.set_param(self,'~selected_class',  selected_class)
      self.selected_class = selected_class
    self.publish_status()

  def setPtTopicCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    pt_topic = msg.data
    nepi_ros.set_param(self,'~pt_namespace',  pt_topic)
    self.publish_status()

  def setVertFovCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    fov = msg.data
    if fov > 0:
      nepi_ros.set_param(self,'~image_fov_vert',  fov)
    self.publish_status()


  def setHorzFovCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    fov = msg.data
    if fov > 0:
      nepi_ros.set_param(self,'~image_fov_horz',  fov)
    self.publish_status()


  def setTimedScanCb(self,msg):
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

    min_tilt = nepi_ros.get_param(self,"~min_tilt_angle",self.init_min_tilt)
    max_tilt = nepi_ros.get_param(self,"~max_tilt_angle",self.init_max_tilt)
    if val >= min_tilt and val <= max_tilt:
      nepi_ros.set_param(self,'~scan_tilt_offset',  val)
    self.publish_status()

  def setMinMaxPanCb(self,msg):
    min_pan = msg.start_range
    max_pan = msg.stop_range
    ##nepi_msg.publishMsgInfo(self,msg)
    if min_pan >= -180 and max_pan <= 180 and min_pan < max_pan:
      nepi_ros.set_param(self,"~min_pan_angle",min_pan)
      nepi_ros.set_param(self,"~max_pan_angle",max_pan)
    self.publish_status()

  def setMinMaxTiltCb(self,msg):
    min_tilt = msg.start_range
    max_tilt = msg.stop_range
    ##nepi_msg.publishMsgInfo(self,msg)
    if min_tilt >= -180 and max_tilt <= 180 and min_tilt < max_tilt:
      nepi_ros.set_param(self,"~min_tilt_angle",min_tilt)
      nepi_ros.set_param(self,"~max_tilt_angle",max_tilt)
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
    min_pan = nepi_ros.get_param(self,"~min_pan_angle",self.init_min_pan)
    max_pan = nepi_ros.get_param(self,"~max_pan_angle",self.init_max_pan)
    min_tilt = nepi_ros.get_param(self,"~min_tilt_angle",self.init_min_tilt)
    max_tilt = nepi_ros.get_param(self,"~max_tilt_angle",self.init_max_tilt)
    if val >= min_tilt and val <= max_tilt:
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
  def ptStatusCb(self,pt_status_msg):
    # This is just to get the current pt positions
    self.pt_status_msg = pt_status_msg
    if self.pt_status_msg.yaw_now_deg != self.last_pan_pos:
      self.last_pan_pos = self.pt_status_msg.yaw_now_deg
      self.is_moving = True
    else:
      self.is_moving = False

    pan_min = self.pt_status_msg.yaw_min_softstop_deg
    pan_max = self.pt_status_msg.yaw_max_softstop_deg
    min_pan = nepi_ros.get_param(self,"~min_pan_angle",self.init_min_pan)
    max_pan = nepi_ros.get_param(self,"~max_pan_angle",self.init_max_pan)
    if min_pan < pan_min:
      min_pan = pan_min
    if max_pan > pan_max:
      max_pan = pan_max
    nepi_ros.set_param(self,"~min_pan_angle",min_pan)
    nepi_ros.set_param(self,"~max_pan_angle",max_pan)
    

    tilt_min = self.pt_status_msg.pitch_min_softstop_deg
    tilt_max = self.pt_status_msg.pitch_max_softstop_deg
    min_tilt = nepi_ros.get_param(self,"~min_tilt_angle",self.init_min_tilt)
    max_tilt = nepi_ros.get_param(self,"~max_tilt_angle",self.init_max_tilt)
    if min_tilt < tilt_min:
      min_tilt = tilt_min
    if max_tilt > tilt_max:
      max_tilt = tilt_max
    nepi_ros.set_param(self,"~min_tilt_angle",min_tilt)
    nepi_ros.set_param(self,"~max_tilt_angle",max_tilt)

    self.pt_connected = True
    


  def scanTrackCb(self,timer):
    app_enabled = nepi_ros.get_param(self,"~app_enabled", self.init_app_enabled)
    self.target_box_lock.acquire()
    box = copy.deepcopy(self.target_box)      
    self.target_box_lock.release()

    min_pan = nepi_ros.get_param(self,"~min_pan_angle",self.init_min_pan)
    max_pan = nepi_ros.get_param(self,"~max_pan_angle",self.init_max_pan)
    min_tilt = nepi_ros.get_param(self,"~min_tilt_angle",self.init_min_tilt)
    max_tilt = nepi_ros.get_param(self,"~max_tilt_angle",self.init_max_tilt)

    lost_target = self.lost_target_count > self.LOST_TARGET_COUNT_LIMIT

    #nepi_msg.publishMsgWarn(self," Running scan track process with app enabled: " + str(app_enabled))
    #nepi_msg.publishMsgWarn(self," Running scan track process with pt_connected: " + str(self.pt_connected))
    #nepi_msg.publishMsgWarn(self," Running scan track process with pt_status valid: " + str(self.pt_status_msg is not None))
    if app_enabled == False or self.pt_connected == False or self.pt_status_msg is None or self.class_selected == False:
      #nepi_msg.publishMsgWarn(self," Scan Track process not ready")
      self.target_detected=False
      self.is_tracking = False
      self.is_scanning = False
    else:  
      was_tracking = copy.deepcopy(self.is_tracking)
      if was_tracking == False:
        self.publish_status()


      # publish error and make change
      if box is not None:
        #nepi_msg.publishMsgWarn(self,"Tracking on target box: " + str(box))
        self.is_tracking = True
        self.is_scanning = False
        error_goal = nepi_ros.get_param(self,"~error_goal",self.init_error_goal)
        track_speed_ratio = nepi_ros.get_param(self,"~track_speed_ratio",self.init_track_speed_ratio)
        track_tilt_offset = nepi_ros.get_param(self,"~track_tilt_offset", self.init_track_tilt_offset)
        if self.has_adjustable_speed == True:
          self.set_pt_speed_ratio_pub.publish(track_speed_ratio)

        tilt_cur = self.pt_status_msg.pitch_now_deg
        tilt_goal = self.pt_status_msg.pitch_goal_deg

        pan_cur = self.pt_status_msg.yaw_now_deg
        pan_goal = self.pt_status_msg.yaw_goal_deg


        [pan_error,tilt_error] = self.get_target_bearings(box)
        tilt_error = tilt_error + track_tilt_offset
        self.pitch_yaw_errors_deg = [pan_error,tilt_error]
        #nepi_msg.publishMsgWarn(self,"Error Goal set to: " + str(error_goal))
        #nepi_msg.publishMsgWarn(self,"Got Targets Errors pan tilt: " + str(self.pitch_yaw_errors_deg))

        if abs(tilt_error) < error_goal and abs(pan_error) < error_goal:
            #nepi_msg.publishMsgWarn(self,"Tracking within error bounds")
            self.pt_stop_motion_pub.publish(Empty())
        else:
            # Set pan angle goal
            if abs(pan_error) > error_goal:
                pan_to_goal = pan_cur + pan_error/3
            else:
                pan_to_goal = pan_cur
            if pan_to_goal < min_pan:
                pan_to_goal = min_pan
            if pan_to_goal > max_pan:
                pan_to_goal = max_pan
            # Set tilt angle goal
            if abs(tilt_error) > error_goal:
                tilt_to_goal = tilt_cur + tilt_error/3 
            else:
                tilt_to_goal = tilt_cur
            if tilt_to_goal < min_tilt:
                tilt_to_goal = min_tilt
            if tilt_to_goal > max_tilt:
                tilt_to_goal = max_tilt

            #nepi_msg.publishMsgWarn(self,"Current Pos: " + str([pan_cur,tilt_cur]))
            #nepi_msg.publishMsgWarn(self,"Track to Pos: " + str([pan_to_goal,tilt_to_goal]))
            if self.has_position_feedback == True:
              # Send angle goal
              pt_pos_msg = PanTiltPosition()
              pt_pos_msg.yaw_deg = pan_to_goal
              pt_pos_msg.pitch_deg = tilt_to_goal
              if not nepi_ros.is_shutdown():
                  self.set_pt_position_pub.publish(pt_pos_msg)   
              else:
                pass # add timed jog controls


            if pan_error > 0:
                self.last_track_dir = 1
            else: 
                self.last_track_dir = -1
            #nepi_msg.publishMsgWarn(self,"Track dir: " + str(self.last_track_dir))
      elif lost_target == True:
        was_scanning = copy.deepcopy(self.is_scanning)
        self.is_tracking = False
        self.is_scanning = True
        self.pitch_yaw_errors_deg = [0,0]

        scan_speed_ratio = nepi_ros.get_param(self,"~scan_speed_ratio",self.init_scan_speed_ratio)
        scan_tilt_offset = nepi_ros.get_param(self,"~scan_tilt_offset",self.init_scan_tilt_offset)

        
        # Check tilt limits
        if scan_tilt_offset < min_tilt:
          scan_tilt_offset = min_tilt
        if scan_tilt_offset > max_tilt:
          scan_tilt_offset = max_tilt
        nepi_ros.set_param(self,"~scan_tilt_offset",scan_tilt_offset)

        tilt_cur = self.pt_status_msg.pitch_now_deg
        tilt_goal = self.pt_status_msg.pitch_goal_deg
        pan_cur = self.pt_status_msg.yaw_now_deg
        pan_goal = self.pt_status_msg.yaw_goal_deg

        start_scanning = False
        if was_scanning == False:
            start_scanning = True
            self.current_scan_dir = self.last_track_dir
            self.publish_status()

        if self.has_adjustable_speed == True:
          self.set_pt_speed_ratio_pub.publish(scan_speed_ratio)

        #nepi_msg.publishMsgWarn(self,"Scanning in direction: " + str(self.current_scan_dir))
        #nepi_msg.publishMsgWarn(self,"With tilt angle: " + str(scan_tilt_offset))

        # Check if scan dir change needed
        check_str = str([pan_cur,min_pan + 5,max_pan - 5,self.current_scan_dir])
        #nepi_msg.publishMsgWarn(self,"Scanning with scan checks: " + check_str)


        if self.has_position_feedback == True:
          if ((pan_cur < (min_pan + 5)) and self.current_scan_dir != 1):
            pan_tilt_pos_msg = PanTiltPosition()
            pan_tilt_pos_msg.yaw_deg = max_pan
            pan_tilt_pos_msg.pitch_deg = scan_tilt_offset
            self.set_pt_position_pub.publish(pan_tilt_pos_msg)
            self.current_scan_dir = 1
            #nepi_msg.publishMsgWarn(self,"Changed to scan dir: " + str(self.current_scan_dir))

          elif (pan_cur > (max_pan - 5)) and self.current_scan_dir != -1:
            pan_tilt_pos_msg = PanTiltPosition()
            pan_tilt_pos_msg.yaw_deg = min_pan
            pan_tilt_pos_msg.pitch_deg = scan_tilt_offset
            self.set_pt_position_pub.publish(pan_tilt_pos_msg)
            self.current_scan_dir = -1
            #nepi_msg.publishMsgInfo(self,"Changed to scan dir: " + str(self.current_scan_dir))

          elif start_scanning == True or self.is_moving == False:
            if self.current_scan_dir > 0:
              pan_tilt_pos_msg = PanTiltPosition()
              pan_tilt_pos_msg.yaw_deg = max_pan
              pan_tilt_pos_msg.pitch_deg = scan_tilt_offset
              self.set_pt_position_pub.publish(pan_tilt_pos_msg)
            else:
              pan_tilt_pos_msg = PanTiltPosition()
              pan_tilt_pos_msg.yaw_deg = min_pan
              pan_tilt_pos_msg.pitch_deg = scan_tilt_offset
              self.set_pt_position_pub.publish(pan_tilt_pos_msg)
              #nepi_msg.publishMsgInfo(self,"Changed to scan dir: " + str(self.current_scan_dir))
        else:
          pass # Add timed jog controls

    # Publish errors msg
    errors_msg = TrackingErrors()
    errors_msg.pitch_deg = self.pitch_yaw_errors_deg[0]
    errors_msg.yaw_deg = self.pitch_yaw_errors_deg[1]
    self.errors_pub.publish(errors_msg)

    # Publish status message




  def get_target_bearings(self,box):
      target_vert_angle_deg = 0
      target_horz_angle_deg = 0
      if self.img_height != 0 and self.img_width != 0:
        # Iterate over all of the objects and calculate range and bearing data
        image_fov_vert = nepi_ros.get_param(self,'~image_fov_vert',  self.init_image_fov_vert)
        image_fov_horz = nepi_ros.get_param(self,'~image_fov_horz', self.init_image_fov_horz)
        box_y = box.ymin + (box.ymax - box.ymin)
        box_x = box.xmin + (box.xmax - box.xmin)
        box_center = [box_y,box_x]
        y_len = (box.ymax - box.ymin)
        x_len = (box.xmax - box.xmin)
        # Calculate target bearings
        object_loc_y_pix = float(box.ymin + ((box.ymax - box.ymin))  / 2) 
        object_loc_x_pix = float(box.xmin + ((box.xmax - box.xmin))  / 2)
        object_loc_y_ratio_from_center = float(object_loc_y_pix - self.img_height/2) / float(self.img_height/2)
        object_loc_x_ratio_from_center = float(object_loc_x_pix - self.img_width/2) / float(self.img_width/2)
        target_vert_angle_deg = (object_loc_y_ratio_from_center * float(image_fov_vert/2))
        target_horz_angle_deg = -1* (object_loc_x_ratio_from_center * float(image_fov_horz/2))
      return target_horz_angle_deg, target_vert_angle_deg



  def imagePubCb(self,timer):
    data_product = 'tracking_image'
    has_subscribers = self.img_has_subs
    saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
    snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
    app_enabled = nepi_ros.get_param(self,"~app_enabled", self.init_app_enabled)
    if app_enabled == False:
      #nepi_msg.publishMsgWarn(self,"Publishing Not Enabled image")
      if not nepi_ros.is_shutdown():
        self.app_ne_img.header.stamp = nepi_ros.time_now()
        self.image_pub.publish(self.app_ne_img)
    elif self.image_sub == None and has_subscribers:
      if not nepi_ros.is_shutdown():
        self.classifier_nr_img.header.stamp = nepi_ros.time_now()
        self.image_pub.publish(self.classifier_nr_img)
    elif has_subscribers or saving_is_enabled or snapshot_enabled:
      self.img_lock.acquire()
      img_msg = copy.deepcopy(self.img_msg)
      self.img_lock.release()
      if img_msg is not None:
        ros_timestamp = img_msg.header.stamp
        self.target_box_lock.acquire()
        box = copy.deepcopy(self.target_box)      
        self.target_box_lock.release()
        #nepi_msg.publishMsgWarn(self,"Got image to publish")
        if box is not None:
          #nepi_msg.publishMsgWarn(self,"Overlaying target on image: " + str(box))
          current_image_header = img_msg.header
          ros_timestamp = img_msg.header.stamp     
          cv2_img = nepi_img.rosimg_to_cv2img(img_msg).astype(np.uint8)

          #nepi_msg.publishMsgWarn(self," Box: " + str(box))
          class_name = box.Class

          [xmin,xmax,ymin,ymax] = [box.xmin,box.xmax,box.ymin,box.ymax]
          start_point = (xmin, ymin)
          end_point = (xmax, ymax)
          class_name = class_name
          class_color = (0,255,0)
          line_thickness = 2
          cv2.rectangle(cv2_img, start_point, end_point, class_color, thickness=line_thickness)

          # Publish new image to ros
          if not nepi_ros.is_shutdown() and has_subscribers: #and has_subscribers:
              #Convert OpenCV image to ROS image
              cv2_shape = cv2_img.shape
              if  cv2_shape[2] == 3:
                encode = 'bgr8'
              else:
                encode = 'mono8'
              img_out_msg = nepi_img.cv2img_to_rosimg(cv2_img, encoding=encode)
              img_out_msg.header.stamp = ros_timestamp
              self.image_pub.publish(img_out_msg)
          # Save Data if Time
          if saving_is_enabled or snapshot_enabled:
            nepi_save.save_img2file(self,data_product,cv2_img,ros_timestamp,save_check = False)
        else:
            if not nepi_ros.is_shutdown() and has_subscribers:
              self.image_pub.publish(img_msg)
        # Set up next scan track process

    ### If object(s) detected, save bounding box info to global
  def objectDetectedCb(self,bounding_boxes_msg):
    app_enabled = nepi_ros.get_param(self,"~app_enabled", self.init_app_enabled)
    selected_class = self.selected_class 
    min_area_ratio =  nepi_ros.get_param(self,"~min_area_ratio",self.init_min_area_ratio)
    ros_timestamp = bounding_boxes_msg.header.stamp
    bb_list = bounding_boxes_msg.bounding_boxes
    self.img_height = bounding_boxes_msg.image_height
    self.img_width = bounding_boxes_msg.image_width
    if app_enabled == False:
      self.target_box_lock.acquire()
      self.target_box = None      
      self.target_box_lock.release()
    else:
      #nepi_msg.publishMsgWarn(self,"Got Targets Locs: " + str(target_locs_msg))
      # Iterate over all of the objects reported by the detector and return center of largest box in degrees relative to img center
      largest_target = None
      largest_box_area_ratio=0 # Initialize largest box area
      for box in bb_list:
        # Check for the object of interest and take appropriate actions
        #nepi_msg.publishMsgWarn(self,"Looking for selected target: " + str(selected_class))
        if box.Class == selected_class:
          # Check if largest box
          box_area_ratio = box.area_ratio
          if box_area_ratio > largest_box_area_ratio:
            largest_box_area_ratio=box_area_ratio
            largest_target=box
      if largest_box_area_ratio < min_area_ratio:
            self.lost_target_count += 1
            self.target_detected = False
            self.target_box_lock.acquire()
            self.target_box = None      
            self.target_box_lock.release()
      else:
            #nepi_msg.publishMsgWarn(self,"Got target loc: " + str(largest_target))
            self.lost_target_count = 0
            self.target_detected = True
            self.target_box_lock.acquire()
            self.target_box = largest_target      
            self.target_box_lock.release()

            




  ### Monitor Output of AI model to clear detection status
  def foundObjectCb(self,found_obj_msg):
    app_enabled = nepi_ros.get_param(self,"~app_enabled", self.init_app_enabled)
    #Clean Up
    if found_obj_msg.count == 0:      
      self.lost_target_count += 1
      self.target_detected = False
      self.target_box_lock.acquire()
      self.target_box = None      
      self.target_box_lock.release()




  def imageCb(self,img_msg):    
      self.img_lock.acquire()
      self.img_msg = img_msg
      self.img_lock.release()
      

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    global led_intensity_pub
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")




#########################################
# Main
#########################################
if __name__ == '__main__':
  pantiltTargetTrackerApp()


