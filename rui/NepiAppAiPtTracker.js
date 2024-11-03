/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import { SliderAdjustment } from "./AdjustmentWidgets"
import Button, { ButtonMenu } from "./Button"
import Label from "./Label"
import Input from "./Input"
import Toggle from "react-toggle"
import Styles from "./Styles"
import BooleanIndicator from "./BooleanIndicator"


import AiDetectorMgr from "./NepiMgrAiDetector"
import CameraViewer from "./CameraViewer"
import NepiIFSaveData from "./Nepi_IF_SaveData"

import {createShortUniqueValues, onDropdownSelectedSendStr, createMenuListFromStrList, createShortValuesFromNamespaces, onUpdateSetStateValue, onEnterSendFloatValue} from "./Utilities"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}

@inject("ros")
@observer

class AiPtTrackerApp extends Component {
  constructor(props) {
    super(props)

    this.state = {
		
      appName: 'app_ai_pt_tracker',
	    appNamespace: null,
      baseNamespace: null,

      tracking_enabled: false,

      classifier_running: false,
      targeting_running: false,

      image_topic: "None",
      use_live_image: false,
      use_last_image: false,
            
      available_classes_list: [],
      selected_class: "None",
      target_detected: false,
            
      selected_pantilt: "None",
      pantilt_connected: false,
      has_position_feedback: false,
      tilt_min_max_deg: [-10,10],
      pan_min_max_deg: [-10,10],
      max_scan_time_sec: 10,
      scan_time_sec: 5,

      min_area_ratio: 0,
      scan_speed_ratio: 0,
      scan_tilt_offset: 0,
      scan_pan_angle: 0,
      track_speed_ratio: 0,
      track_tilt_offset: 0,
      
      error_goal_min_max_deg: [1,10],
      error_goal_deg: 5,

      is_running: false,
      is_scanning: false,
      is_tracking: false,

      statusListener: null,
      statusErrorListener: null,
      connected: false,
      needs_update: true,



      pitch_deg: null,
      yaw_deg: null

    }

    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusErrorListener = this.statusErrorListener.bind(this)
    this.updateStatusErrorListener = this.updateStatusErrorListener.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.createPTXOptions = this.createPTXOptions.bind(this)

    

  }


  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return appNamespace
  }



  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({

      tracking_enabled: message.tracking_enabled,

      classifier_running: message.classifier_running,
      targeting_running: message.targeting_running,
            
      image_topic: message.image_topic,
      use_live_image: message.use_live_image,
      use_last_image: message.use_last_image,

      available_classes_list: message.available_classes_list,
      selected_class: message.selected_class,
      target_detected: message.target_detected,

      selected_pantilt: message.pantilt_device,
      pantilt_connected: message.pantilt_connected,
      has_position_feedback: message.has_position_feedback,
      tilt_min_max_deg: message.tilt_min_max_deg,
      pan_min_max_deg: message.pan_min_max_deg,
      max_scan_time_sec: message.max_scan_time_sec,
      scan_time_sec: message.scan_time_sec,

      min_area_ratio: message.min_area_ratio,
      scan_speed_ratio: message.scan_speed_ratio,
      scan_tilt_offset: message.scan_tilt_offset,
      scan_pan_angle: message.scan_pan_angle,
      track_speed_ratio: message.track_speed_ratio,
      track_tilt_offset: message.track_tilt_offset,
            
      error_goal_min_max_deg: message.error_goal_min_max_deg,
      error_goal_deg: message.error_goal_deg,


      is_scanning: message.is_scanning,
      is_tracking: message.is_tracking  
    
  })

  
  this.setState({
      connected: true
  })


  }
  // Callback for handling ROS Status messages
  statusErrorListener(message) {
    this.setState({

      pitch_deg: message.pitch_deg,
      yaw_deg: message.yaw_deg
    
  })

  this.setState({
      connected: true
  })


}
    // Function for configuring and subscribing to Status
    updateStatusListener() {
      const namespace = this.getAppNamespace()
      const statusNamespace = namespace + '/status'
      if (this.state.statusListener) {
        this.state.statusListener.unsubscribe()
      }
      var statusListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_app_ai_pt_tracker/AiPtTrackerStatus",
            this.statusListener
          )
      this.setState({ 
        statusListener: statusListener,
        needs_update: false
      })
    }

    updateStatusErrorListener() {
      const namespace = this.getAppNamespace()
      const statusNamespace = namespace + '/status'
      if (this.state.statusErrorListener) {
        this.state.statusErrorListener.unsubscribe()
      }
      var statusErrorListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_app_ai_pt_tracker/TrackeringErrors",
            this.statusErrorListener
          )
      this.setState({ 
        statusErrorListener: statusErrorListener,
        needs_update: false
      })
    }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (prevState.appNamespace !== namespace && namespace !== null)
    const needs_update = (this.state.needs_update && namespace !== null)
    if (namespace_updated || needs_update) {
      if (namespace.indexOf('null') === -1){
        this.setState({appNamespace: namespace})
        this.updateStatusListener()
        this.updateStatusErrorListener()
      } 
    }
  }


  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    if (this.state.statusErrorListener) {
      this.state.statusErrorListener.unsubscribe()
    }
  }

 // Function for configuring and subscribing to ptx/status
 onPTXUnitSelected(event) {
  if (this.state.listener) {
    this.state.listener.unsubscribe()
  }

  var idx = event.nativeEvent.target.selectedIndex
  //var text = event.nativeEvent.target[idx].text
  var value = event.target.value

  // Handle the "None" option -- always index 0
  if (idx === 0) {
    this.setState({ disabled: true })
    return
  }

  this.setState({ ptxNamespace: value })

  var listener = this.props.ros.setupPTXStatusListener(
      value,
      this.ptxStatusListener
    )
    
  this.setState({ ptxNamespace: value, listener: listener, disabled: false })
}


// Function for creating topic options for Select input
createPTXOptions(caps_dictionaries, filter) {
  const topics = Object.keys(caps_dictionaries)
  var filteredTopics = topics
  var i
  if (filter) {
    filteredTopics = []
    for (i = 0; i < topics.length; i++) {
      // includes does a substring search
      if (topics[i].includes(filter)) {
        filteredTopics.push(topics[i])
      }
    }
  }

  var items = []
  items.push(<Option>{""}</Option>)
  var unique_names = createShortUniqueValues(filteredTopics)
  for (i = 0; i < filteredTopics.length; i++) {
    items.push(<Option value={filteredTopics[i]}>{unique_names[i]}</Option>)
  }

  return items
}


  renderAppControls() {
    const {sendTriggerMsg, sendStringMsg, sendBoolMsg, ptxUnits} = this.props.ros
    const appNamespace = this.state.appNamespace
    const NoneOption = <Option>None</Option>
    const is_running = this.state.is_running
    const pt_connected = this.state.pt_connected
    const classifier_connected = this.state.classifier_connected
    const can_start = (pt_connected && classifier_connected && is_running === false)
    const pan_tilt_option = this.createPTXOptions(ptxUnits)

    return (
      <div>

      <Columns>
      <Column>

      <Label title={"Tracker Running"}>
      <BooleanIndicator value={is_running} />
    </Label>

      <Label title="Enable Tracking">
              <Toggle
              checked={this.state.tracking_enabled===true}
              onClick={() => sendBoolMsg(appNamespace + "/tracking_enabled",!this.state.tracking_enabled)}>
              </Toggle>
      </Label>



      <Label title={"Classifier Running"}>
        <BooleanIndicator value={this.state.classifier_running} />
      </Label>

      <Label title={"Targeting Running"}>
        <BooleanIndicator value={this.state.targeting_running} />
      </Label>



      <Label title={"Classifier Name"}>
        <Input disabled value={this.state.classifier_name} />
      </Label>

      <Label title={"State"}>
        <Input disabled value={this.state.classifier_state} />
      </Label>

      <Label title={"Connected"}>
        <Input disabled value={this.state.classifier_connected} />
      </Label>

      <Label title={"Image Topic"}>
        <Input disabled value={this.state.image_topic} />
      </Label>

      <Label title="use Live Image">
              <Toggle
              checked={this.state.use_live_image===true}
              onClick={() => sendBoolMsg(appNamespace + "/use_live_image",!this.state.use_live_image)}>
              </Toggle>
      </Label>

      <Label title="use_last_image">
              <Toggle
              checked={this.state.use_last_image===true}
              onClick={() => sendBoolMsg(appNamespace + "/use_last_image",!this.state.use_last_image)}>
              </Toggle>
      </Label>

      </Column>
      <Column>

      <Label title={"Pan/Tilt Unit"}>
              <Select
                onChange={this.onPTXUnitSelected}
                value={this.state.selected_pantilt}
              >
                {this.createPTXOptions(ptxUnits)}
              </Select>
            </Label>

      <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
      <pre style={{ height: "10px", overflowY: "auto" }}>
                {"Scan Options"}
              </pre>
              <Label title={"Has Position Feedback"}>
        <BooleanIndicator value={this.state.has_position_feedback} />
      </Label>

      <div hidden={this.state.has_position_feedback}>
      <Label title={"Scan Speed Ratio"}>
        <Input
          value={this.state.home_lat}
            id="scan_speed_ratio"
            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"scan_speed_ratio")}
            onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event, this.state.appNamespace +"/set_scan_speed_ratio")}
            style={{ width: "80%" }}
        />
      </Label>
      </div>

      <Label title={"Scan Tilt Offset"}>
        <Input
          value={this.state.home_lat}
            id="scan_tilt_offset"
            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"scan_tilt_offset")}
            onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event, this.state.appNamespace +"/set_scan_tilt_offset")}
            style={{ width: "80%" }}
        />
      </Label>

      <Label title={"Scan Pan Angle"}>
        <Input
          value={this.state.home_lat}
            id="scan_pan_angle"
            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"scan_pan_angle")}
            onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event, this.state.appNamespace +"/set_scan_pan_angle")}
            style={{ width: "80%" }}
        />
      </Label>

      <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
      <pre style={{ height: "10px", overflowY: "auto" }}>
                {"Track Options"}
              </pre>


              <Label title={"Track Speed Ratio"}>
        <Input
          value={this.state.home_lat}
            id="track_speed_ratio"
            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"track_speed_ratio")}
            onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event, this.state.appNamespace +"/set_track_speed_ratio")}
            style={{ width: "80%" }}
        />
      </Label>

      <Label title={"Track Tilt Offset"}>
        <Input
          value={this.state.home_lat}
            id="track_tilt_offset"
            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"track_tilt_offset")}
            onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event, this.state.appNamespace +"/set_track_tilt_offset")}
            style={{ width: "80%" }}
        />
      </Label>

      <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>



      </Column>
      <Column>

      <Label title={"Selected Class"}>
          <Select
            id="pt_selected_class"
            onChange={(event) => onDropdownSelectedSendStr.bind(this)(event, appNamespace + "/selected_class")}
            value={this.state.selected_class}
          >
            {this.state.available_classes_list
              ? createMenuListFromStrList(this.state.available_classes_list, false, [],[],[])
              : NoneOption}
          </Select>
      </Label>

      <Label title={"Target Detected"}>
        <BooleanIndicator value={this.state.target_detected} />
      </Label>


      <Label title={"Pantilt Device"}>
        <Input disabled value={this.state.pantilt_device} />
      </Label>

      <Label title={"Pantilt Connected"}>
        <BooleanIndicator value={this.state.pantilt_connected} />
      </Label>


      </Column>
      </Columns>
      </div>
    )
  }


renderAppControls2() {
    const {sendTriggerMsg, sendStringMsg, sendBoolMsg} = this.props.ros
    const appNamespace = this.state.appNamespace
    const NoneOption = <Option>None</Option>
    const is_running = this.state.is_running
    const pt_connected = this.state.pt_connected
    const classifier_connected = this.state.classifier_connected
    const can_start = (pt_connected && classifier_connected && is_running === false)

    return (
      <div>

      <Columns>
      <Column>

      <Label title={"Min Area Ratio"}>
        <Input
          value={this.state.home_lat}
            id="min_area_ratio"
            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"min_area_ratio")}
            onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event, this.state.appNamespace +"/set_min_area_ratio")}
            style={{ width: "80%" }}
        />
      </Label>

     

      </Column>
      <Column>

      <Label title={"Scanning"}>
        <BooleanIndicator value={this.state.is_scanning} />
      </Label>

      <Label title={"Tracking"}>
        <BooleanIndicator value={this.state.is_tracking} />
      </Label>


      <Label title={"Pitch Deg Error"}>
        <Input disabled value={this.state.pitch_deg} />
      </Label>

      <Label title={"Yaw Deg Error"}>
        <Input disabled value={this.state.yaw_deg} />
      </Label>

      </Column>
      </Columns>
      </div>

    )
  }


 renderApp() {
    const {sendTriggerMsg} = this.props.ros
    const appNamespace = this.state.appNamespace
    const NoneOption = <Option>None</Option>
    return (
      <div>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>



        <Columns>
          <Column>

            <ButtonMenu>
              <Button onClick={() => sendTriggerMsg( appNamespace + "/reset_app")}>{"Reset App"}</Button>
            </ButtonMenu>

              <ButtonMenu>
                <Button onClick={() => sendTriggerMsg(appNamespace + "/save_config")}>{"Save Config"}</Button>
          </ButtonMenu>

          <ButtonMenu>
                <Button onClick={() => sendTriggerMsg( appNamespace + "/reset_config")}>{"Reset Config"}</Button>
          </ButtonMenu>

  
          </Column>
        </Columns>

</div>



    )
  }



  renderImageViewer(){
    const connected = this.state.connected
    const appNamespace = this.getAppNamespace()
    const imageNamespace = (connected) ? appNamespace + "/tracking_image" : null 
    return (

      <CameraViewer
        imageTopic={imageNamespace}
        title={this.state.imageText}
        hideQualitySelector={false}
      />

      )
    }  

  render() {
    const namespace = this.getAppNamespace()
    const appNamespace = (this.state.connected) ? namespace: null
    return (

    <Columns>
      <Column>

      {this.renderImageViewer()}

      {this.renderApp()}

      {this.renderAppControls()}

      {this.renderAppControls2()}

      </Column>
    </Columns>

      )
    }  



}

export default AiPtTrackerApp
