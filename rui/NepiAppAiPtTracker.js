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

import {createShortUniqueValues, onDropdownSelectedSendStr, createMenuListFromStrList, createShortValuesFromNamespaces, onChangeSwitchStateValue, onUpdateSetStateValue, onEnterSendFloatValue} from "./Utilities"

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

      app_enabled: false,
      app_msg: "Connecting",

      image_name: "tracking_image",
      show_detector_box: false,

      classifier_running: false,


      image_topic: "None",
      image_fov_vert_degs: null,
      image_fov_horz_degs: null,

            
      available_classes_list: [],
      selected_class: "None",
      target_detected: false,
            
      selected_pantilt: "None",
      pantilt_connected: false,
      has_position_feedback: false,
      tilt_min_max_deg: [-20,20],
      pan_min_max_deg: [-180,180],
      max_scan_time_sec: null,
      scan_time_sec: null,

      min_area_ratio: null,
      scan_speed_ratio: null,
      scan_tilt_offset: null,
      scan_pan_angle: null,
      track_speed_ratio: null,
      track_tilt_offset: null,
      
      error_goal_min_max_deg: [1,20],
      error_goal_deg: null,

      is_scanning: false,
      is_tracking: false,

      statusListener: null,
      statusErrorListener: null,
      ptListener: null,
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

      app_enabled: message.app_enabled,
      app_msg: message.app_msg,

      classifier_running: message.classifier_running,
            
      image_topic: message.image_topic,
      image_fov_vert_degs: message.image_fov_vert_degs,
      image_fov_horz_degs: message.image_fov_horz_degs,      

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
      const statusNamespace = namespace + '/errors'
      if (this.state.statusErrorListener) {
        this.state.statusErrorListener.unsubscribe()
      }
      var statusErrorListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_app_ai_pt_tracker/TrackingErrors",
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



// Function for creating topic options for Select input
createPTXOptions() {
  const { ptxUnits} = this.props.ros
  const topics = Object.keys(ptxUnits)
  var filteredTopics = topics
  const sel_pt = this.state.selected_pantilt
  var i
  var items = []
  items.push(<Option>{"None"}</Option>)
  var unique_names = createShortUniqueValues(topics)
  for (i = 0; i < topics.length; i++) {
    items.push(<Option value={topics[i]}>{unique_names[i]}</Option>)
  }
  return items
}




renderApp() {
  const {sendTriggerMsg, sendStringMsg, sendBoolMsg, ptxUnits} = this.props.ros
  const pan_tilt_options = this.createPTXOptions()
  const pt_connected = this.state.pt_connected
  const sel_pt = this.state.selected_pantilt
  const NoneOption = <Option>None</Option>
  const classifier_running = this.state.classifier_running
  const selectedClass = this.state.selected_class
  const class_sel = selectedClass !== null && selectedClass !== 'None'
  const connected = this.state.connected === true
  const appNamespace = this.getAppNamespace()


  return (
    <Section title={"AI PT Tracking App"}>

      <Columns>
      <Column>


      <Columns>
        <Column>

        <Label title="Enable App">
            <Toggle
            checked={this.state.app_enabled===true}
            onClick={() => sendBoolMsg(appNamespace + "/enable_app",!this.state.app_enabled)}>
            </Toggle>
      </Label>


          </Column>
        <Column>


        </Column>
      </Columns>



      <pre style={{ height: "40px", overflowY: "auto" ,fontWeight: 'bold' , color: Styles.vars.colors.Green, textAlign: "left" }}>
          {this.state.app_msg}
        </pre>

        <Columns>
          <Column>


      <Label title={"Classifier Running"}>
        <BooleanIndicator value={this.state.classifier_running} />
      </Label>


      <Label title={"Pantilt Connected"}>
        <BooleanIndicator value={pt_connected} />
      </Label>

      <Label title={"Target Class Selected"}>
        <BooleanIndicator value={class_sel} />
      </Label>

            </Column>
          <Column>

        <Label title={"Scanning"}>
        <BooleanIndicator value={this.state.is_scanning} />
      </Label>

      <Label title={"Tracking"}>
        <BooleanIndicator value={this.state.is_tracking} />
      </Label>

      <Label title={"Target Detected"}>
        <BooleanIndicator value={this.state.target_detected} />
      </Label>



          <Label title={"Pitch Deg Error"}>
        <Input disabled value={this.state.pitch_deg} />
      </Label>



          <Label title={"Yaw Deg Error"}>
        <Input disabled value={this.state.yaw_deg} />
      </Label> 
  
          </Column>
        </Columns>













      <div hidden={!this.state.classifier_running || !this.state.app_enabled}>
     

      <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
        {"App Settings"}
       </label>


       <Columns>
          <Column>

          <Label title={"Select PanTilt Device"}>
          <Select
            id="pt_select"
            onChange={(event) => onDropdownSelectedSendStr.bind(this)(event, appNamespace + "/select_pantilt")}
            value={this.state.selected_pantilt}
          >
            {(pan_tilt_options.length > 1)
              ? pan_tilt_options
              : NoneOption}
          </Select>
      </Label>

      <Label title={"Has Position Feedback"}>
        <BooleanIndicator value={this.state.has_position_feedback} />
      </Label>


      <SliderAdjustment
          title={"Target Size Filter"}
          msgType={"std_msgs/float32"}
          adjustment={this.state.min_area_ratio}
          topic={appNamespace + "/set_min_area_ratio"}
          scaled={0.01}
          min={0}
          max={100}
          tooltip={""}
          unit={"%"}
      />

            </Column>
          <Column>

          <Label title={"Select Target Class"}>
          <Select
            id="class_select"
            onChange={(event) => onDropdownSelectedSendStr.bind(this)(event, appNamespace + "/select_class")}
            value={this.state.selected_class}
          >
            {this.state.available_classes_list
              ? createMenuListFromStrList(this.state.available_classes_list, false, [],['None'],[])
              : NoneOption}
          </Select>
      </Label>

      <div hidden={this.state.has_position_feedback === false}>

      <SliderAdjustment
          title={"Set Error Goal (Degs)"}
          msgType={"std_msgs/float32"}
          adjustment={this.state.error_goal_deg}
          topic={appNamespace + "/set_error_goal_degs"}
          scaled={1.0}
          min={this.state.error_goal_min_max_deg[0]}
          max={this.state.error_goal_min_max_deg[1]}
          tooltip={""}
          unit={"Degs"}
      />

      </div>
  
          </Column>
        </Columns>



    <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>



    <Columns>
          <Column>

          <Label title={"Sensor Vertical Degrees"}>
          <Input id="image_fov_vert_degs" 
            value={this.state.image_fov_vert_degs} 
            onChange={(event) => onUpdateSetStateValue.bind(this)(event,"image_fov_vert_degs")} 
            onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,appNamespace + "/set_image_fov_vert")} />
        </Label>

            </Column>
          <Column>

          <Label title={"Sensor Horzontal Degrees"}>
          <Input id="image_fov_horz_degs" 
            value={this.state.image_fov_horz_degs} 
            onChange={(event) => onUpdateSetStateValue.bind(this)(event,"image_fov_vert_degs")} 
            onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,appNamespace + "/set_image_fov_horz")} />
        </Label>

          </Column>
        </Columns>





      <Columns>
          <Column>
          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Scan Settings"}
         </label>


      <SliderAdjustment
          title={"Scan Speed Ratio"}
          msgType={"std_msgs/float32"}
          adjustment={this.state.scan_speed_ratio}
          topic={appNamespace + "/set_scan_speed_ratio"}
          scaled={0.01}
          min={0}
          max={100}
          tooltip={""}
          unit={"%"}
      />

      <div hidden={this.state.has_position_feedback === false}>

      <Label title={"Scan Tilt Offset"}>
        <Input
          value={this.state.scan_tilt_offset}
            id="scan_tilt_offset"
            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"scan_tilt_offset")}
            onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event, this.state.appNamespace +"/set_scan_tilt_offset")}
            style={{ width: "80%" }}
        />
      </Label>

      <Label title={"Scan Pan Angle"}>
        <Input
          value={this.state.scan_pan_angle}
            id="scan_pan_angle"
            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"scan_pan_angle")}
            onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event, this.state.appNamespace +"/set_scan_pan_angle")}
            style={{ width: "80%" }}
        />
      </Label>

      </div>


            </Column>
          <Column>


          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Track Settings"}
         </label>

      <SliderAdjustment
          title={"Track Speed Ratio"}
          msgType={"std_msgs/float32"}
          adjustment={this.state.track_speed_ratio}
          topic={appNamespace + "/set_track_speed_ratio"}
          scaled={0.01}
          min={0}
          max={100}
          tooltip={""}
          unit={"%"}
      />



      <div hidden={this.state.has_position_feedback === false}>
      <Label title={"Track Tilt Offset"}>
        <Input
          value={this.state.track_tilt_offset}
            id="track_tilt_offset"
            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"track_tilt_offset")}
            onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event, this.state.appNamespace +"/set_track_tilt_offset")}
            style={{ width: "80%" }}
        />
      </Label>

      </div>

  
          </Column>
        </Columns>


    </div>





    <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

    <Columns>
        <Column>

          <ButtonMenu>
            <Button onClick={() => sendTriggerMsg( appNamespace + "/reset_app")}>{"Reset App"}</Button>
          </ButtonMenu>

          </Column>
        <Column>

            <ButtonMenu>
              <Button onClick={() => sendTriggerMsg(appNamespace + "/save_config")}>{"Save Config"}</Button>
        </ButtonMenu>

        </Column>
        <Column>

        <ButtonMenu>
              <Button onClick={() => sendTriggerMsg( appNamespace + "/reset_config")}>{"Reset Config"}</Button>
        </ButtonMenu>


        </Column>
      </Columns>



    </Column>
      </Columns>

    </Section>

  
  )
}


  render() {
    const connected = this.state.connected === true
    const appNamespace = (connected) ? this.getAppNamespace() : null
    const show_detector_box = this.state.show_detector_box
    const imageNamespace = appNamespace + '/' + this.state.image_name

    return (

      <Columns>
      <Column equalWidth={true}>

       

      <CameraViewer
        imageTopic={imageNamespace}
        title={this.state.image_name}
        hideQualitySelector={false}
      />


      </Column>
      <Column>


      <Columns>
      <Column>

      <Label title="Show Detector Settings">
              <Toggle
              checked={(this.state.show_detector_box === true)}
              onClick={() => onChangeSwitchStateValue.bind(this)("show_detector_box",this.state.show_detector_box)}>
              </Toggle>
        </Label>

      </Column>
      <Column>

    </Column>
    </Columns>



      <div hidden={!show_detector_box}>

      <AiDetectorMgr
              title={"Nepi_Mgr_AI_Detector"}
          />

      </div>


      {this.renderApp()}


      <div hidden={!connected}>

        <NepiIFSaveData
          saveNamespace={appNamespace}
          title={"Nepi_IF_SaveData"}
        />

      </div>

      </Column>
      </Columns>

      )
    }  



}

export default AiPtTrackerApp
