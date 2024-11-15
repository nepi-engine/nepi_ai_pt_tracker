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
import RangeAdjustment from "./RangeAdjustment"
import RangeAdjustmentAbs from "./RangeAdjustmentAbs"
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
      has_adjustable_speed: false,

      pan_min: -180,
      pan_max: 180,
      tilt_min: -180,
      tilt_max: 180,
      
      set_pan_min: -60,
      set_pan_max: 60,
      set_tilt_min: -30,
      set_tilt_max: 30,

      max_scan_time_sec: null,
      scan_time_sec: null,

      min_area_ratio: null,
      scan_speed_ratio: null,
      scan_tilt_offset: null,
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
    this.onEnterSendInputBoxRangeWindowValue = this.onEnterSendInputBoxRangeWindowValue.bind(this)
    

  }


  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
      if (this.state.connected === false){
        const pub_status_topic = appNamespace + "/publish_status"
        this.props.ros.sendTriggerMsg(pub_status_topic)
      }
    }
    return appNamespace
  }



  // Callback for handling ROS Status messages
  statusListener(message) {

    const tilt_min_max_deg = message.tilt_min_max_deg
    const pan_min_max_deg = message.pan_min_max_deg

    const set_tilt_min_max_deg = message.set_tilt_min_max_deg
    const set_pan_min_max_deg = message.set_pan_min_max_deg

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
      has_adjustable_speed: message.has_adjustable_speed,

      pan_min: pan_min_max_deg[0],
      pan_max: pan_min_max_deg[1],
      tilt_min: tilt_min_max_deg[0],
      tilt_max: tilt_min_max_deg[1],
      
      set_pan_min: set_pan_min_max_deg[0],
      set_pan_max: set_pan_min_max_deg[1],
      set_tilt_min: set_tilt_min_max_deg[0],
      set_tilt_max: set_tilt_min_max_deg[1],

      max_scan_time_sec: message.max_scan_time_sec,
      scan_time_sec: message.scan_time_sec,

      min_area_ratio: message.min_area_ratio,
      scan_speed_ratio: message.scan_speed_ratio,
      scan_tilt_offset: message.scan_tilt_offset,

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
  items.push(<Option value={"None"}>{"None"}</Option>)
  var unique_names = createShortUniqueValues(topics)
  for (i = 0; i < topics.length; i++) {
    items.push(<Option value={topics[i]}>{unique_names[i]}</Option>)
  }
  return items
}



onEnterSendInputBoxRangeWindowValue(event, topicName, entryName) {
  const {publishRangeWindow} = this.props.ros
  const namespace = this.props.processNamespace + topicName
  if(event.key === 'Enter'){
    const value = parseFloat(event.target.value)
    if (!isNaN(value)){
      var min = this.state.range_clip_min_m
      var max = this.state.range_clip_max_m
      if (entryName === "min"){
        min = value
      }
      else if (entryName === "max"){
        max = value
      }
      publishRangeWindow(namespace,min,max,false)
    }
    document.getElementById(event.target.id).style.color = Styles.vars.colors.black
  }
}


renderApp() {
  const {sendTriggerMsg, sendStringMsg, sendBoolMsg, ptxUnits} = this.props.ros
  const pantilt_options = this.createPTXOptions()
  const sel_pantilt = this.state.selected_pantilt
  const pantilt_connected = this.state.pantilt_connected
  const sel_pt = this.state.selected_pantilt
  const NoneOption = <Option>None</Option>
  const classifier_running = this.state.classifier_running
  const selectedClass = this.state.selected_class
  const class_sel = selectedClass !== null && selectedClass !== 'None'
  const connected = this.state.connected === true
  const appNamespace = this.getAppNamespace()
  const pan_min = this.state.pan_min ? this.state.pan_min : -180
  const pan_max = this.state.pan_max ? this.state.pan_max : 180
  const tilt_min = this.state.tilt_min ? this.state.tilt_min : -180
  const tilt_max = this.state.tilt_max  ? this.state.tilt_max : 180
  const set_pan_min = this.state.set_pan_min ? this.state.set_pan_min : -180
  const set_pan_max = this.state.set_pan_max ? this.state.set_pan_max : 180
  const set_tilt_min = this.state.set_tilt_min ? this.state.set_tilt_min : -180
  const set_tilt_max = this.state.set_tilt_max ? this.state.set_tilt_max : 180


  return (
    <Section title={"AI PT Tracking App"}>

    <Columns>
      <Column>



      <Columns>
        <Column>

            <div hidden={(connected === true)}>

              <pre style={{ height: "40px", overflowY: "auto" ,fontWeight: 'bold' , color: Styles.vars.colors.Green, textAlign: "left" }}>
                  {"Loading"}
                </pre>

              </div>

              <div hidden={(connected === false)}>

                <Label title="Enable App">
                    <Toggle
                    checked={this.state.app_enabled===true}
                    onClick={() => sendBoolMsg(appNamespace + "/enable_app",!this.state.app_enabled)}>
                    </Toggle>
              </Label>

            </div>

        </Column>
        <Column>

        </Column>
      </Columns>




    <div hidden={(connected !== true || this.state.app_enabled !== true)}>

      <Columns>
        <Column>


          <Label title={"Classifier Running"}>
            <BooleanIndicator value={this.state.classifier_running} />
          </Label>


          <Label title={"Pantilt Connected"}>
            <BooleanIndicator value={pantilt_connected} />
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
     

      <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
        {"Image Settings"}
      </label>

    <Columns>
      <Column>


      <SliderAdjustment
          title={"Image Vertical (Degs)"}
          msgType={"std_msgs/float32"}
          adjustment={this.state.image_fov_vert_degs}
          topic={appNamespace + "/set_image_fov_vert"}
          scaled={1.0}
          min={50}
          max={150}
          tooltip={""}
          unit={""}
      />


      </Column>
      <Column>


      <SliderAdjustment
          title={"Image Horizontal (Degs)"}
          msgType={"std_msgs/float32"}
          adjustment={this.state.image_fov_horz_degs}
          topic={appNamespace + "/set_image_fov_horz"}
          scaled={1.0}
          min={50}
          max={150}
          tooltip={""}
          unit={""}
      />

      </Column>
    </Columns>

    <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
    <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
        {"Target Settings"}
       </label>

    <Columns>
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

      </Column>
      <Column>

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

          <SliderAdjustment
              title={"Set Error Goal (Degs)"}
              msgType={"std_msgs/float32"}
              adjustment={this.state.error_goal_deg}
              topic={appNamespace + "/set_error_goal_deg"}
              scaled={1.0}
              min={this.state.error_goal_min_max_deg[0]}
              max={this.state.error_goal_min_max_deg[1]}
              tooltip={""}
              unit={""}
          />

      </Column>
    </Columns>


    <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
    <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
        {"Pan Tilt Settings"}
       </label>

       <Columns>
          <Column>

              <Label title={"Select PanTilt Device"}>
              <Select
                id="pt_select"
                onChange={(event) => onDropdownSelectedSendStr.bind(this)(event, appNamespace + "/select_pantilt")}
                value={sel_pantilt}
              >
                {(pantilt_options.length > 1)
                  ? pantilt_options
                  : NoneOption}
              </Select>
            </Label>

          </Column>
          <Column>
 
          </Column>
        </Columns>



      <div hidden={(pantilt_connected === false)}>

                <Columns>
                  <Column>

                            <Label title={"Has Position Feedback"}>
                          <BooleanIndicator value={this.state.has_position_feedback} />
                        </Label>


                </Column>
                <Column>

                            <Label title={"Has Adjustable Speed"}>
                          <BooleanIndicator value={this.state.has_adjustable_speed} />
                        </Label>
          
                  </Column>
                </Columns>


                <Label title={"Set Pan Min"}>
                    <Input id="set_pan_min" 
                      value={this.state.pan_min} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"pan_min")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_min_max_pan_angles","min")} />
              </Label>
            

                  <Label title={"Set Pan Max"}>
                    <Input id="set_pan_min" 
                     value={this.state.pan_max} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"pan_max")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_min_max_pan_angles","max")} />                      
                  </Label>  


                  <Label title={"Set Tilt Min"}>
                    <Input id="set_tilt_min" 
                      value={this.state.tilt_min} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"tilt_min")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_min_max_tilt_angles","min")} />
              </Label>
            

                  <Label title={"Set Tilt Max"}>
                    <Input id="set_tilt_min" 
                     value={this.state.tilt_max} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"tilt_max")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_min_max_tilt_angles","max")} />                      
                  </Label>  

              <Columns>
                <Column>
                  <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                        {"Scan Settings"}
                      </label>

                      <div hidden={this.state.has_adjustable_speed === false}>

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

                        </div>

                        <div hidden={this.state.has_position_feedback === false}>

                            <SliderAdjustment
                                title={"Scan Tilt Offset (Degs)"}
                                msgType={"std_msgs/float32"}
                                adjustment={this.state.scan_tilt_offset}
                                topic={appNamespace + "/set_scan_tilt_offset"}
                                scaled={1.0}
                                min={tilt_min}
                                max={tilt_max}
                                tooltip={""}
                                unit={""}
                            />

                        </div>

              </Column>
              <Column>


                          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                              {"Track Settings"}
                            </label>

                          <div hidden={this.state.has_adjustable_speed === false}>

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

                          </div>


                          <div hidden={this.state.has_position_feedback === false}>

                              <SliderAdjustment
                                  title={"Track Tilt Offset (Degs)"}
                                  msgType={"std_msgs/float32"}
                                  adjustment={this.state.track_tilt_offset}
                                  topic={appNamespace + "/set_track_tilt_offset"}
                                  scaled={1.0}
                                  min={tilt_min}
                                  max={tilt_max}
                                  tooltip={""}
                                  unit={""}
                              />
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

      </div>

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
