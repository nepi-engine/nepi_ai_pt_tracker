/*
 #
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#
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

import {createShortUniqueValues, onDropdownSelectedSendStr, createMenuListFromStrList, onUpdateSetStateValue} from "./Utilities"

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
      })
    }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (prevState.appNamespace !== namespace && namespace !== null)
    if (namespace_updated) {
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
  var i
  var items = []
  items.push(<Option value={"None"}>{"None"}</Option>)
  var unique_names = createShortUniqueValues(topics)
  for (i = 0; i < topics.length; i++) {
    items.push(<Option value={topics[i]}>{unique_names[i]}</Option>)
  }
  return items
}



onEnterSendInputBoxRangeWindowValue(event, topicName, entryName, other_val) {
  const {publishRangeWindow} = this.props.ros
  const appNamespace = this.getAppNamespace()
  const namespace = appNamespace + topicName
  var min = -60
  var max = 60
  if(event.key === 'Enter'){
    const value = parseFloat(event.target.value)
    if (!isNaN(value)){
      if (entryName === "min"){
        min = value
        max = other_val
      }
      else if (entryName === "max"){
        min = other_val
        max = value
      }
      publishRangeWindow(namespace,min,max,false)
    }
    document.getElementById(event.target.id).style.color = Styles.vars.colors.black
  }
}


renderApp() {
  const {sendTriggerMsg, sendBoolMsg} = this.props.ros
  const pantilt_options = this.createPTXOptions()
  const sel_pantilt = this.state.selected_pantilt
  const pantilt_connected = this.state.pantilt_connected
  const NoneOption = <Option>None</Option>
  const selectedClass = this.state.selected_class
  const class_sel = selectedClass !== null && selectedClass !== 'None'
  const connected = this.state.connected === true
  const appNamespace = this.getAppNamespace()
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
                  {"Loading or Refresh Page"}
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


          <Label title={"AI Detection Running"}>
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


  
      </Column>
    </Columns>
     

    <Columns>
        <Column>

           <Label title={"Pitch Deg Error"}>
            <Input disabled value={round(this.state.pitch_deg,2)} />
          </Label>

      </Column>
      <Column>

              <Label title={"Yaw Deg Error"}>
            <Input disabled value={round(this.state.yaw_deg,2)} />
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


                <Columns>
                  <Column>

                <Label title={"Set Pan Min"}>
                    <Input id="set_pan_min" 
                      value={this.state.set_pan_min} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"set_pan_min")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_min_max_pan_angles","min",this.state.set_pan_max)} />
              </Label>
            

                  <Label title={"Set Pan Max"}>
                    <Input id="set_pan_max" 
                     value={this.state.set_pan_max} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"set_pan_max")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_min_max_pan_angles","max",this.state.set_pan_min)} />                      
                  </Label>  

                </Column>
                <Column>

                  <Label title={"Set Tilt Min"}>
                    <Input id="set_tilt_min" 
                      value={this.state.set_tilt_min} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"set_tilt_min")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_min_max_tilt_angles","min",this.state.set_tilt_max)} />
              </Label>
            

                  <Label title={"Set Tilt Max"}>
                    <Input id="set_tilt_max" 
                     value={this.state.set_tilt_max} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"set_tilt_max")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_min_max_tilt_angles","max",this.state.set_tilt_min)} />                      
                  </Label>  

                  </Column>
                </Columns>

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
                                min={set_tilt_min}
                                max={set_tilt_max}
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
                                  min={set_tilt_min/2}
                                  max={set_tilt_max/2}
                                  tooltip={""}
                                  unit={""}
                              />
                          </div>

                </Column>
              </Columns>


      </div>

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
    if (this.state.needs_update === true){
      this.setState({needs_update: false})
    }
    const connected = this.state.connected === true
    const appNamespace = (connected) ? this.getAppNamespace() : null
    const imageNamespace = appNamespace + '/' + this.state.image_name

    return (

      <Columns>
      <Column equalWidth={true}>

       
      <div hidden={!connected}>

      <NepiIFSaveData
        saveNamespace={appNamespace}
        title={"Nepi_IF_SaveData"}
      />

      </div>

      <CameraViewer
        imageTopic={imageNamespace}
        title={this.state.image_name}
        hideQualitySelector={false}
      />


      </Column>
      <Column>


      <AiDetectorMgr
              title={"Nepi_Mgr_AI_Detector"}
          />



      {this.renderApp()}

      </Column>
      </Columns>

      )
    }  



}

export default AiPtTrackerApp
