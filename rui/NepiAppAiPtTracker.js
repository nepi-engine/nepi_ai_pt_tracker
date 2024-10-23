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

      classifier_name: "None",
      classifier_state: "Stopped",
      classifier_connected: false,
            
      available_targets_list: [],
      target_class: "",
      target_detected: false,
            
      pantilt_device: 0,
      pantilt_connected: 0,
      has_position_feedback: 0,
      min_area_ratio: 0,
      scan_speed_ratio: 0,
      scan_tilt_offset: 0,
      scan_pan_angle: 0,
      track_speed_ratio: 0,
      track_tilt_offset: 0,
            
      is_running: false,
      is_scanning: false,
      is_tracking: false,

      statusListener: null,
      connected: false,
      needs_update: true

    }

    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getBaseNamespace = this.getBaseNamespace.bind(this)


  }


  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return appNamespace
  }

  getBaseNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    }
    return baseNamespace
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({
      classifier_state: message.classifier_state  ,
      classifier_connected: message.classifier_connected  ,
            
      available_targets_list: message.available_targets_list  ,
      target_class: message.target_class  ,
      target_detected: message.target_detected  ,
            
      pantilt_device: message.pantilt_device  ,
      pantilt_connected: message.pantilt_connected  ,
      has_position_feedback: message.has_position_feedback  ,
      min_area_ratio: message.min_area_ratio  ,
      scan_speed_ratio: message.scan_speed_ratio  ,
      scan_tilt_offset: message.scan_tilt_offset  ,
      scan_pan_angle: message.scan_pan_angle  ,
      track_speed_ratio: message.track_speed_ratio  ,
      track_tilt_offset: message.track_tilt_offset  ,
            
      is_running: message.is_running ,
      is_scanning: message.is_scanning  ,
      is_tracking: message.is_tracking  
    
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
      } 
    }
  }


  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
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
    const {sendTriggerMsg, sendStringMsg, sendBoolMsg} = this.props.ros
    const appNamespace = this.state.appNamespace
    const NoneOption = <Option>None</Option>
    const is_running = this.state.is_running
    const pt_connected = this.state.pt_connected
    const classifier_connected = this.state.classifier_connected
    const can_start = (pt_connected && classifier_connected && is_running === false)

    return (


    <Columns>
    <Column>


        <div hidden={!this.state.connected}>

          <Label title={"Tracker Running"}>
              <BooleanIndicator value={is_running} />
            </Label>

              <div hidden={is_running}>
            <ButtonMenu>
              <Button 
                disabled={is_running}
                onClick={() => this.props.ros.sendTriggerMsg(appNamespace + "/start_tacker")}>{"Start Tracking"}</Button>
            </ButtonMenu>
            </div>

            <div hidden={!is_running}>
            <ButtonMenu>
              <Button onClick={() => this.props.ros.sendTriggerMsg(appNamespace + "/stop_tracker")}>{"Stop Tracking"}</Button>
            </ButtonMenu>
            </div>

            <Label title={"Image Count"}>
            <Input disabled value={this.state.file_count} />
            </Label>

            <Label title={"Current Folder"} >
          </Label>
          <pre style={{ height: "25px", overflowY: "auto" }}>
            {this.state.current_folder}
          </pre>


          <Label title={"Current Folder"} >
          </Label>
          <pre style={{ height: "25px", overflowY: "auto" }}>
            {this.state.current_file}
          </pre>

            <Label title={"Set Image Size"}>
            <Select
              id="select_targset_sizeet"
              onChange={(event) => onDropdownSelectedSendStr.bind(this)(event, appNamespace + "/set_size")}
              value={this.state.set_size}
            >
              {this.state.size_options_list
                ? createMenuListFromStrList(this.state.size_options_list, false, [],[],[])
                : NoneOption}
            </Select>
            </Label>


            <Label title={"Set Image Encoding"}>
            <Select
              id="set_encoding"
              onChange={(event) => onDropdownSelectedSendStr.bind(this)(event, appNamespace + "/set_encoding")}
              value={this.state.set_encoding}
            >
              {this.state.encoding_options_list
                ? createMenuListFromStrList(this.state.encoding_options_list, false, [],[],[])
                : NoneOption}
            </Select>
            </Label>

            <Label title="Set Random Order">
              <Toggle
              checked={this.state.set_random===true}
              onClick={() => sendBoolMsg(appNamespace + "/set_random",!this.state.set_random)}>
              </Toggle>
        </Label>

        <Label title="Set Overlay">
              <Toggle
              checked={this.state.set_overlay===true}
              onClick={() => sendBoolMsg(appNamespace + "/set_overlay",!this.state.set_overlay)}>
              </Toggle>
        </Label>

        <Label title={"Set Delay (Seconds)"}>
          <Input id="set_delay" 
            value={this.state.set_delay} 
            onChange={(event) => onUpdateSetStateValue.bind(this)(event,"set_delay")} 
            onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,appNamespace + "/set_delay")} />
        </Label>

        <Label title="Pause">
              <Toggle
              checked={this.state.paused===true}
              onClick={() => sendBoolMsg(appNamespace + "/pause_pub",!this.state.paused)}>
              </Toggle>
        </Label>

        <div hidden={this.state.paused === false}>
            <ButtonMenu>
              <Button onClick={() => this.props.ros.sendTriggerMsg(appNamespace + "/step_backward")}>{"Back"}</Button>
            </ButtonMenu>

            <ButtonMenu>
              <Button onClick={() => this.props.ros.sendTriggerMsg(appNamespace + "/step_forward")}>{"Forward"}</Button>
            </ButtonMenu>

            </div>
        </div>

        </Column>
        </Columns>


    )
  }




  // Function for creating image topic options.
  createFolderOptions() {
    const cur_folder = this.state.current_folder
    const sel_folder = this.state.selected_folder
    var items = []
    if (cur_folder){
      items.push(<Option value={"Home"}>{"Home"}</Option>) 
      if (sel_folder !== 'Home'){
        items.push(<Option value={"Back"}>{"Back"}</Option>) 
      }
      const folders = this.state.current_folders
      for (var i = 0; i < folders.length; i++) {
        items.push(<Option value={folders[i]}>{folders[i]}</Option>)
      }
    }
    return items
  }

  onChangeFolderSelection(event) {
    const {sendTriggerMsg, sendStringMsg} = this.props.ros
    const namespace = this.state.appNamespace
    const setNamespace = namespace + "/select_folder"
    const homeNamespace = namespace + "/home_folder"
    const backNamespace = namespace + "/back_folder"
    const home_folder = this.state.home_folder
    const value = event.target.value
    if (namespace !== null){    
      var selector_idx = 0
      if (value === 'Home') {
        sendTriggerMsg(homeNamespace)
      }
      else if (value === 'Back') {
        sendTriggerMsg(backNamespace)
      }
      else {
        sendStringMsg(setNamespace,value)
      }
    }
    this.setState({selected_folder: value})
  }



  toggleViewableFolders() {
    const viewable = !this.state.viewableFolders
    this.setState({viewableFolders: viewable})
  }


 renderApp() {
    const {sendTriggerMsg, sendStringMsg} = this.props.ros
    const appNamespace = this.state.appNamespace
    const folderOptions = this.createFolderOptions()
    const pubRunning = this.state.pub_running
    const appImageTopic = pubRunning === true ? this.state.appNamespace + "/images" : null
    const viewableFolders = (this.state.viewableFolders || pubRunning === false)
    const NoneOption = <Option>None</Option>
    return (

      <Columns>
      <Column>

        <Columns>
        <Column>

        <Label title="Select Target"> </Label>

        <Select
          id="select_target"
          onChange={(event) => onDropdownSelectedSendStr.bind(this)(event, appNamespace + "/select_target")}
          value={this.state.selected_target}
        >
          {this.state.available_targets_list
            ? createMenuListFromStrList(this.state.available_targets_list, false, [],[],[])
            : NoneOption}
        </Select>

    

        </Column>
        <Column>

        {this.renderPubControls()}

        </Column>
        </Columns>




        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        

        <div hidden={!this.state.connected}>

        <Columns>
        <Column>

          <ButtonMenu>
          <Button onClick={() => sendTriggerMsg( appNamespace + "/reset_app")}>{"Reset App"}</Button>
          </ButtonMenu>

        </Column>
        <Column>

          <ButtonMenu>
          <Button onClick={() => sendTriggerMsg(appNamespace + "/reset_config")}>{"Reset Config"}</Button>
          </ButtonMenu>

        </Column>
        <Column>

          <ButtonMenu>
          <Button onClick={() => sendTriggerMsg(appNamespace + "/save_config")}>{"Save Config"}</Button>
          </ButtonMenu>

        </Column>
        </Columns>
      
       </div>

    </Column>
    </Columns>



    )
  }



  renderImageViewer(){
    const connected = this.state.connected
    const baseNamespace = this.getBaseNamespace()
    const imageNamespace = (connected) ? baseNamespace + "/" + this.state.imageTopic : null 
    return (

      <CameraViewer
        imageTopic={imageNamespace}
        title={this.state.imageText}
        hideQualitySelector={false}
      />

      )
    }  

  render() {
    const connected = this.state.connected
    const namespace = this.getAppNamespace()
    const appNamespace = (connected) ? namespace: null
    return (

      <Columns>
      <Column equalWidth={true}>

      <div hidden={connected}>


      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Connecting"}
         </label>
      
      </div>


      <div hidden={!connected}>

      {this.renderImageViewer()}

      </div>
      </Column>
      <Column>

      <div hidden={!connected}>
      <AiDetectorMgr
              title={"Nepi_Mgr_AI_Detector"}
          />

      {this.renderApp()}

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
