#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

APP_NAME = 'AI_PT_Tracker' # Use in display menus
FILE_TYPE = 'APP'
APP_DICT = dict(
    description = 'Application for tracking a target based on AI detected objects',
    pkg_name = 'nepi_app_ai_pt_tracker',
    group_name = 'AI',
    config_file = 'app_ai_pt_tracker.yaml',
    app_file = 'pt_tracker_app_node.py',
    node_name = 'app_ai_pt_tracker'
)
RUI_DICT = dict(
    rui_menu_name = "AI PanTilt Tracker", # RUI menu name or "None" if no rui support
    rui_files = ['NepiAppPtTracker.js'],
    rui_main_file = "NepiAppPtTracker.js",
    rui_main_class = "AiPtTrackerApp"
)




