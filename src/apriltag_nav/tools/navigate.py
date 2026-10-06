#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
AprilTag Navigation - Main Entry Point

This script orchestrates the navigation process:
1. Loads configuration.
2. Interacts with the user to select a mode.
3. Generates the sequence of waypoints (tasks).
4. Commands the robot controller to execute the path.

⚠️ Holds its own MobileController, so it publishes /cmd_vel itself. Since
2026-08-11 mobile_node is the sole /cmd_vel publisher in the running stack, and
there is no arbitration below it — navifra's base_controller obeys whichever
message arrived last. **Never run this while mobile_manipulator.launch is up.**
It is a standalone bring-up tool for a stack that is not running; to drive from
a live stack use `GOTO <tag>` on /task_command, or /mobile/goto_tag directly.
"""

import rospy
import os
import argparse
import yaml
from apriltag_nav.map_manager import MapManager
from apriltag_nav.mobile_controller import MobileController
from apriltag_nav import utils

# navigate.py is the standalone bring-up navigator: navigation only, no
# scanning. It is a SECOND /cmd_vel publisher and must never run while the
# stack (mobile_node) is up — see the module docstring.

# Dock tag. Must match TaskManager.START_TAG and the DOCK entry in map.yaml.
DOCK_TAG = 500

# Hardcoded paths relative to this script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH = os.path.join(PKG_DIR, 'config', 'robot.yaml')
MAP_PATH = os.path.join(PKG_DIR, 'config', 'map.yaml')

def load_config(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def main():
    rospy.init_node('apriltag_navigator', anonymous=True)
    
    # --- 1. Argument Parsing ---
    parser = argparse.ArgumentParser(description="AprilTag Navigation Node")
    parser.add_argument('--mode', type=int, help="Mode 1 (Task), 2 (Manual)")
    parser.add_argument('--task', type=str, help="Task name for Mode 1")
    parser.add_argument('--target', type=int, help="Target Tag ID for Mode 2")
    args = parser.parse_args()

    # --- 2. Initialization ---
    rospy.loginfo("Initializing components...")
    
    # Load Configs
    robot_config = load_config(CONFIG_PATH)
    
    # Initialize Modules
    map_mgr = MapManager(MAP_PATH)
    robot = MobileController(robot_config, map_mgr)
    
    # --- 3. Mode Selection Logic ---
    mode = args.mode
    waypoints = []
    
    # Interactive Menu if no mode arg
    if mode is None:
        print("\n" + "="*40)
        print(" AprilTag Navigation ")
        print("="*40)
        print(" 1. Preset Tasks")
        print(" 2. Manual Navigation")
        print("="*40)
        
        while True:
            try:
                val = input("Select Mode (1-2): ").strip()
                if val in ['1', '2']:
                    mode = int(val)
                    break
            except KeyboardInterrupt:
                return

    # --- 4. Task Generation based on Mode ---
    
    # MODE 1: Preset Tasks
    if mode == 1:
        task_name = args.task
        
        # Interactive Task Selection
        if not task_name:
            tasks = map_mgr.get_all_task_names()
            print("\nAvailable Tasks:")
            for i, t in enumerate(tasks):
                print(f" [{i+1}] {t}")
            
            idx = int(input("Select Task Number: ")) - 1
            if 0 <= idx < len(tasks):
                task_name = tasks[idx]
            else:
                rospy.logerr("Invalid task selection.")
                return

        waypoints = map_mgr.get_preset_task(task_name)
        rospy.loginfo(f"Mode 1: Loaded task '{task_name}' with {len(waypoints)} waypoints.")

    # MODE 2: Manual Navigation
    elif mode == 2:
        target_id = args.target
        
        # Interactive Input
        if target_id is None:
            target_id = utils.get_user_input_tag("\nEnter Destination Tag ID: ")
            if target_id is None: return # User quit
            
        # Get Current Position
        current_id = robot.get_current_tag_id()
        if not current_id:
             current_id = DOCK_TAG # Default dock if lost
             rospy.logwarn(f"Current tag not detected, assuming start at {current_id}")
             
        waypoints = map_mgr.find_path(current_id, target_id)
        if not waypoints:
            rospy.logerr(f"No path found from {current_id} to {target_id}")
            return
            
        rospy.loginfo(f"Mode 2: Path calculated: {waypoints}")

    # --- 5. Execution ---

    rospy.loginfo("Starting Mission...")
    
    # Mode 1 & 2 (no scan)
    if not waypoints:
        rospy.logwarn("No waypoints to execute.")
        return

    current_id = waypoints[0]
    for wp in waypoints[1:]:
        if rospy.is_shutdown(): break
        success = robot.go_to_next_tag(wp, known_start_id=current_id)
        if not success:
            rospy.logerr(f"Failed to reach tag {wp}. Aborting.")
            break
        current_id = wp

    rospy.loginfo("Mission Complete.")


if __name__ == '__main__':
    main()
