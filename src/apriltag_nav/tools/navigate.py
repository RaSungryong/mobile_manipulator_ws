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
from std_msgs.msg import String
from apriltag_nav.map_manager import MapManager
from apriltag_nav.mobile_controller import MobileController
from apriltag_nav import utils
# ----------------------------------------------------------------------
# Scan delegation
# ----------------------------------------------------------------------
# navigate.py is the lower-level navigation-only entry point (Plan B).
# Scanning is owned by task_executor.py (Plan A). When Mode 3 needs to
# trigger a scan at a target tag, we publish a TASK command on
# /task_command so task_executor picks it up. If task_executor is not
# running, the call is a no-op with a warning.
_scan_cmd_pub = None

# Dock tag. Must match TaskManager.START_TAG and the DOCK entry in map.yaml.
DOCK_TAG = 500


def _get_scan_cmd_pub():
    global _scan_cmd_pub
    if _scan_cmd_pub is None:
        _scan_cmd_pub = rospy.Publisher('/task_command', String, queue_size=1)
        # Allow a brief moment for subscriber connection.
        rospy.sleep(0.3)
    return _scan_cmd_pub


def _request_scan_via_task_executor(target_tag_id):
    """Publish a scan request to task_executor for the given tag id.

    Replaces the legacy MobileController.perform_scan_procedure(), which
    no longer exists (the scanning role moved to ArmController via
    task_executor). If no subscriber is connected, logs a warning.
    """
    pub = _get_scan_cmd_pub()
    if pub.get_num_connections() == 0:
        rospy.logwarn(
            f"[navigate] No subscriber on /task_command — scan at tag "
            f"{target_tag_id} skipped. Run task_executor.py in parallel "
            f"to enable scan delegation."
        )
        return
    msg = f"GOTO {target_tag_id}"
    rospy.loginfo(f"[navigate] Delegating scan/visit to task_executor: {msg}")
    pub.publish(String(data=msg))

# Hardcoded paths relative to this script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH = os.path.join(PKG_DIR, 'config', 'robot.yaml')
MAP_PATH = os.path.join(PKG_DIR, 'config', 'map.yaml')
EXCEL_DIR = os.path.join(PKG_DIR, 'data', 'excel')

def load_config(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def main():
    rospy.init_node('apriltag_navigator', anonymous=True)
    
    # --- 1. Argument Parsing ---
    parser = argparse.ArgumentParser(description="AprilTag Navigation Node")
    parser.add_argument('--mode', type=int, help="Mode 1 (Task), 2 (Manual), 3 (Excel)")
    parser.add_argument('--task', type=str, help="Task name for Mode 1")
    parser.add_argument('--target', type=int, help="Target Tag ID for Mode 2")
    parser.add_argument('--excel', type=str, help="Excel filename for Mode 3")
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
    scan_targets = []  # For Mode 3
    
    # Interactive Menu if no mode arg
    if mode is None:
        print("\n" + "="*40)
        print(" AprilTag Navigation ")
        print("="*40)
        print(" 1. Preset Tasks")
        print(" 2. Manual Navigation")
        print(" 3. Excel Navigation")
        print("="*40)
        
        while True:
            try:
                val = input("Select Mode (1-3): ").strip()
                if val in ['1', '2', '3']:
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

    # MODE 3: Excel Navigation
    elif mode == 3:
        filename = args.excel
        
        # Interactive File Selection
        if not filename:
            files = utils.list_excel_files(EXCEL_DIR)
            if not files:
                rospy.logerr("No Excel files found in data/excel/")
                return
                
            print("\nAvailable Excel Files:")
            for i, f in enumerate(files):
                print(f" [{i+1}] {f}")
                
            idx = int(input("Select File Number: ")) - 1
            if 0 <= idx < len(files):
                filename = files[idx]
            else:
                rospy.logerr("Invalid file selection.")
                return
        
        full_path = os.path.join(EXCEL_DIR, filename)
        scan_targets = utils.load_excel_waypoints(full_path)
        rospy.loginfo(f"Mode 3: Loaded {len(scan_targets)} scan targets from {filename}.")

    # --- 5. Execution ---

    rospy.loginfo("Starting Mission...")
    
    if mode == 3:
        # Task-Aware Execution for Excel
        current_id = DOCK_TAG
        for target in scan_targets:
            if rospy.is_shutdown(): break
            
            # Skip if already at target
            if current_id == target:
                rospy.loginfo(f"Already at target {target}. Starting scan directly.")
                _request_scan_via_task_executor(target)
                continue
                
            path = map_mgr.find_path(current_id, target)
            if not path:
                rospy.logerr(f"No path to target {target}. Skipping.")
                continue
            
            # 1. Drive the segment waypoints
            for wp in path[1:]:
                if rospy.is_shutdown(): break
                success = robot.go_to_next_tag(wp, known_start_id=current_id)
                if not success:
                    rospy.logerr(f"Failed to reach waypoint {wp}. Aborting.")
                    return
                current_id = wp
            
            # 2. Arrived at Destination -> Scan
            rospy.loginfo(f"Arrived at Destination Target: {target}. Triggering Scan.")
            _request_scan_via_task_executor(target)
            
        # Return to dock
        rospy.loginfo("Mission tasks complete. Returning to dock...")
        return_path = map_mgr.find_path(current_id, DOCK_TAG)
        if return_path:
            for wp in return_path[1:]:
                if rospy.is_shutdown(): break
                robot.go_to_next_tag(wp, known_start_id=current_id)
                current_id = wp
                
    else:
        # Standard Execution for Mode 1 & 2 (No Scan)
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
