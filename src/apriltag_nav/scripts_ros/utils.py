#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rospy
import pandas as pd
import yaml

def get_package_path():
    """Returns the absolute path to the package root."""
    # Assuming this script is in <pkg>/scripts/
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(script_dir)

def list_excel_files(directory):
    """
    Lists all .xlsx files in the given directory.
    Returns a list of filenames.
    """
    if not os.path.exists(directory):
        rospy.logwarn(f"Directory not found: {directory}")
        return []
    
    files = [f for f in os.listdir(directory) if f.endswith('.xlsx') or f.endswith('.xls')]
    return sorted(files)

def load_excel_waypoints(file_path):
    """
    Reads 'group_id' column from Excel and converts to tag IDs.
    Returns a list of integer tag IDs (100 + group_id).
    """
    try:
        df = pd.read_excel(file_path, usecols=['group_id'])
        
        # Extract group_ids in order, filtering consecutive duplicates
        # e.g., [4, 4, 5, 5] -> [4, 5]
        group_order = []
        prev_gid = None
        for gid in df['group_id']:
            if gid != prev_gid:
                group_order.append(gid)
                prev_gid = gid
        
        # Convert to tag IDs (group_id 0 -> tag 100)
        # Assuming mapping is Tag ID = 100 + Group ID
        scan_tags = [100 + int(gid) for gid in group_order]
        return scan_tags
        
    except Exception as e:
        rospy.logerr(f"Failed to read Excel file: {e}")
        return []

def get_user_input_tag(prompt="Enter tag ID: "):
    """Safely gets an integer tag ID from user input."""
    while not rospy.is_shutdown():
        try:
            val = input(prompt).strip()
            if val.lower() == 'q':
                return None
            return int(val)
        except ValueError:
            print("Invalid input. Please enter a number or 'q' to quit.")

def load_config(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)