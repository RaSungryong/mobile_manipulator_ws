#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import yaml

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