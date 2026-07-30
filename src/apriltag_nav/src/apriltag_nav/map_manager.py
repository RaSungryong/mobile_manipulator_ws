#!/usr/bin/env python
# -*- coding: utf-8 -*-
import yaml
import rospy
from collections import deque

class MapManager:
    """
    Handles the Tag Database, Map Topology, and Path Planning.
    Reads configuration from 'config/map.yaml'.
    """
    
    def __init__(self, map_yaml_path):
        self.tags = {}
        self.edges = {}
        self._load_map(map_yaml_path)
        
    def _load_map(self, yaml_path):
        """Parses the YAML file into internal structures."""
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            # yaml.safe_load returns None for empty files; guard explicitly so
            # downstream `.get()` calls don't AttributeError.
            if data is None:
                rospy.logerr(
                    f"[MapManager] Map file is empty or unparsable: {yaml_path}"
                )
                return

            # 1. Load Tags
            # YAML format: {id: {x: ..., y: ...}, ...}
            self.tags = data.get('tags', {}) or {}

            # 2. Load Edges
            # YAML format: [{from: ..., to: ..., direction: ...}, ...]
            raw_edges = data.get('edges', []) or []
            for e in raw_edges:
                u = e['from']
                if u not in self.edges:
                    self.edges[u] = []
                self.edges[u].append(e)

        except Exception as e:
            # Loud failure: without a map, every subsequent GOTO/path call
            # would silently return None and the operator would have no idea
            # why navigation is dead.
            rospy.logerr(f"[MapManager] Error loading map '{yaml_path}': {e}")

    def get_tag_info(self, tag_id):
        """Returns the dictionary info for a specific tag ID."""
        return self.tags.get(tag_id) # Returns None if not found

    def get_tag_type(self, tag_id):
        """Returns the type string (WORK, PIVOT, MOVE, DOCK) or None."""
        info = self.get_tag_info(tag_id)
        return info.get('type') if info else None

    def get_edge(self, from_id, to_id):
        """Finds the edge definition between two tags."""
        if from_id in self.edges:
            for edge in self.edges[from_id]:
                if edge['to'] == to_id:
                    return edge
        return None

    def find_path(self, start_id, goal_id):
        """
        Finds the shortest path of Tag IDs from start to goal using BFS.
        Returns a list: [start_id, next_id, ..., goal_id]
        """
        if start_id == goal_id:
            return [start_id]
        
        if start_id not in self.edges:
            return None
            
        visited = {start_id}
        queue = deque([(start_id, [start_id])])
        
        while queue:
            current, path = queue.popleft()
            
            if current not in self.edges:
                continue
                
            for edge in self.edges[current]:
                neighbor = edge['to']
                if neighbor not in visited:
                    new_path = path + [neighbor]
                    if neighbor == goal_id:
                        return new_path
                    visited.add(neighbor)
                    queue.append((neighbor, new_path))
        
        return None
