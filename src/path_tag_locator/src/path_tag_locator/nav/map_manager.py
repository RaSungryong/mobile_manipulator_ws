"""
map_manager.py
==============
Map / topology loader + BFS path planner. Copied from
apriltag_nav/scripts/map_manager.py. Schema of map.yaml:

    tags:
      <id>: {x: float, y: float, type: str, zone: str, name: str (opt)}
    edges:
      - {from: <id>, to: <id>, direction: str, type: "move"|"pivot"}
"""
from collections import deque

import rospy
import yaml


class MapManager:
    """Handles the tag database, map topology, and path planning."""

    def __init__(self, map_yaml_path):
        self.tags = {}
        self.edges = {}
        self._load_map(map_yaml_path)

    def _load_map(self, yaml_path):
        try:
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)
            if data is None:
                rospy.logerr(
                    f"[MapManager] Map file is empty or unparsable: {yaml_path}"
                )
                return
            self.tags = data.get("tags", {}) or {}
            raw_edges = data.get("edges", []) or []
            for e in raw_edges:
                u = e["from"]
                self.edges.setdefault(u, []).append(e)
        except Exception as e:
            rospy.logerr(f"[MapManager] Error loading map '{yaml_path}': {e}")

    def get_tag_info(self, tag_id):
        return self.tags.get(tag_id)

    def get_tag_type(self, tag_id):
        info = self.get_tag_info(tag_id)
        return info.get("type") if info else None

    def get_edge(self, from_id, to_id):
        if from_id in self.edges:
            for edge in self.edges[from_id]:
                if edge["to"] == to_id:
                    return edge
        return None

    def find_path(self, start_id, goal_id):
        """BFS over directed edges. Returns [start_id, ..., goal_id] or None."""
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
                neighbor = edge["to"]
                if neighbor in visited:
                    continue
                new_path = path + [neighbor]
                if neighbor == goal_id:
                    return new_path
                visited.add(neighbor)
                queue.append((neighbor, new_path))
        return None
