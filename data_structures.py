"""
Data structures for UAV Strategic Deconfliction System.
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict

@dataclass
class Waypoint3D:
    """
    Represents a 3D waypoint with optional time information.
    """
    x: float
    y: float
    z: float
    time: Optional[float] = None
    
    def to_array(self) -> np.ndarray:
        """Convert waypoint to numpy array."""
        return np.array([self.x, self.y, self.z])

@dataclass
class Mission3D:
    """
    Represents a complete UAV mission with waypoints and timing information.
    """
    waypoints: List[Waypoint3D]
    start_time: float
    end_time: float
    uav_id: str
    speed: float = 10.0

@dataclass
class TrajectorySegment:
    """
    Represents a segment between two waypoints with timing information.
    """
    start_pos: Waypoint3D
    end_pos: Waypoint3D
    start_time: float
    end_time: float
    uav_id: str
    segment_id: int
    
    def get_mbr(self, safety_buffer: float) -> Tuple:
        """Get minimum bounding rectangle in 4D (x, y, z, time)."""
        min_x = min(self.start_pos.x, self.end_pos.x) - safety_buffer
        max_x = max(self.start_pos.x, self.end_pos.x) + safety_buffer
        min_y = min(self.start_pos.y, self.end_pos.y) - safety_buffer
        max_y = max(self.start_pos.y, self.end_pos.y) + safety_buffer
        min_z = min(self.start_pos.z, self.end_pos.z) - safety_buffer
        max_z = max(self.start_pos.z, self.end_pos.z) + safety_buffer
        
        return (min_x, min_y, min_z, self.start_time,
                max_x, max_y, max_z, self.end_time)

@dataclass
class Conflict3D:
    """
    Represents a conflict between two UAVs in space and time.
    """
    location: Waypoint3D
    time: float
    uav_id_1: str
    uav_id_2: str
    distance: float