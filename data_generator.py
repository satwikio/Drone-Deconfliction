"""
Generator for UAV missions with realistic flight paths.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Tuple, Dict, Optional
import random
import os
from datetime import datetime
import json

from data_structures import Waypoint3D, Mission3D

class DeliveryWaypointGenerator:
    """Generate realistic delivery drone waypoints with smooth paths."""
    
    def __init__(self, 
                area_bounds: Tuple[float, float, float, float] = (0, 1000, 0, 1000),
                altitude_range: Tuple[float, float] = (80, 150),
                urban_centers: Optional[List[Tuple[float, float, float]]] = None):
        """
        Initialize the delivery waypoint generator.
        
        Args:
            area_bounds: (min_x, max_x, min_y, max_y) simulation area boundaries
            altitude_range: (min_z, max_z) altitude range for drones
            urban_centers: List of (x, y, radius) representing urban areas with higher activity
        """
        self.area_bounds = area_bounds
        self.altitude_range = altitude_range
        
        # Default urban centers if none provided (delivery hubs and destinations)
        self.urban_centers = urban_centers or [
            (200, 200, 150),    # Downtown
            (800, 300, 100),    # Industrial zone
            (400, 700, 120),    # Residential area
            (700, 800, 90)      # Suburban area
        ]
        
        # Delivery mission parameters
        self.min_speed = 10
        self.max_speed = 20
        self.min_duration = 15
        self.max_duration = 40
        self.max_turn_angle = 45  # Maximum allowed turn angle in degrees
    
    def generate_smooth_path(self, 
                           start_point: Tuple[float, float], 
                           end_point: Tuple[float, float], 
                           num_waypoints: int = 4) -> List[Tuple[float, float, float]]:
        """
        Generate a smooth path between start and end points with limited turn angles.
        
        Args:
            start_point: (x, y) starting position
            end_point: (x, y) ending position
            num_waypoints: total number of waypoints including start and end
            
        Returns:
            List of (x, y, z) waypoints representing a smooth path
        """
        if num_waypoints < 2:
            num_waypoints = 2
            
        waypoints = []
        
        # Start point
        start_x, start_y = start_point
        end_x, end_y = end_point
        
        # Generate a base altitude for this mission (will vary slightly between waypoints)
        base_altitude = random.uniform(self.altitude_range[0], self.altitude_range[1])
        
        # Add start point
        waypoints.append((start_x, start_y, base_altitude))
        
        if num_waypoints == 2:
            # Only start and end points
            waypoints.append((end_x, end_y, base_altitude))
            return waypoints
        
        # Generate intermediate waypoints for a smooth path
        # We'll use Bezier curve approach for smooth paths with limited angles
        
        # Calculate direct vector from start to end
        direct_vector = np.array([end_x - start_x, end_y - start_y])
        distance = np.linalg.norm(direct_vector)
        
        # Calculate the number of intermediate points
        num_intermediate = num_waypoints - 2
        
        # Create control points for Bezier curve
        # We'll use two control points to create a quadratic Bezier curve
        control_point_1 = np.array([
            start_x + direct_vector[0] * 0.33 + random.uniform(-0.15, 0.15) * distance,
            start_y + direct_vector[1] * 0.33 + random.uniform(-0.15, 0.15) * distance
        ])
        
        control_point_2 = np.array([
            start_x + direct_vector[0] * 0.67 + random.uniform(-0.15, 0.15) * distance,
            start_y + direct_vector[1] * 0.67 + random.uniform(-0.15, 0.15) * distance
        ])
        
        # Generate intermediate points using Bezier curve
        for i in range(num_intermediate):
            t = (i + 1) / (num_intermediate + 1)
            
            # Cubic Bezier formula
            point = (1-t)**3 * np.array([start_x, start_y]) + \
                   3*(1-t)**2*t * control_point_1 + \
                   3*(1-t)*t**2 * control_point_2 + \
                   t**3 * np.array([end_x, end_y])
            
            # Add small altitude variation for realism
            altitude = base_altitude + random.uniform(-5, 5)
            
            waypoints.append((point[0], point[1], altitude))
        
        # Add end point
        waypoints.append((end_x, end_y, base_altitude))
        
        # Verify no sharp turns
        smooth_waypoints = self._ensure_smooth_turns(waypoints)
        
        return smooth_waypoints
    
    def _ensure_smooth_turns(self, waypoints: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        """
        Ensure no sharp turns exist in the path by adjusting waypoints.
        
        Args:
            waypoints: List of (x, y, z) waypoints
            
        Returns:
            Adjusted waypoints with no sharp turns
        """
        if len(waypoints) < 3:
            return waypoints
        
        result = [waypoints[0]]
        
        for i in range(1, len(waypoints) - 1):
            prev = np.array(waypoints[i-1][:2])
            curr = np.array(waypoints[i][:2])
            next_pt = np.array(waypoints[i+1][:2])
            
            # Calculate vectors
            v1 = curr - prev
            v2 = next_pt - curr
            
            # Calculate angle between vectors
            angle = self._angle_between(v1, v2)
            
            # If angle is too sharp, adjust the waypoint
            if abs(angle) > self.max_turn_angle:
                # Calculate a new point that maintains smoother angle
                new_direction = self._rotate_vector(v1, self.max_turn_angle * np.sign(angle))
                new_direction = new_direction / np.linalg.norm(new_direction) * np.linalg.norm(v2)
                
                # Create new point
                new_point = curr + new_direction
                
                # Update waypoint
                result.append((new_point[0], new_point[1], waypoints[i][2]))
            else:
                result.append(waypoints[i])
        
        result.append(waypoints[-1])
        return result
    
    def _angle_between(self, v1: np.ndarray, v2: np.ndarray) -> float:
        """Calculate angle between two vectors in degrees."""
        v1_norm = v1 / np.linalg.norm(v1)
        v2_norm = v2 / np.linalg.norm(v2)
        dot_product = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        angle = np.arccos(dot_product)
        
        # Convert to degrees
        angle_deg = np.degrees(angle)
        
        # Determine sign of angle (clockwise or counter-clockwise)
        cross_product = np.cross(v1_norm, v2_norm)
        if cross_product < 0:
            angle_deg = -angle_deg
            
        return angle_deg
    
    def _rotate_vector(self, vector: np.ndarray, angle_deg: float) -> np.ndarray:
        """Rotate a 2D vector by the specified angle in degrees."""
        angle_rad = np.radians(angle_deg)
        rotation_matrix = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad)],
            [np.sin(angle_rad), np.cos(angle_rad)]
        ])
        
        return np.dot(rotation_matrix, vector)
    
    def generate_urban_point(self) -> Tuple[float, float]:
        """Generate a point near or in urban centers with higher probability."""
        # 80% chance to be near an urban center for delivery missions
        if random.random() < 0.8:
            # Select a random urban center
            center = random.choice(self.urban_centers)
            x_center, y_center, radius = center
            
            # Generate point with normal distribution around center
            distance = random.normalvariate(0, radius/2)
            angle = random.uniform(0, 2 * np.pi)
            
            x = x_center + distance * np.cos(angle)
            y = y_center + distance * np.sin(angle)
            
            # Ensure point is within bounds
            min_x, max_x, min_y, max_y = self.area_bounds
            x = max(min_x, min(max_x, x))
            y = max(min_y, min(max_y, y))
            
            return (x, y)
        else:
            # Generate a point anywhere in the area
            min_x, max_x, min_y, max_y = self.area_bounds
            return (random.uniform(min_x, max_x), 
                    random.uniform(min_y, max_y))
    
    def generate_delivery_mission(self, mission_id: str) -> Mission3D:
        """
        Generate a realistic delivery drone mission with smooth path.
        
        Args:
            mission_id: Identifier for the mission
            
        Returns:
            Mission3D object with mission details
        """
        # Generate start and end points (typically in urban centers)
        start_point = self.generate_urban_point()
        end_point = self.generate_urban_point()
        
        # Ensure start and end are not too close
        while np.linalg.norm(np.array(start_point) - np.array(end_point)) < 100:
            end_point = self.generate_urban_point()
        
        # Determine number of waypoints based on distance
        distance = np.linalg.norm(np.array(start_point) - np.array(end_point))
        base_waypoints = 2  # Start and end
        
        # Add more waypoints for longer distances
        if distance > 300:
            additional_waypoints = random.randint(2, 3)
        else:
            additional_waypoints = random.randint(1, 2)
            
        num_waypoints = base_waypoints + additional_waypoints
        
        # Generate smooth path
        waypoints_xyz = self.generate_smooth_path(start_point, end_point, num_waypoints)
        
        # Generate speed
        speed = random.uniform(self.min_speed, self.max_speed)
        
        # Generate timing
        start_time = random.uniform(0, 60)  # Random start time in the first hour
        
        # Calculate duration based on path length and speed
        total_distance = 0
        for i in range(len(waypoints_xyz) - 1):
            wp1 = np.array(waypoints_xyz[i])
            wp2 = np.array(waypoints_xyz[i+1])
            total_distance += np.linalg.norm(wp2 - wp1)
        
        duration = total_distance / speed
        end_time = start_time + duration
        
        # Create Mission3D object
        waypoints = [Waypoint3D(x=x, y=y, z=z) for x, y, z in waypoints_xyz]
        mission = Mission3D(
            waypoints=waypoints,
            start_time=start_time,
            end_time=end_time,
            uav_id=mission_id,
            speed=speed
        )
        
        return mission
    
    def generate_multiple_missions(self, 
                                 num_primary: int = 1,
                                 num_simulated: int = 10) -> Dict:
        """
        Generate multiple delivery missions for simulation.
        
        Args:
            num_primary: Number of primary missions
            num_simulated: Number of simulated other missions
            
        Returns:
            Dictionary with primary and simulated missions
        """
        primary_missions = []
        for i in range(num_primary):
            mission_id = f"PRIMARY_DRONE_{i+1}"
            primary_missions.append(self.generate_delivery_mission(mission_id))
        
        simulated_missions = []
        for i in range(num_simulated):
            mission_id = f"SIM_DRONE_{i+1}"
            simulated_missions.append(self.generate_delivery_mission(mission_id))
        
        return {
            "primary_missions": primary_missions,
            "simulated_missions": simulated_missions
        }
    
    def save_missions_to_csv(self, 
                          missions: Dict, 
                          output_dir: str = "data",
                          base_filename: str = "delivery_missions") -> Dict[str, str]:
        """
        Save generated missions to CSV files.
        
        Args:
            missions: Dictionary with primary and simulated missions
            output_dir: Directory to save files
            base_filename: Base name for output files
            
        Returns:
            Dictionary with paths to saved files
        """
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Prepare waypoints dataframe
        all_waypoints = []
        
        # Process primary missions
        for mission in missions["primary_missions"]:
            for i, wp in enumerate(mission.waypoints):
                all_waypoints.append({
                    "mission_id": mission.uav_id,
                    "waypoint_id": i,
                    "x": wp.x,
                    "y": wp.y,
                    "z": wp.z,
                    "is_primary": True
                })
        
        # Process simulated missions
        for mission in missions["simulated_missions"]:
            for i, wp in enumerate(mission.waypoints):
                all_waypoints.append({
                    "mission_id": mission.uav_id,
                    "waypoint_id": i,
                    "x": wp.x,
                    "y": wp.y,
                    "z": wp.z,
                    "is_primary": False
                })
        
        # Create dataframes
        waypoints_df = pd.DataFrame(all_waypoints)
        
        # Prepare missions dataframe
        mission_data = []
        for mission in missions["primary_missions"] + missions["simulated_missions"]:
            mission_data.append({
                "mission_id": mission.uav_id,
                "mission_type": "delivery",
                "start_time": mission.start_time,
                "end_time": mission.end_time,
                "speed": mission.speed,
                "is_primary": mission.uav_id.startswith("PRIMARY")
            })
        
        missions_df = pd.DataFrame(mission_data)
        
        # Save to CSV
        waypoints_path = os.path.join(output_dir, f"{base_filename}_waypoints_{timestamp}.csv")
        missions_path = os.path.join(output_dir, f"{base_filename}_details_{timestamp}.csv")
        
        waypoints_df.to_csv(waypoints_path, index=False)
        missions_df.to_csv(missions_path, index=False)
        
        # Also save raw data for reference
        json_data = {
            "primary_missions": [self._mission_to_dict(m) for m in missions["primary_missions"]],
            "simulated_missions": [self._mission_to_dict(m) for m in missions["simulated_missions"]]
        }
        
        json_path = os.path.join(output_dir, f"{base_filename}_raw_{timestamp}.json")
        with open(json_path, 'w') as f:
            json.dump(json_data, f, indent=2)
        
        return {
            "waypoints": waypoints_path,
            "missions": missions_path,
            "json": json_path
        }
    
    def _mission_to_dict(self, mission: Mission3D) -> Dict:
        """Convert Mission3D object to dictionary for JSON serialization."""
        return {
            "mission_id": mission.uav_id,
            "mission_type": "delivery",
            "start_time": mission.start_time,
            "end_time": mission.end_time,
            "speed": mission.speed,
            "waypoints": [{"x": wp.x, "y": wp.y, "z": wp.z} for wp in mission.waypoints]
        }


def load_missions_from_json(json_path: str) -> Dict:
    """
    Load missions from a JSON file.
    
    Args:
        json_path: Path to the JSON file
        
    Returns:
        Dictionary with primary and simulated missions as Mission3D objects
    """
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    primary_missions = []
    for mission_data in data["primary_missions"]:
        waypoints = [
            Waypoint3D(x=wp["x"], y=wp["y"], z=wp["z"]) 
            for wp in mission_data["waypoints"]
        ]
        mission = Mission3D(
            waypoints=waypoints,
            start_time=mission_data["start_time"],
            end_time=mission_data["end_time"],
            uav_id=mission_data["mission_id"],
            speed=mission_data["speed"]
        )
        primary_missions.append(mission)
    
    simulated_missions = []
    for mission_data in data["simulated_missions"]:
        waypoints = [
            Waypoint3D(x=wp["x"], y=wp["y"], z=wp["z"]) 
            for wp in mission_data["waypoints"]
        ]
        mission = Mission3D(
            waypoints=waypoints,
            start_time=mission_data["start_time"],
            end_time=mission_data["end_time"],
            uav_id=mission_data["mission_id"],
            speed=mission_data["speed"]
        )
        simulated_missions.append(mission)
    
    return {
        "primary_missions": primary_missions,
        "simulated_missions": simulated_missions
    }