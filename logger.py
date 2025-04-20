"""
Logging utilities for UAV Strategic Deconfliction.
"""

import os
import logging
import json
from datetime import datetime
from typing import List, Dict, Any

from data_structures import Conflict3D, Mission3D

def setup_logger(log_dir: str = "logs") -> logging.Logger:
    """
    Set up a logger for the application.
    
    Args:
        log_dir: Directory to store logs
        
    Returns:
        Configured logger
    """
    # Create log directory if it doesn't exist
    os.makedirs(log_dir, exist_ok=True)
    
    # Get current timestamp for log filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f"deconfliction_{timestamp}.log")
    
    # Configure logger
    logger = logging.getLogger("uav_deconfliction")
    logger.setLevel(logging.INFO)
    
    # Create file handler
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.INFO)
    
    # Create console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    
    # Create formatter
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    
    # Add handlers to logger
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger

def log_conflicts(conflicts: List[Conflict3D], output_dir: str = "results") -> str:
    """
    Log detected conflicts to a file.
    
    Args:
        conflicts: List of detected conflicts
        output_dir: Directory to store conflict logs
        
    Returns:
        Path to the saved conflict log file
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Get current timestamp for filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    conflict_file = os.path.join(output_dir, f"conflicts_{timestamp}.json")
    
    # Convert conflicts to dictionaries
    conflict_dicts = []
    for conflict in conflicts:
        conflict_dict = {
            "location": {
                "x": conflict.location.x,
                "y": conflict.location.y,
                "z": conflict.location.z
            },
            "time": conflict.time,
            "uav_id_1": conflict.uav_id_1,
            "uav_id_2": conflict.uav_id_2,
            "distance": conflict.distance
        }
        conflict_dicts.append(conflict_dict)
    
    # Save to file
    with open(conflict_file, 'w') as f:
        json.dump(conflict_dicts, f, indent=2)
    
    return conflict_file

def log_mission_check_results(
    primary_mission: Mission3D,
    status: str,
    conflicts: List[Conflict3D], 
    output_dir: str = "results"
) -> str:
    """
    Log mission check results to a file.
    
    Args:
        primary_mission: The primary mission that was checked
        status: Check status ('clear' or 'conflict detected')
        conflicts: List of detected conflicts
        output_dir: Directory to store result logs
        
    Returns:
        Path to the saved result log file
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Get current timestamp for filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_file = os.path.join(output_dir, f"check_result_{timestamp}.json")
    
    # Prepare result data
    result = {
        "mission_id": primary_mission.uav_id,
        "status": status,
        "check_time": timestamp,
        "mission_details": {
            "start_time": primary_mission.start_time,
            "end_time": primary_mission.end_time,
            "speed": primary_mission.speed,
            "waypoint_count": len(primary_mission.waypoints)
        },
        "conflicts": []
    }
    
    # Add conflict details if any
    if conflicts:
        for conflict in conflicts:
            conflict_dict = {
                "location": {
                    "x": conflict.location.x,
                    "y": conflict.location.y,
                    "z": conflict.location.z
                },
                "time": conflict.time,
                "conflicting_uav": conflict.uav_id_1 if conflict.uav_id_2 == primary_mission.uav_id else conflict.uav_id_2,
                "distance": conflict.distance
            }
            result["conflicts"].append(conflict_dict)
    
    # Save to file
    with open(result_file, 'w') as f:
        json.dump(result, f, indent=2)
    
    return result_file