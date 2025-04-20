"""
Conflict detection system for UAV Strategic Deconfliction.
"""

import numpy as np
from rtree import index
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor
import os
import time
import logging
from typing import List, Tuple, Dict, Optional

from data_structures import Waypoint3D, Mission3D, TrajectorySegment, Conflict3D

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("conflict_detector")

def interpolate_position_on_segment(segment: TrajectorySegment, time: float) -> Waypoint3D:
    """
    Interpolate position at given time on a segment.
    
    Args:
        segment: The trajectory segment
        time: The time to interpolate at
        
    Returns:
        Interpolated waypoint at the given time
    """
    # Calculate progress along segment
    total_time = segment.end_time - segment.start_time
    elapsed_time = time - segment.start_time
    progress = elapsed_time / total_time
    
    # Linear interpolation
    start_array = segment.start_pos.to_array()
    end_array = segment.end_pos.to_array()
    interpolated = start_array + progress * (end_array - start_array)
    
    return Waypoint3D(
        x=interpolated[0],
        y=interpolated[1],
        z=interpolated[2],
        time=time
    )

def check_segment_conflicts_worker(args):
    """
    Worker function for parallel conflict checking.
    
    Args:
        args: Tuple containing (segment1, segment2, safety_buffer, time_step)
        
    Returns:
        List of detected conflicts
    """
    segment1, segment2, safety_buffer, time_step = args
    
    # Find temporal overlap
    overlap_start = max(segment1.start_time, segment2.start_time)
    overlap_end = min(segment1.end_time, segment2.end_time)
    
    if overlap_start >= overlap_end:
        return []
    
    # Sample points along the overlapping interval
    time_steps = np.arange(overlap_start, overlap_end, time_step)
    conflicts = []
    
    for t in time_steps:
        pos1 = interpolate_position_on_segment(segment1, t)
        pos2 = interpolate_position_on_segment(segment2, t)
        
        distance = np.linalg.norm(pos1.to_array() - pos2.to_array())
        
        if distance < safety_buffer:
            # Found a conflict
            conflict = Conflict3D(
                location=pos1,
                time=t,
                uav_id_1=segment1.uav_id,
                uav_id_2=segment2.uav_id,
                distance=distance
            )
            conflicts.append(conflict)
    
    return conflicts

class ParallelRTreeConflictDetector:
    """
    Conflict detector using R-tree for spatial indexing and parallel processing.
    """
    
    def __init__(self, safety_buffer: float = 50.0, time_step: float = 1.0, num_workers: int = None):
        """
        Initialize the conflict detector.
        
        Args:
            safety_buffer: Minimum distance (in meters) allowed between drones
            time_step: Time step (in seconds) for conflict checking
            num_workers: Number of parallel workers for conflict detection
        """
        self.safety_buffer = safety_buffer
        self.time_step = time_step
        self.num_workers = num_workers or mp.cpu_count()
        
        # Initialize 4D R-tree
        p = index.Property()
        p.dimension = 4  # x, y, z, time
        p.variant = index.RT_Star  # Better for dynamic updates
        
        # Create temporary file path for R-tree
        self.rtree_path = f"rtree_index_{os.getpid()}"
        self.idx = index.Index(self.rtree_path, properties=p)
        
        # Store segments in a simple dict (no locks needed)
        self.trajectory_segments = {}
        self.segment_counter = 0
        
        logger.info(f"Initialized conflict detector with safety buffer {safety_buffer}m and {self.num_workers} workers")
        
    def add_simulated_mission(self, mission: Mission3D) -> None:
        """
        Add a simulated mission to the R-tree index.
        
        Args:
            mission: The mission to add
        """
        segments = self._create_trajectory_segments(mission)
        
        for segment in segments:
            # Store segment
            self.trajectory_segments[segment.segment_id] = segment
            # Insert into R-tree
            self.idx.insert(segment.segment_id, segment.get_mbr(self.safety_buffer))
    
    def _create_trajectory_segments(self, mission: Mission3D) -> List[TrajectorySegment]:
        """
        Break down mission into trajectory segments.
        
        Args:
            mission: The mission to segment
            
        Returns:
            List of trajectory segments
        """
        segments = []
        waypoints = mission.waypoints
        
        # Calculate time at each waypoint based on distance and speed
        times = [mission.start_time]
        cumulative_time = mission.start_time
        
        for i in range(len(waypoints) - 1):
            distance = np.linalg.norm(waypoints[i].to_array() - waypoints[i+1].to_array())
            travel_time = distance / mission.speed
            cumulative_time += travel_time
            times.append(cumulative_time)
        
        # Create segments
        for i in range(len(waypoints) - 1):
            segment_id = self.segment_counter
            self.segment_counter += 1
            
            segment = TrajectorySegment(
                start_pos=waypoints[i],
                end_pos=waypoints[i+1],
                start_time=times[i],
                end_time=times[i+1],
                uav_id=mission.uav_id,
                segment_id=segment_id
            )
            segments.append(segment)
        
        return segments
    
    def check_mission(self, primary_mission: Mission3D) -> Tuple[str, List[Conflict3D]]:
        """
        Check for conflicts using parallel processing.
        
        Args:
            primary_mission: The primary mission to check
            
        Returns:
            Tuple of (status, conflicts) where status is "clear" or "conflict detected"
        """
        logger.info(f"Checking mission {primary_mission.uav_id} for conflicts")
        primary_segments = self._create_trajectory_segments(primary_mission)
        
        # Collect potential conflicts for each segment using R-tree
        potential_conflicts = []
        for segment in primary_segments:
            hits = list(self.idx.intersection(segment.get_mbr(self.safety_buffer)))
            for hit_id in hits:
                if hit_id in self.trajectory_segments:
                    sim_segment = self.trajectory_segments[hit_id]
                    if segment.uav_id != sim_segment.uav_id:  # Skip self-intersections
                        potential_conflicts.append((segment, sim_segment))
        
        logger.info(f"Found {len(potential_conflicts)} potential conflicts to check")
        
        if not potential_conflicts:
            return "clear", []
        
        # Prepare arguments for parallel processing
        worker_args = [
            (seg1, seg2, self.safety_buffer, self.time_step)
            for seg1, seg2 in potential_conflicts
        ]
        
        # Parallel conflict checking using multiprocessing
        start_time = time.time()
        
        with ProcessPoolExecutor(max_workers=self.num_workers) as executor:
            results = list(executor.map(check_segment_conflicts_worker, worker_args))
        
        # Flatten results
        all_conflicts = []
        for conflict_list in results:
            all_conflicts.extend(conflict_list)
        
        check_time = time.time() - start_time
        logger.info(f"Conflict check completed in {check_time:.2f} seconds, found {len(all_conflicts)} conflicts")
        
        return ("conflict detected" if all_conflicts else "clear", all_conflicts)
    
    def cleanup(self):
        """Clean up resources.
        
        This should be called when done using the detector to remove temporary files.
        """
        # Close R-tree
        self.idx.close()
        
        # Clean up R-tree files
        for ext in ['.dat', '.idx']:
            try:
                os.remove(self.rtree_path + ext)
            except OSError:
                pass
        
        logger.info("Cleaned up R-tree temporary files")