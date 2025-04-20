"""
Tests for the UAV Strategic Deconfliction System.
"""

import unittest
import tempfile
import shutil
import os
import numpy as np
from typing import List, Tuple

from data_structures import Waypoint3D, Mission3D
from conflict_detector import ParallelRTreeConflictDetector

class TestUAVDeconfliction(unittest.TestCase):
    """Test cases for UAV Strategic Deconfliction."""
    
    def setUp(self):
        """Set up test environment."""
        # Create temporary directory for test outputs
        self.test_dir = tempfile.mkdtemp()
        
        # Create detector with small safety buffer for testing
        self.detector = ParallelRTreeConflictDetector(safety_buffer=10.0, time_step=0.5)
    
    def tearDown(self):
        """Clean up after tests."""
        # Clean up detector resources
        self.detector.cleanup()
        
        # Remove temporary directory
        shutil.rmtree(self.test_dir)
    
    def test_no_conflict(self):
        """Test scenario with no conflicts."""
        # Create simulated mission
        sim_waypoints = [
            Waypoint3D(x=0, y=0, z=100),
            Waypoint3D(x=100, y=0, z=100)
        ]
        sim_mission = Mission3D(
            waypoints=sim_waypoints,
            start_time=0,
            end_time=10,
            uav_id="SIM_1",
            speed=10.0
        )
        
        # Add to detector
        self.detector.add_simulated_mission(sim_mission)
        
        # Create primary mission that doesn't conflict (different altitude)
        primary_waypoints = [
            Waypoint3D(x=0, y=0, z=150),
            Waypoint3D(x=100, y=0, z=150)
        ]
        primary_mission = Mission3D(
            waypoints=primary_waypoints,
            start_time=0,
            end_time=10,
            uav_id="PRIMARY",
            speed=10.0
        )
        
        # Check for conflicts
        status, conflicts = self.detector.check_mission(primary_mission)
        
        # Verify no conflicts detected
        self.assertEqual(status, "clear")
        self.assertEqual(len(conflicts), 0)
    
    def test_spatial_conflict(self):
        """Test scenario with spatial conflict."""
        # Create simulated mission
        sim_waypoints = [
            Waypoint3D(x=0, y=0, z=100),
            Waypoint3D(x=100, y=0, z=100)
        ]
        sim_mission = Mission3D(
            waypoints=sim_waypoints,
            start_time=0,
            end_time=10,
            uav_id="SIM_1",
            speed=10.0
        )
        
        # Add to detector
        self.detector.add_simulated_mission(sim_mission)
        
        # Create primary mission that conflicts spatially (same path)
        primary_waypoints = [
            Waypoint3D(x=0, y=0, z=100),
            Waypoint3D(x=100, y=0, z=100)
        ]
        primary_mission = Mission3D(
            waypoints=primary_waypoints,
            start_time=0,
            end_time=10,
            uav_id="PRIMARY",
            speed=10.0
        )
        
        # Check for conflicts
        status, conflicts = self.detector.check_mission(primary_mission)
        
        # Verify conflicts detected
        self.assertEqual(status, "conflict detected")
        self.assertGreater(len(conflicts), 0)
    
    def test_temporal_conflict(self):
        """Test scenario with temporal conflict."""
        # Create simulated mission
        sim_waypoints = [
            Waypoint3D(x=0, y=0, z=100),
            Waypoint3D(x=100, y=0, z=100)
        ]
        sim_mission = Mission3D(
            waypoints=sim_waypoints,
            start_time=0,
            end_time=10,
            uav_id="SIM_1",
            speed=10.0
        )
        
        # Add to detector
        self.detector.add_simulated_mission(sim_mission)
        
        # Create primary mission that conflicts temporally (same space, different time)
        primary_waypoints = [
            Waypoint3D(x=0, y=0, z=100),
            Waypoint3D(x=100, y=0, z=100)
        ]
        primary_mission = Mission3D(
            waypoints=primary_waypoints,
            start_time=20,  # After the simulated mission ends
            end_time=30,
            uav_id="PRIMARY",
            speed=10.0
        )
        
        # Check for conflicts
        status, conflicts = self.detector.check_mission(primary_mission)
        
        # Verify no conflicts detected due to temporal separation
        self.assertEqual(status, "clear")
        self.assertEqual(len(conflicts), 0)
    
    def test_crossing_paths(self):
        """Test scenario with crossing paths."""
        # Create simulated mission (diagonal path)
        sim_waypoints = [
            Waypoint3D(x=0, y=0, z=100),
            Waypoint3D(x=100, y=100, z=100)
        ]
        sim_mission = Mission3D(
            waypoints=sim_waypoints,
            start_time=0,
            end_time=14.14,  # √2 * 10 seconds for diagonal
            uav_id="SIM_1",
            speed=10.0
        )
        
        # Add to detector
        self.detector.add_simulated_mission(sim_mission)
        
        # Create primary mission (crossing path)
        primary_waypoints = [
            Waypoint3D(x=0, y=100, z=100),
            Waypoint3D(x=100, y=0, z=100)
        ]
        primary_mission = Mission3D(
            waypoints=primary_waypoints,
            start_time=0,
            end_time=14.14,  # √2 * 10 seconds for diagonal
            uav_id="PRIMARY",
            speed=10.0
        )
        
        # Check for conflicts
        status, conflicts = self.detector.check_mission(primary_mission)
        
        # Verify conflicts detected
        self.assertEqual(status, "conflict detected")
        self.assertGreater(len(conflicts), 0)
        
        # Verify conflict is near the crossing point
        if conflicts:
            conflict = conflicts[0]
            # Should be near (50, 50, 100)
            self.assertAlmostEqual(conflict.location.x, 50, delta=20)
            self.assertAlmostEqual(conflict.location.y, 50, delta=20)
            self.assertAlmostEqual(conflict.location.z, 100, delta=5)
    
    def test_multiple_simulated_drones(self):
        """Test scenario with multiple simulated drones."""
        # Create several simulated missions
        for i in range(5):
            sim_waypoints = [
                Waypoint3D(x=i*50, y=0, z=100),
                Waypoint3D(x=i*50, y=100, z=100)
            ]
            sim_mission = Mission3D(
                waypoints=sim_waypoints,
                start_time=0,
                end_time=10,
                uav_id=f"SIM_{i+1}",
                speed=10.0
            )
            self.detector.add_simulated_mission(sim_mission)
        
        # Create primary mission that crosses all simulated paths
        primary_waypoints = [
            Waypoint3D(x=0, y=50, z=100),
            Waypoint3D(x=250, y=50, z=100)
        ]
        primary_mission = Mission3D(
            waypoints=primary_waypoints,
            start_time=0,
            end_time=25,
            uav_id="PRIMARY",
            speed=10.0
        )
        
        # Check for conflicts
        status, conflicts = self.detector.check_mission(primary_mission)
        
        # Verify conflicts detected
        self.assertEqual(status, "conflict detected")
        self.assertGreater(len(conflicts), 0)
        
        # Check we have conflicts with multiple drones
        conflict_drones = set()
        for conflict in conflicts:
            if conflict.uav_id_1 != "PRIMARY":
                conflict_drones.add(conflict.uav_id_1)
            else:
                conflict_drones.add(conflict.uav_id_2)
        
        # Should have conflicts with multiple drones
        self.assertGreater(len(conflict_drones), 1)
    
    def test_edge_case_safety_buffer(self):
        """Test edge case with drones just outside safety buffer."""
        # Create detector with precise safety buffer
        detector = ParallelRTreeConflictDetector(safety_buffer=10.0, time_step=0.1)
        
        # Create simulated mission
        sim_waypoints = [
            Waypoint3D(x=0, y=0, z=100),
            Waypoint3D(x=100, y=0, z=100)
        ]
        sim_mission = Mission3D(
            waypoints=sim_waypoints,
            start_time=0,
            end_time=10,
            uav_id="SIM_1",
            speed=10.0
        )
        
        # Add to detector
        detector.add_simulated_mission(sim_mission)
        
        # Create primary mission that runs parallel at exactly the safety buffer distance
        primary_waypoints = [
            Waypoint3D(x=0, y=10.1, z=100),  # Just outside buffer
            Waypoint3D(x=100, y=10.1, z=100)
        ]
        primary_mission = Mission3D(
            waypoints=primary_waypoints,
            start_time=0,
            end_time=10,
            uav_id="PRIMARY",
            speed=10.0
        )
        
        # Check for conflicts
        status, conflicts = detector.check_mission(primary_mission)
        
        # Verify no conflicts detected (just outside buffer)
        self.assertEqual(status, "clear")
        self.assertEqual(len(conflicts), 0)
        
        # Now try with mission just inside buffer
        primary_waypoints = [
            Waypoint3D(x=0, y=9.9, z=100),  # Just inside buffer
            Waypoint3D(x=100, y=9.9, z=100)
        ]
        primary_mission = Mission3D(
            waypoints=primary_waypoints,
            start_time=0,
            end_time=10,
            uav_id="PRIMARY",
            speed=10.0
        )
        
        # Check for conflicts
        status, conflicts = detector.check_mission(primary_mission)
        
        # Verify conflicts detected (just inside buffer)
        self.assertEqual(status, "conflict detected")
        self.assertGreater(len(conflicts), 0)
        
        # Clean up
        detector.cleanup()

if __name__ == "__main__":
    unittest.main()