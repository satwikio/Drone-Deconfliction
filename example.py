"""
Example usage of the UAV Strategic Deconfliction System.
"""

import os
from data_structures import Mission3D, Waypoint3D
from data_generator import DeliveryWaypointGenerator
from conflict_detector import ParallelRTreeConflictDetector
from visualization import create_overview_visualization, create_conflict_visualization
from logger import setup_logger, log_conflicts, log_mission_check_results

def run_example():
    """Run an example of the UAV Strategic Deconfliction System."""
    # Create output directories
    output_dir = "example_output"
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(os.path.join(output_dir, "plots"), exist_ok=True)
    
    # Set up logger
    logger = setup_logger(log_dir=os.path.join(output_dir, "logs"))
    logger.info("Starting UAV Strategic Deconfliction System example")
    
    # Generate missions
    logger.info("Generating drone missions")
    generator = DeliveryWaypointGenerator()
    missions = generator.generate_multiple_missions(num_primary=1, num_simulated=20)
    
    # Save missions
    file_paths = generator.save_missions_to_csv(missions, output_dir=output_dir)
    logger.info(f"Missions saved to: {file_paths}")
    
    # Create overview visualization
    overview_path = os.path.join(output_dir, "plots", "missions_overview.png")
    create_overview_visualization(
        missions,
        urban_centers=generator.urban_centers,
        area_bounds=generator.area_bounds,
        show_plot=True,
        save_path=overview_path
    )
    logger.info(f"Overview visualization saved to: {overview_path}")
    
    # Create conflict detector
    detector = ParallelRTreeConflictDetector(safety_buffer=30.0, time_step=0.5)
    
    # Add simulated missions to detector
    for mission in missions["simulated_missions"]:
        detector.add_simulated_mission(mission)
    
    # Check for conflicts
    primary_mission = missions["primary_missions"][0]
    logger.info(f"Checking conflicts for mission {primary_mission.uav_id}")
    status, conflicts = detector.check_mission(primary_mission)
    
    # Log results
    logger.info(f"Status: {status}")
    logger.info(f"Number of conflicts: {len(conflicts)}")
    
    if conflicts:
        # Log conflict details
        for i, conflict in enumerate(conflicts):
            logger.info(f"Conflict {i+1}:")
            logger.info(f"  Time: {conflict.time:.2f}s")
            logger.info(f"  Location: ({conflict.location.x:.1f}, {conflict.location.y:.1f}, {conflict.location.z:.1f})")
            logger.info(f"  Between: {conflict.uav_id_1} and {conflict.uav_id_2}")
            logger.info(f"  Distance: {conflict.distance:.1f}m")
        
        # Save conflicts to file
        conflict_file = log_conflicts(conflicts, output_dir=output_dir)
        logger.info(f"Conflicts logged to: {conflict_file}")
    
    # Save check results
    result_file = log_mission_check_results(primary_mission, status, conflicts, output_dir=output_dir)
    logger.info(f"Check results logged to: {result_file}")
    
    # Create conflict visualization
    if conflicts:
        conflict_path = os.path.join(output_dir, "plots", "conflicts.png")
        create_conflict_visualization(
            primary_mission=primary_mission,
            simulated_missions=missions["simulated_missions"],
            conflicts=conflicts,
            output_file=conflict_path,
            show_plot=True
        )
        logger.info(f"Conflict visualization saved to: {conflict_path}")
    
    # Clean up detector
    detector.cleanup()
    
    logger.info("Example completed successfully")

if __name__ == "__main__":
    run_example()