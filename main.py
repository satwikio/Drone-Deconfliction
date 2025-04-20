"""
Main entry point for UAV Strategic Deconfliction System.
"""

import os
import argparse
import time
from typing import Dict, List, Tuple, Optional
from datetime import datetime

from data_structures import Mission3D, Waypoint3D
from data_generator import DeliveryWaypointGenerator, load_missions_from_json
from conflict_detector import ParallelRTreeConflictDetector
from visualization import create_overview_visualization, create_conflict_visualization
from logger import setup_logger, log_conflicts, log_mission_check_results

def generate_data(args):
    """
    Generate UAV mission data.
    
    Args:
        args: Command line arguments
    
    Returns:
        Dictionary with generated missions
    """
    logger.info(f"Generating {args.primary} primary and {args.simulated} simulated missions")
    
    generator = DeliveryWaypointGenerator()
    
    missions = generator.generate_multiple_missions(
        num_primary=args.primary,
        num_simulated=args.simulated
    )
    
    if args.save_data:
        # Save to CSV and JSON
        file_paths = generator.save_missions_to_csv(
            missions,
            output_dir=args.output_dir
        )
        
        logger.info(f"Generated files saved to: {file_paths}")
    
    if args.visualize_overview:
        # Create plots directory if it doesn't exist
        plots_dir = os.path.join(args.output_dir, 'plots')
        os.makedirs(plots_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plot_path = os.path.join(plots_dir, f'missions_overview_{timestamp}.png')
        
        create_overview_visualization(
            missions,
            urban_centers=generator.urban_centers,
            area_bounds=generator.area_bounds,
            show_plot=not args.no_display,
            save_path=plot_path
        )
        
        logger.info(f"Overview visualization saved to: {plot_path}")
    
    return missions

def check_conflicts(args, primary_mission: Mission3D, simulated_missions: List[Mission3D]):
    """
    Check for conflicts between a primary mission and simulated missions.
    
    Args:
        args: Command line arguments
        primary_mission: The primary mission to check
        simulated_missions: List of simulated missions to check against
        
    Returns:
        Tuple of (status, conflicts)
    """
    logger.info(f"Checking conflicts for mission {primary_mission.uav_id}")
    
    # Create conflict detector
    detector = ParallelRTreeConflictDetector(
        safety_buffer=args.safety_buffer,
        time_step=args.time_step,
        num_workers=args.workers
    )
    
    # Add all simulated missions to the detector
    for mission in simulated_missions:
        detector.add_simulated_mission(mission)
    
    # Check for conflicts
    start_time = time.time()
    status, conflicts = detector.check_mission(primary_mission)
    check_time = time.time() - start_time
    
    logger.info(f"Conflict check completed in {check_time:.2f} seconds")
    logger.info(f"Status: {status}")
    logger.info(f"Number of conflicts: {len(conflicts)}")
    
    # Log detailed conflict information
    if conflicts:
        logger.info("First 5 conflicts:")
        for i, conflict in enumerate(conflicts[:5]):
            logger.info(f"Conflict {i+1}:")
            logger.info(f"  Time: {conflict.time:.2f}s")
            logger.info(f"  Location: ({conflict.location.x:.1f}, {conflict.location.y:.1f}, {conflict.location.z:.1f})")
            logger.info(f"  Between: {conflict.uav_id_1} and {conflict.uav_id_2}")
            logger.info(f"  Distance: {conflict.distance:.1f}m")
    
    # Save conflicts to file
    if conflicts and args.save_conflicts:
        conflict_file = log_conflicts(conflicts, output_dir=args.output_dir)
        logger.info(f"Conflicts logged to: {conflict_file}")
    
    # Log check results
    if args.save_results:
        result_file = log_mission_check_results(
            primary_mission, 
            status, 
            conflicts, 
            output_dir=args.output_dir
        )
        logger.info(f"Check results logged to: {result_file}")
    
    # Clean up detector
    detector.cleanup()
    
    return status, conflicts

def visualize_conflicts(args, primary_mission: Mission3D, simulated_missions: List[Mission3D], conflicts: List):
    """
    Create visualization of conflicts.
    
    Args:
        args: Command line arguments
        primary_mission: The primary mission
        simulated_missions: List of simulated missions
        conflicts: List of detected conflicts
    """
    if not args.visualize_conflicts:
        return
    
    # Create plots directory if it doesn't exist
    plots_dir = os.path.join(args.output_dir, 'plots')
    os.makedirs(plots_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_path = os.path.join(plots_dir, f'conflicts_{timestamp}.png')
    
    # Create conflict visualization
    output_file = create_conflict_visualization(
        primary_mission=primary_mission,
        simulated_missions=simulated_missions,
        conflicts=conflicts,
        output_file=plot_path,
        show_plot=not args.no_display
    )
    
    logger.info(f"Conflict visualization saved to: {output_file}")

def main():
    """Main entry point for the application."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='UAV Strategic Deconfliction System')
    
    # General arguments
    parser.add_argument('--output-dir', type=str, default='output',
                        help='Directory to save output files')
    parser.add_argument('--no-display', action='store_true',
                        help='Do not display visualizations (save only)')
    
    # Data generation arguments
    parser.add_argument('--generate', action='store_true',
                        help='Generate new mission data')
    parser.add_argument('--primary', type=int, default=1,
                        help='Number of primary missions to generate')
    parser.add_argument('--simulated', type=int, default=10,
                        help='Number of simulated missions to generate')
    parser.add_argument('--save-data', action='store_true',
                        help='Save generated mission data')
    
    # Conflict detection arguments
    parser.add_argument('--load-json', type=str, default=None,
                        help='Load missions from JSON file instead of generating')
    parser.add_argument('--safety-buffer', type=float, default=50.0,
                        help='Safety buffer distance (in meters)')
    parser.add_argument('--time-step', type=float, default=1.0,
                        help='Time step for conflict detection (in seconds)')
    parser.add_argument('--workers', type=int, default=None,
                        help='Number of parallel workers (default: CPU count)')
    parser.add_argument('--save-conflicts', action='store_true',
                        help='Save detected conflicts to file')
    parser.add_argument('--save-results', action='store_true',
                        help='Save mission check results to file')
    
    # Visualization arguments
    parser.add_argument('--visualize-overview', action='store_true',
                        help='Create overview visualization of all missions')
    parser.add_argument('--visualize-conflicts', action='store_true',
                        help='Create visualization of detected conflicts')
    
    args = parser.parse_args()
    
    # Create output directory if it doesn't exist
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Set up global logger
    global logger
    logger = setup_logger(log_dir=os.path.join(args.output_dir, 'logs'))
    
    # Load or generate missions
    if args.load_json:
        logger.info(f"Loading missions from {args.load_json}")
        missions = load_missions_from_json(args.load_json)
    elif args.generate:
        missions = generate_data(args)
    else:
        logger.error("No mission data provided. Use --generate or --load-json.")
        return
    
    # Process each primary mission
    for primary_mission in missions["primary_missions"]:
        logger.info(f"Processing primary mission {primary_mission.uav_id}")
        
        # Check for conflicts
        status, conflicts = check_conflicts(args, primary_mission, missions["simulated_missions"])
        
        # Visualize conflicts if requested
        if args.visualize_conflicts:
            visualize_conflicts(args, primary_mission, missions["simulated_missions"], conflicts)
    
    logger.info("UAV Strategic Deconfliction System completed successfully")

if __name__ == "__main__":
    main()