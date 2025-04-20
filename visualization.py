"""
Visualization utilities for UAV Strategic Deconfliction.
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os
from typing import List, Dict, Tuple, Optional
from datetime import datetime

from data_structures import Mission3D, Conflict3D, Waypoint3D

def set_axes_equal(ax):
    """
    Set equal aspect ratio for 3D plot.
    
    Args:
        ax: Matplotlib 3D axis
    """
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    
    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])
    
    max_range = max([x_range, y_range, z_range])
    
    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)
    
    ax.set_xlim3d([x_middle - max_range/2, x_middle + max_range/2])
    ax.set_ylim3d([y_middle - max_range/2, y_middle + max_range/2])
    ax.set_zlim3d([z_middle - max_range/2, z_middle + max_range/2])

def create_overview_visualization(
    missions: Dict, 
    urban_centers: Optional[List[Tuple[float, float, float]]] = None,
    area_bounds: Tuple[float, float, float, float] = (0, 1000, 0, 1000),
    show_plot: bool = True,
    save_path: Optional[str] = None
) -> None:
    """
    Create overview visualization of all missions in 2D and 3D.
    
    Args:
        missions: Dictionary with primary and simulated missions
        urban_centers: List of (x, y, radius) representing urban areas
        area_bounds: (min_x, max_x, min_y, max_y) simulation area boundaries
        show_plot: Whether to display the plot
        save_path: Path to save the visualization (if None, don't save)
    """
    # Create figure with 2 subplots (2D and 3D)
    fig = plt.figure(figsize=(18, 8))
    
    # 2D Plot
    ax1 = fig.add_subplot(121)
    
    # Plot urban centers (delivery hubs & destinations) if provided
    if urban_centers:
        for x, y, radius in urban_centers:
            circle = plt.Circle((x, y), radius, color='blue', alpha=0.2, label='_nolegend_')
            ax1.add_patch(circle)
            ax1.text(x, y, "Hub", ha='center', va='center', color='blue', fontweight='bold')
    
    # Plot mission paths
    legend_handles = []
    
    # Plot primary missions
    for i, mission in enumerate(missions["primary_missions"]):
        x_coords = [wp.x for wp in mission.waypoints]
        y_coords = [wp.y for wp in mission.waypoints]
        
        line, = ax1.plot(x_coords, y_coords, 'o-', linewidth=2, 
                       label=f"{mission.uav_id}")
        legend_handles.append(line)
        
        # Mark start and end
        ax1.plot(x_coords[0], y_coords[0], '^', color=line.get_color(), markersize=10)
        ax1.plot(x_coords[-1], y_coords[-1], 's', color=line.get_color(), markersize=10)
    
    # Plot simulated missions (with thinner lines)
    for i, mission in enumerate(missions["simulated_missions"]):
        if i < 5:  # Only show first 5 in legend to avoid clutter
            x_coords = [wp.x for wp in mission.waypoints]
            y_coords = [wp.y for wp in mission.waypoints]
            
            line, = ax1.plot(x_coords, y_coords, 'o-', linewidth=1, alpha=0.7,
                           label=f"{mission.uav_id}")
            legend_handles.append(line)
            
            # Mark start and end
            ax1.plot(x_coords[0], y_coords[0], '^', color=line.get_color(), markersize=8)
            ax1.plot(x_coords[-1], y_coords[-1], 's', color=line.get_color(), markersize=8)
        else:
            x_coords = [wp.x for wp in mission.waypoints]
            y_coords = [wp.y for wp in mission.waypoints]
            
            ax1.plot(x_coords, y_coords, 'o-', linewidth=1, alpha=0.7, label='_nolegend_')
            
            # Mark start and end without adding to legend
            ax1.plot(x_coords[0], y_coords[0], '^', color=plt.gca().lines[-1].get_color(), 
                    markersize=8)
            ax1.plot(x_coords[-1], y_coords[-1], 's', color=plt.gca().lines[-1].get_color(), 
                    markersize=8)
    
    # Add custom legend elements for environment features
    from matplotlib.lines import Line2D
    custom_lines = []
    
    if urban_centers:
        custom_lines.append(Line2D([0], [0], color='blue', lw=0, marker='o', markersize=10, alpha=0.3))
    
    # Add mission type markers
    mission_markers = [
        Line2D([0], [0], marker='^', color='black', markersize=8, linestyle='None'),
        Line2D([0], [0], marker='s', color='black', markersize=8, linestyle='None')
    ]
    
    # Add environment features to legend
    legend_labels = [h.get_label() for h in legend_handles]
    if urban_centers:
        legend_labels += ['Delivery Hub/Urban Area']
    legend_labels += ['Mission Start', 'Mission End']
    
    ax1.legend(
        handles=legend_handles + custom_lines + mission_markers,
        labels=legend_labels,
        loc='center left', bbox_to_anchor=(1, 0.5), fontsize=8
    )
    
    # Set plot limits and labels
    min_x, max_x, min_y, max_y = area_bounds
    ax1.set_xlim(min_x, max_x)
    ax1.set_ylim(min_y, max_y)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('2D Visualization of UAV Missions')
    ax1.grid(True, alpha=0.3)
    
    # 3D Plot
    ax2 = fig.add_subplot(122, projection='3d')
    
    # Plot mission paths in 3D
    for mission in missions["primary_missions"]:
        x_coords = [wp.x for wp in mission.waypoints]
        y_coords = [wp.y for wp in mission.waypoints]
        z_coords = [wp.z for wp in mission.waypoints]
        
        ax2.plot(x_coords, y_coords, z_coords, 'o-', linewidth=2)
        
        # Mark start and end
        ax2.scatter([x_coords[0]], [y_coords[0]], [z_coords[0]], 
                   marker='^', s=100, color=plt.gca().lines[-1].get_color())
        ax2.scatter([x_coords[-1]], [y_coords[-1]], [z_coords[-1]], 
                   marker='s', s=100, color=plt.gca().lines[-1].get_color())
    
    # Plot simulated missions
    for mission in missions["simulated_missions"]:
        x_coords = [wp.x for wp in mission.waypoints]
        y_coords = [wp.y for wp in mission.waypoints]
        z_coords = [wp.z for wp in mission.waypoints]
        
        ax2.plot(x_coords, y_coords, z_coords, 'o-', linewidth=1, alpha=0.7)
        
        # Mark start and end
        ax2.scatter([x_coords[0]], [y_coords[0]], [z_coords[0]], 
                   marker='^', s=80, color=plt.gca().lines[-1].get_color())
        ax2.scatter([x_coords[-1]], [y_coords[-1]], [z_coords[-1]], 
                   marker='s', s=80, color=plt.gca().lines[-1].get_color())
    
    # Set plot labels
    ax2.set_xlabel('X Position (m)')
    ax2.set_ylabel('Y Position (m)')
    ax2.set_zlabel('Altitude (m)')
    ax2.set_title('3D Visualization of UAV Missions')
    
    # Add grid
    ax2.grid(True, alpha=0.3)
    
    # Adjust view angle
    ax2.view_init(elev=30, azim=45)
    
    # Set equal aspect ratio
    set_axes_equal(ax2)
    
    # Add mission count to title
    primary_count = len(missions["primary_missions"])
    sim_count = len(missions["simulated_missions"])
    fig.suptitle(f'UAV Missions ({primary_count} Primary, {sim_count} Simulated)', 
                fontsize=16)
    
    # Adjust layout
    plt.tight_layout()
    fig.subplots_adjust(top=0.9)
    
    # Save if requested
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    # Show if requested
    if show_plot:
        plt.show()
    else:
        plt.close()

def plot_drone_path(ax, mission: Mission3D, color, label: str, linewidth: int = 2, alpha: float = 1.0):
    """
    Plot a drone's path with connected segments between waypoints.
    
    Args:
        ax: Matplotlib 3D axis
        mission: The mission to plot
        color: Color for plotting
        label: Label for legend
        linewidth: Line width
        alpha: Transparency (0-1)
    """
    waypoints = mission.waypoints
    
    # Plot connected line segments
    for i in range(len(waypoints) - 1):
        xs = [waypoints[i].x, waypoints[i+1].x]
        ys = [waypoints[i].y, waypoints[i+1].y]
        zs = [waypoints[i].z, waypoints[i+1].z]
        
        # Plot the line segment
        ax.plot(xs, ys, zs, '-', color=color, linewidth=linewidth, alpha=alpha)
    
    # Plot waypoints
    x_coords = [wp.x for wp in waypoints]
    y_coords = [wp.y for wp in waypoints]
    z_coords = [wp.z for wp in waypoints]
    
    ax.scatter(x_coords, y_coords, z_coords, color=color, s=30, marker='o', 
              edgecolors='black', linewidth=0.5)
    
    # Mark start point with a triangle
    ax.scatter(waypoints[0].x, waypoints[0].y, waypoints[0].z, color=color, s=80, 
              marker='^', edgecolors='black', linewidth=1)
    
    # Mark end point with a square
    ax.scatter(waypoints[-1].x, waypoints[-1].y, waypoints[-1].z, color=color, s=80, 
              marker='s', edgecolors='black', linewidth=1)

def create_conflict_visualization(
    primary_mission: Mission3D, 
    simulated_missions: List[Mission3D], 
    conflicts: List[Conflict3D],
    output_file: str = 'drone_conflicts_3d.png',
    show_plot: bool = True
) -> str:
    """
    Create a 3D visualization showing all drone paths and conflict points.
    
    Args:
        primary_mission: The primary mission (will be plotted in green)
        simulated_missions: List of all simulated missions
        conflicts: List of detected conflicts (will be marked in red)
        output_file: Output file path
        show_plot: Whether to display the plot
        
    Returns:
        Path to the saved visualization file
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Identify drones involved in conflicts
    conflict_drone_ids = set()
    for conflict in conflicts:
        if conflict.uav_id_1 == primary_mission.uav_id:
            conflict_drone_ids.add(conflict.uav_id_2)
        elif conflict.uav_id_2 == primary_mission.uav_id:
            conflict_drone_ids.add(conflict.uav_id_1)
    
    # Create color map for drones (primary is always green)
    drone_colors = {}
    drone_colors[primary_mission.uav_id] = 'green'
    
    # Assign colors to other drones (conflicting drones get brighter colors)
    color_options = ['blue', 'orange', 'purple', 'brown', 'magenta', 'cyan', 'lime']
    color_idx = 0
    
    # First assign colors to conflicting drones
    for mission in simulated_missions:
        if mission.uav_id in conflict_drone_ids:
            drone_colors[mission.uav_id] = color_options[color_idx % len(color_options)]
            color_idx += 1
    
    # Then assign colors to non-conflicting drones
    for mission in simulated_missions:
        if mission.uav_id not in drone_colors:
            drone_colors[mission.uav_id] = color_options[color_idx % len(color_options)]
            color_idx += 1
    
    # Plot primary mission
    plot_drone_path(ax, primary_mission, drone_colors[primary_mission.uav_id], 
                  label=f"{primary_mission.uav_id} (Primary)", linewidth=3)
    
    # Plot all simulated missions
    for mission in simulated_missions:
        # Highlight conflicting drones with thicker lines
        linewidth = 2.5 if mission.uav_id in conflict_drone_ids else 1.5
        alpha = 1.0 if mission.uav_id in conflict_drone_ids else 0.7
        
        plot_drone_path(ax, mission, drone_colors[mission.uav_id], 
                      label=mission.uav_id, linewidth=linewidth, alpha=alpha)
    
    # Mark all conflict points with red dots
    if conflicts:
        x_conflicts = [c.location.x for c in conflicts]
        y_conflicts = [c.location.y for c in conflicts]
        z_conflicts = [c.location.z for c in conflicts]
        
        ax.scatter(x_conflicts, y_conflicts, z_conflicts, 
                  color='red', s=100, marker='o', label='Conflicts', 
                  edgecolors='black', linewidth=1, alpha=1.0)
        
        # Add a text annotation for each conflict
        for i, conflict in enumerate(conflicts):
            ax.text(conflict.location.x, conflict.location.y, conflict.location.z + 10,  
                   f"C{i+1}", color='red', fontweight='bold', fontsize=10)
    
    # Add annotations for conflicting drones
    for mission in [primary_mission] + simulated_missions:
        if mission.uav_id == primary_mission.uav_id or mission.uav_id in conflict_drone_ids:
            # Annotate the start of each flight path
            wp = mission.waypoints[0]
            ax.text(wp.x, wp.y, wp.z + 20, 
                   mission.uav_id, color=drone_colors[mission.uav_id], 
                   fontweight='bold', fontsize=12)
    
    # Set labels and title
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_zlabel('Z Position (m)', fontsize=12)
    
    title = f"UAV Flight Paths and Conflicts\n"
    if conflicts:
        title += f"({len(conflicts)} conflicts detected between {len(conflict_drone_ids) + 1} drones)"
    else:
        title += "(No conflicts detected)"
    
    ax.set_title(title, fontsize=14, fontweight='bold')
    
    # Add grid
    ax.grid(True, alpha=0.3)
    
    # Set view angle
    ax.view_init(elev=35, azim=30)
    
    # Add legend with only the primary and conflicting drones
    legend_elements = []
    
    # Primary drone
    legend_elements.append(plt.Line2D([0], [0], color=drone_colors[primary_mission.uav_id], 
                                     lw=3, label=f"{primary_mission.uav_id} (Primary)"))
    
    # Conflicting drones
    for drone_id in conflict_drone_ids:
        legend_elements.append(plt.Line2D([0], [0], color=drone_colors[drone_id], 
                                         lw=2.5, label=drone_id))
    
    # Conflicts
    if conflicts:
        legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', 
                                         markerfacecolor='red', markersize=10, 
                                         label='Conflict Point'))
    
    ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.05, 1), fontsize=10)
    
    # Set equal aspect ratio
    set_axes_equal(ax)
    
    # Tight layout
    plt.tight_layout()
    
    # Save figure
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    
    # Show plot if requested
    if show_plot:
        plt.show()
    else:
        plt.close()
    
    return output_file