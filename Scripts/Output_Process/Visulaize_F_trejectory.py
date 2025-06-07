#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Enhanced trajectory visualization script
For visualizing the trajectory data output by the SLAM system
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches
import os

def setup_chinese_font():
    """Set Chinese font and global font size"""
    try:
        # Try to set Chinese font
        plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'WenQuanYi Micro Hei']
        plt.rcParams['axes.unicode_minus'] = False
    except:
        # If Chinese font is unavailable, use English
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
        print("Warning: Chinese fonts not available, using English labels")
    
    # Set global font size
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.titlesize'] = 14
    plt.rcParams['axes.labelsize'] = 12
    plt.rcParams['xtick.labelsize'] = 10
    plt.rcParams['ytick.labelsize'] = 10
    plt.rcParams['legend.fontsize'] = 10

def create_output_directories():
    """Create output directories"""
    #! 输出目录
    output_dir = "trajectory_visualization_output"
    plots_dir = os.path.join(output_dir, "plots")
    data_dir = os.path.join(output_dir, "data")
    
    os.makedirs(plots_dir, exist_ok=True)
    os.makedirs(data_dir, exist_ok=True)
    
    return output_dir, plots_dir, data_dir

def load_trajectory(filename):
    """
    Load trajectory file
    Args:
        filename: Trajectory file path
    Returns:
        timestamps: Timestamp array
        positions: Position coordinates array (N, 3)
        quaternions: Quaternion array (N, 4)
    """
    if not os.path.exists(filename):
        raise FileNotFoundError(f"File {filename} not found")
    
    data = np.loadtxt(filename)
    timestamps = data[:, 0]
    positions = data[:, 1:4]  # x, y, z
    quaternions = data[:, 4:8]  # qx, qy, qz, qw
    
    return timestamps, positions, quaternions

def calculate_trajectory_stats(positions):
    """Calculate trajectory statistics"""
    if len(positions) < 2:
        return {
            'length': 0,
            'avg_speed': 0,
            'max_speed': 0,
            'total_displacement': 0
        }
    
    # Calculate trajectory length
    diffs = np.diff(positions, axis=0)
    distances = np.sqrt(np.sum(diffs**2, axis=1))
    total_length = np.sum(distances)
    
    # Calculate total displacement (straight-line distance from start to end)
    total_displacement = np.linalg.norm(positions[-1] - positions[0])
    
    # Calculate speed statistics (assuming equal time intervals)
    speeds = distances  # Simplified processing
    avg_speed = np.mean(speeds)
    max_speed = np.max(speeds)
    
    return {
        'length': total_length,
        'avg_speed': avg_speed,
        'max_speed': max_speed,
        'total_displacement': total_displacement,
        'distances': distances
    }

def plot_trajectory_2d_enhanced(positions, title="Trajectory Visualization (2D Views)"):
    """
    Plot enhanced 2D trajectory graph
    """
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(title, fontsize=18, fontweight='bold', y=1.02)
    
    # Calculate statistics
    stats = calculate_trajectory_stats(positions)
    
    # XY Plane (Top View)
    axes[0, 0].plot(positions[:, 0], positions[:, 1], 'b-', linewidth=1.5, alpha=0.8, label='Trajectory')
    # Mark start and end points
    axes[0, 0].plot(positions[0, 0], positions[0, 1], 'gs', markersize=10, label='Start')
    axes[0, 0].plot(positions[-1, 0], positions[-1, 1], 'r^', markersize=10, label='End')
    axes[0, 0].set_xlabel('X (m)', fontsize=14)
    axes[0, 0].set_ylabel('Y (m)', fontsize=14)
    axes[0, 0].set_title('XY Plane (Top View)', fontsize=16)
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend(fontsize=12)
    axes[0, 0].axis('equal')
    
    # XZ Plane (Side View)
    axes[0, 1].plot(positions[:, 0], positions[:, 2], 'b-', linewidth=1.5, alpha=0.8, label='Trajectory')
    axes[0, 1].plot(positions[0, 0], positions[0, 2], 'gs', markersize=10, label='Start')
    axes[0, 1].plot(positions[-1, 0], positions[-1, 2], 'r^', markersize=10, label='End')
    axes[0, 1].set_xlabel('X (m)', fontsize=14)
    axes[0, 1].set_ylabel('Z (m)', fontsize=14)
    axes[0, 1].set_title('XZ Plane (Side View)', fontsize=16)
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend(fontsize=12)
    
    # YZ Plane (Front View)
    axes[1, 0].plot(positions[:, 1], positions[:, 2], 'b-', linewidth=1.5, alpha=0.8, label='Trajectory')
    axes[1, 0].plot(positions[0, 1], positions[0, 2], 'gs', markersize=10, label='Start')
    axes[1, 0].plot(positions[-1, 1], positions[-1, 2], 'r^', markersize=10, label='End')
    axes[1, 0].set_xlabel('Y (m)', fontsize=14)
    axes[1, 0].set_ylabel('Z (m)', fontsize=14)
    axes[1, 0].set_title('YZ Plane (Front View)', fontsize=16)
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend(fontsize=12)
    
    # Statistics
    axes[1, 1].axis('off')
    stats_text = f"""Trajectory Statistics:

- Data points: {len(positions)}
- Path length: {stats['length']:.3f} m
- Displacement: {stats['total_displacement']:.3f} m
- Avg step size: {stats['avg_speed']:.6f} m
- Max step size: {stats['max_speed']:.6f} m

Coordinate Ranges:
- X range: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}] m
- Y range: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}] m
- Z range: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}] m

Path Efficiency: {(stats['total_displacement']/stats['length']*100):.2f}%"""
    
    axes[1, 1].text(0.05, 0.95, stats_text, transform=axes[1, 1].transAxes,
                    fontsize=12, verticalalignment='top', fontfamily='monospace',
                    bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
    
    plt.tight_layout()
    return fig

def plot_trajectory_3d_enhanced(positions, title="3D Trajectory Visualization"):
    """
    Create enhanced 3D trajectory plot
    """
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(title, fontsize=18, fontweight='bold', y=0.95)
    
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot trajectory with color mapping to show time progression
    n_points = len(positions)
    colors = plt.cm.viridis(np.linspace(0, 1, n_points))
    
    # Plot trajectory line
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
            'b-', linewidth=2, alpha=0.8, label='Trajectory')
    
    # Mark start and end points
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2],
               c='green', s=200, marker='s', label='Start', edgecolors='black', linewidth=2)
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2],
               c='red', s=200, marker='^', label='End', edgecolors='black', linewidth=2)
    
    # Add color-mapped points for time progression
    scatter = ax.scatter(positions[::10, 0], positions[::10, 1], positions[::10, 2],
                        c=np.arange(0, len(positions), 10), cmap='viridis',
                        s=30, alpha=0.7, label='Time progression')
    
    ax.set_xlabel('X (m)', fontsize=14)
    ax.set_ylabel('Y (m)', fontsize=14)
    ax.set_zlabel('Z (m)', fontsize=14)
    ax.legend(fontsize=12)
    
    # Add color bar
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.5, aspect=20)
    cbar.set_label('Time progression', fontsize=12)
    
    # Set equal aspect ratio for the axes
    max_range = np.array([positions[:, 0].max()-positions[:, 0].min(),
                         positions[:, 1].max()-positions[:, 1].min(),
                         positions[:, 2].max()-positions[:, 2].min()]).max() / 2.0
    
    mid_x = (positions[:, 0].max()+positions[:, 0].min()) * 0.5
    mid_y = (positions[:, 1].max() + positions[:, 1].min()) * 0.5
    mid_z = (positions[:, 2].max() + positions[:, 2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    return fig

def plot_motion_analysis(timestamps, positions):
    """
    Generate motion analysis charts
    """
    # Convert timestamps to relative time (seconds)
    time = (timestamps - timestamps[0]) / 1e9  # nanoseconds to seconds
    
    # Calculate statistics
    stats = calculate_trajectory_stats(positions)
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Motion Analysis', fontsize=18, fontweight='bold', y=1.02)
    
    # Coordinate changes over time
    coordinates = ['X', 'Y', 'Z']
    colors = ['red', 'green', 'blue']
    
    for i in range(3):
        axes[0, 0].plot(time, positions[:, i], color=colors[i], linewidth=1.5,
                       alpha=0.8, label=f'{coordinates[i]} coordinate')
    
    axes[0, 0].set_xlabel('Time (s)', fontsize=14)
    axes[0, 0].set_ylabel('Position (m)', fontsize=14)
    axes[0, 0].set_title('Position vs Time', fontsize=16)
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend(fontsize=12)
    
    # Step length analysis
    if len(stats['distances']) > 0:
        axes[0, 1].plot(stats['distances'], 'b-', linewidth=1.5, alpha=0.8)
        axes[0, 1].set_xlabel('Step Number', fontsize=14)
        axes[0, 1].set_ylabel('Step Size (m)', fontsize=14)
        axes[0, 1].set_title('Step Size Analysis', fontsize=16)
        axes[0, 1].grid(True, alpha=0.3)
        
        # Add statistical lines
        axes[0, 1].axhline(y=np.mean(stats['distances']), color='r', linestyle='--',
                          label=f'Mean: {np.mean(stats["distances"]):.6f}')
        axes[0, 1].axhline(y=np.median(stats['distances']), color='g', linestyle='--',
                          label=f'Median: {np.median(stats["distances"]):.6f}')
        axes[0, 1].legend(fontsize=12)
    
    # Trajectory density heatmap (XY plane)
    try:
        h = axes[1, 0].hist2d(positions[:, 0], positions[:, 1], bins=30, cmap='Blues', alpha=0.7)
        axes[1, 0].plot(positions[:, 0], positions[:, 1], 'r-', linewidth=1, alpha=0.5)
        plt.colorbar(h[3], ax=axes[1, 0])
    except:
        axes[1, 0].plot(positions[:, 0], positions[:, 1], 'b-', linewidth=1.5)
    
    axes[1, 0].set_xlabel('X (m)', fontsize=14)
    axes[1, 0].set_ylabel('Y (m)', fontsize=14)
    axes[1, 0].set_title('Trajectory Density (XY Plane)', fontsize=16)
    
    # Height variation analysis
    axes[1, 1].plot(time, positions[:, 2], 'b-', linewidth=1.5, alpha=0.8, label='Height')
    axes[1, 1].fill_between(time, positions[:, 2], alpha=0.3)
    axes[1, 1].set_xlabel('Time (s)', fontsize=14)
    axes[1, 1].set_ylabel('Z (m)', fontsize=14)
    axes[1, 1].set_title('Height vs Time', fontsize=16)
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend(fontsize=12)
    
    plt.tight_layout()
    return fig

def main():
    """Main function"""
    # Set font
    setup_chinese_font()
    
    # Create output directory
    output_dir, plots_dir, data_dir = create_output_directories()
    
    # File path (only need the full trajectory file)
    trajectory_file = "/home/zyh/Desktop/ORB-SLAM3_byZyh/f_Track_custom_dataset_name.txt"
    
    try:
        # Load data
        print("Loading trajectory data...")
        timestamps, positions, quaternions = load_trajectory(trajectory_file)
        
        print(f"Trajectory data points: {len(positions)}")
        
        # Copy trajectory file to data directory
        import shutil
        trajectory_file_name = os.path.basename(trajectory_file)
        shutil.copy2(trajectory_file, os.path.join(data_dir, trajectory_file_name))
        print(f"Trajectory file copied to: {data_dir}")
        
        # Create visualization charts
        print("Generating enhanced 2D trajectory plots...")
        fig1 = plot_trajectory_2d_enhanced(positions)
        
        print("Generating enhanced 3D trajectory plot...")
        fig2 = plot_trajectory_3d_enhanced(positions)
        
        print("Generating motion analysis plots...")
        fig3 = plot_motion_analysis(timestamps, positions)
        
        # Save images to plots directory
        print("Saving plots...")
        fig1.savefig(os.path.join(plots_dir, 'trajectory_2d_enhanced.png'), dpi=300, bbox_inches='tight')
        fig2.savefig(os.path.join(plots_dir, 'trajectory_3d_enhanced.png'), dpi=300, bbox_inches='tight')
        fig3.savefig(os.path.join(plots_dir, 'motion_analysis.png'), dpi=300, bbox_inches='tight')
        
        print("Plots saved to:")
        print(f"- {plots_dir}/trajectory_2d_enhanced.png: 2D trajectory views")
        print(f"- {plots_dir}/trajectory_3d_enhanced.png: 3D trajectory with time progression")
        print(f"- {plots_dir}/motion_analysis.png: Motion analysis plots")
        print(f"\nAll outputs saved in: {output_dir}/")
        
        # Display charts
        plt.show()
        
    except FileNotFoundError as e:
        print(f"Error: File not found - {e}")
        print("Please ensure the following file exists:")
        print(f"- {trajectory_file}")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
