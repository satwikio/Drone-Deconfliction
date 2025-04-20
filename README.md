# UAV Strategic Deconfliction in Shared Airspace

This project implements a strategic deconfliction system that verifies whether a drone's planned waypoint mission is safe to execute in shared airspace. The system checks for conflicts in both space and time against the simulated flight paths of multiple other drones.

## Features

- 4D conflict detection (3D spatial coordinates + time)
- Parallel processing using R-tree spatial indexing for efficient conflict detection
- Realistic drone mission generation with smooth paths
- Two types of visualizations:
  - Overview visualization of all drone missions
  - Detailed 3D visualization of conflicts
- Comprehensive logging and result tracking

## Project Structure

- `data_structures.py`: Core data classes for waypoints, missions, and conflicts
- `data_generator.py`: Generation of realistic drone missions
- `conflict_detector.py`: Conflict detection using R-tree and parallel processing
- `visualization.py`: 2D and 3D visualization utilities
- `logger.py`: Logging and result tracking
- `main.py`: Main entry point with command-line interface

## Installation

1. Clone the repository:
```
git clone <repository-url>
cd uav-deconfliction
```

2. Set up a virtual environment (optional but recommended):
```
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```
pip install numpy pandas matplotlib rtree
```

## Usage

### Generating and Visualizing Missions

To generate and visualize drone missions:

```
python main.py --generate --primary 1 --simulated 10 --visualize-overview --save-data
```

This will:
- Generate 1 primary and 10 simulated drone missions
- Create and display an overview visualization
- Save the mission data to the output directory

### Checking for Conflicts

To check for conflicts and visualize the results:

```
python main.py --generate --primary 1 --simulated 10 --safety-buffer 50 --visualize-conflicts --save-conflicts --save-results
```

This will:
- Generate drone missions
- Check for conflicts with a 50-meter safety buffer
- Visualize any detected conflicts
- Save conflict details and check results

### Loading Missions from a File

If you've previously generated and saved missions, you can load them from a JSON file:

```
python main.py --load-json output/delivery_missions_raw_20250420_123456.json --visualize-overview --visualize-conflicts
```

### Full Command Line Options

```
usage: main.py [-h] [--output-dir OUTPUT_DIR] [--no-display] [--generate]
               [--primary PRIMARY] [--simulated SIMULATED] [--save-data]
               [--load-json LOAD_JSON] [--safety-buffer SAFETY_BUFFER]
               [--time-step TIME_STEP] [--workers WORKERS] [--save-conflicts]
               [--save-results] [--visualize-overview] [--visualize-conflicts]

UAV Strategic Deconfliction System

optional arguments:
  -h, --help            show this help message and exit
  --output-dir OUTPUT_DIR
                        Directory to save output files
  --no-display          Do not display visualizations (save only)
  --generate            Generate new mission data
  --primary PRIMARY     Number of primary missions to generate
  --simulated SIMULATED
                        Number of simulated missions to generate
  --save-data           Save generated mission data
  --load-json LOAD_JSON
                        Load missions from JSON file instead of generating
  --safety-buffer SAFETY_BUFFER
                        Safety buffer distance (in meters)
  --time-step TIME_STEP
                        Time step for conflict detection (in seconds)
  --workers WORKERS     Number of parallel workers (default: CPU count)
  --save-conflicts      Save detected conflicts to file
  --save-results        Save mission check results to file
  --visualize-overview  Create overview visualization of all missions
  --visualize-conflicts
                        Create visualization of detected conflicts
```

## Examples

### Example 1: Generate and Visualize Missions

```
python main.py --generate --primary 1 --simulated 15 --visualize-overview --output-dir results
```

### Example 2: Detect and Visualize Conflicts

```
python main.py --generate --primary 1 --simulated 20 --safety-buffer 75 --visualize-conflicts --save-results
```

## Scalability Discussion

For this system to handle real data from tens of thousands of commercial drones, the following architectural changes would be necessary:

1. **Distributed Computing**: 
   - Implement a distributed architecture using technologies like Apache Spark or Dask
   - Partition the airspace into sectors for parallel processing

2. **Real-Time Data Ingestion**:
   - Develop a streaming pipeline using Kafka or similar technologies
   - Implement incremental updates to the spatial index

3. **Optimized Indexing**:
   - Use distributed spatial indexing techniques
   - Implement hierarchical space partitioning (quadtrees, octrees)

4. **Fault Tolerance**:
   - Add redundancy and failover mechanisms
   - Implement checkpointing for recovery

5. **Dynamic Adjustment**:
   - Adapt safety buffers based on drone characteristics and weather conditions
   - Implement priority-based conflict resolution

6. **Cloud Deployment**:
   - Distribute the system across multiple availability zones
   - Implement auto-scaling based on demand

## Contributing

Contributions to improve the system are welcome. Please follow these steps:

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.