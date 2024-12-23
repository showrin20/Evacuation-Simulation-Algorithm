# Cyclone Evacuation Simulation

This project is a **multi-agent-based cyclone evacuation simulation** built with **Pygame**. It demonstrates the evacuation process of residents to nearby shelters using **A* pathfinding** in the presence of a dynamically moving cyclone. The project provides a visual representation of the grid-based evacuation, showcasing residents navigating to shelters while avoiding the cyclone's path.

## Features

- **Multi-Agent Simulation**: Simulates multiple residents finding and traveling to shelters.
- **Dynamic Cyclone Path**: The cyclone moves unpredictably, altering the environment in real-time.
- **A* Pathfinding Algorithm**: Residents navigate efficiently to the nearest available shelter.
- **Capacity Management**: Shelters have limited capacities, requiring residents to find alternatives.
- **Customizable Parameters**: Adjust grid size, population density, shelter locations, and more.

## Installation

### Prerequisites

- Python 3.7 or higher

### Required Libraries

Install the necessary Python libraries using `pip`:

```bash
pip install pygame
pip install numpy
pip install heapq
```

### Optional Dependencies

If you encounter issues with `pygame` or your system lacks support for audio/microphone input:

```bash
pip install pyaudio
```

Ensure `pip` is up to date:

```bash
pip install --upgrade pip
```

## How to Run

1. Clone this repository:

   ```bash
   git clone https://github.com/your-username/cyclone-evacuation-simulation.git
   cd cyclone-evacuation-simulation
   ```

2. Ensure all dependencies are installed (see above).

3. Run the simulation:

   ```bash
   python cyclone_simulation.py
   ```

## Simulation Details

1. **Grid Representation**:
   - Each cell represents a portion of the environment.
   - **Resident**: Red circles move toward shelters.
   - **Shelter**: Green squares, limited in capacity.
   - **Cyclone**: Red areas dynamically appear, representing danger zones.

2. **Pathfinding**:
   - Residents use the **A* algorithm** to find the shortest path to a shelter.
   - The algorithm avoids cyclone-affected cells.

3. **Dynamic Cyclone**:
   - The cyclone moves randomly across the grid.
   - Affected areas are marked red, and residents reroute to avoid them.

## Customization

You can customize the simulation parameters in the code:

- **Grid Size**: Adjust `GRID_SIZE` to change the environment size.
- **Cell Size**: Modify `CELL_SIZE` for larger or smaller visual cells.
- **Population Density**: Change `POPULATION_DENSITY` to increase/decrease the number of residents.
- **Shelter Locations**: Add or remove shelters by modifying the `shelter_locations` list.

## Example Output

![Cyclone Evacuation Simulation Screenshot](example.png)

- Red dots represent residents.
- Green squares are shelters.
- Red squares indicate cyclone-affected areas.

## Future Improvements

- **Obstacle Handling**: Add static obstacles for more realistic environments.
- **Improved Cyclone Movement**: Simulate real-world cyclone patterns.
- **Agent Prioritization**: Implement prioritization for vulnerable residents.
- **Enhanced Visualization**: Use advanced graphics to improve the simulation's appearance.
