# Dijkstra Algorithm Visualization

A real-time visualization of Dijkstra's shortest path algorithm using Pygame. This interactive application demonstrates pathfinding on a 2D grid with obstacles, showing the computed shortest path from a start point to a goal.

## Author
Monjaraz Briseño Luis Fernando

## Features

- **Interactive Pathfinding**: Click the "Search" button to compute the shortest path
- **Visual Grid Display**: Clear visualization of obstacles, start point, goal, and computed path
- **Efficient Implementation**: Uses heapq for optimized priority queue operations
- **Object-Oriented Design**: Clean, maintainable code structure with separate classes
- **Real-time Rendering**: Smooth 30 FPS visualization with Pygame

## Algorithm Overview

**Dijkstra's Algorithm** is a graph search algorithm that finds the shortest path between nodes in a graph. It works by:

1. Starting from the initial node with cost 0
2. Exploring all neighboring nodes
3. Always selecting the node with the minimum accumulated cost (g-value)
4. Marking nodes as visited to avoid reprocessing
5. Continuing until the goal is reached

### Time Complexity
- **O((V + E) log V)** where V is the number of vertices and E is the number of edges
- The heapq implementation provides O(log n) operations for priority queue

### Space Complexity
- **O(V)** for storing visited nodes and the priority queue

## Requirements

```bash
pip install pygame numpy
```

### Dependencies
- **Python 3.6+**
- **Pygame**: For graphical interface and event handling
- **NumPy 1.19+**: For efficient array operations and map storage

## File Structure

```
Dijkstra Algorithm/
│
├── Dijkstra Algorithm.py    # Main application file
├── mapaProfundidad.npy      # Grid map file (0 = obstacle, 1 = walkable)
└── README.md                # This file
```

## Usage

### Basic Usage

1. Ensure `mapaProfundidad.npy` exists in the same directory
2. Run the application:

```bash
python "Dijkstra Algorithm.py"
```

3. Click the "Search" button to compute and visualize the shortest path

### Map File Format

The application expects a NumPy binary file (`.npy`) containing a 2D array where:
- **0**: Obstacle (black squares)
- **1**: Walkable path (white squares)

Example of creating a custom map:

```python
import numpy as np

# Create a 50x50 grid (all walkable)
custom_map = np.ones((50, 50), dtype=int)

# Add some obstacles (0 = obstacle)
custom_map[10:15, 20:30] = 0  # Horizontal wall
custom_map[25:35, 15] = 0     # Vertical wall

# Save the map
np.save('mapaProfundidad.npy', custom_map)
```

## Code Structure

### Classes

#### `MapNode`
Represents a node in the grid with position and cost information.

**Attributes:**
- `position`: [row, column] coordinates in the grid
- `parent`: Reference to the parent node for path reconstruction
- `g`: Accumulated cost from start to this node

**Methods:**
- `__eq__()`: Compares nodes by position
- `__lt__()`: Compares nodes by cost (for priority queue)
- `__hash__()`: Makes nodes hashable for set operations

#### `DijkstraPathfinder`
Implements Dijkstra's algorithm for finding the shortest path.

**Attributes:**
- `movements`: List of possible movement directions [dy, dx, cost]

**Methods:**
- `run(grid_map, start, end)`: Computes shortest path and returns (path, visited_map)

#### `DijkstraVisualization`
Handles all Pygame rendering and user interaction.

**Attributes:**
- `grid_map`: 2D NumPy array representing the map
- `start`, `goal`: Starting and goal positions
- `tile_size`: Size of each grid cell in pixels
- `pathfinder`: Instance of DijkstraPathfinder

**Methods:**
- `_draw_initial_map()`: Renders the initial grid state
- `_draw_path(path)`: Draws the computed path
- `run()`: Main event loop

## Customization

### Change Start and Goal Positions

Modify in the `main()` function:

```python
start = [14, 6]   # [x, y] coordinates
goal = [46, 39]   # [x, y] coordinates
```

### Adjust Visualization Settings

```python
visualizer = DijkstraVisualization(
    grid_map, 
    start, 
    goal,
    tile_size=10,      # Size of each grid cell
    top_padding=50     # Space for buttons at the top
)
```

### Modify Movement Directions

Edit the `movements` list in `DijkstraPathfinder` to include diagonal movements:

```python
self.movements = [
    [0, -1, 1],      # Up
    [-1, 0, 1],      # Left
    [1, 0, 1],       # Right
    [0, 1, 1],       # Down
    [-1, -1, 1.414], # Up-Left (diagonal)
    [1, -1, 1.414],  # Up-Right (diagonal)
    [-1, 1, 1.414],  # Down-Left (diagonal)
    [1, 1, 1.414]    # Down-Right (diagonal)
]
```

### Change Colors

Modify color definitions in `DijkstraVisualization.__init__()`:

```python
self.BLACK = pg.Color('black')      # Obstacles
self.WHITE = pg.Color('white')      # Walkable areas
self.GREEN = pg.Color('green')      # Start point
self.RED = pg.Color('red')          # Goal point
self.BLUE = pg.Color('blue')        # Computed path
```

## How It Works

1. **Initialization**: The map is loaded from a NumPy file and the Pygame window is created
2. **User Interaction**: User clicks the "Search" button to trigger pathfinding
3. **Algorithm Execution**:
   - Nodes are explored using a priority queue (heapq)
   - The node with minimum accumulated cost is always selected next
   - Each neighbor is added to the queue with updated cost
   - The process continues until the goal is reached
4. **Path Reconstruction**: The path is traced backward from goal to start using parent references
5. **Visualization**: The computed path is rendered in blue on the grid

## Comparison with Other Algorithms

| Algorithm | Time Complexity | Optimal Path | Use Case |
|-----------|----------------|--------------|----------|
| Dijkstra | O((V+E) log V) | ✓ Guaranteed | Weighted graphs, shortest path |
| A* | O((V+E) log V) | ✓ With admissible heuristic | Weighted graphs with known goal |
| BFS | O(V + E) | ✓ For unweighted | Unweighted graphs |
| DFS | O(V + E) | ✗ Not guaranteed | Graph exploration, not pathfinding |

## Troubleshooting

### Error: Map file not found
```
Error: Map file 'mapaProfundidad.npy' not found.
```
**Solution**: Ensure `mapaProfundidad.npy` exists in the same directory as the script.

### No Path Found
If clicking "Search" doesn't show a path:
- Verify the start and goal positions are valid (not on obstacles)
- Ensure there is a walkable path between start and goal
- Check that map values are 0 (obstacle) or 1 (walkable)

### Pygame Window Not Responding
**Solution**: Ensure you're not blocking the main loop. The algorithm runs quickly enough for real-time visualization.

### Performance Issues
For very large maps:
- Reduce `tile_size` to make the window smaller
- Consider optimizing the map loading process
- Ensure your map file isn't excessively large

## Educational Value

This visualization is excellent for:
- **Algorithm Learning**: Understanding how Dijkstra's algorithm explores nodes
- **Data Structures**: Seeing priority queue operations in action
- **Graph Theory**: Visualizing graph search on a 2D grid
- **Optimization**: Comparing with other pathfinding algorithms like A* or BFS

## Performance Tips

1. **Use heapq**: The current implementation uses Python's built-in heapq for O(log n) priority queue operations
2. **Track Visited Nodes**: The visited array prevents reprocessing nodes
3. **Early Termination**: The algorithm stops immediately when the goal is found
4. **Efficient Data Structures**: NumPy arrays are used for fast array operations

## Future Enhancements

Potential improvements:
- Add A* algorithm comparison mode
- Implement weighted edges (different terrain costs)
- Add real-time editing of obstacles with mouse clicks
- Show visited nodes during computation
- Display algorithm statistics (nodes explored, path length)
- Export path coordinates to file

## License

This project is created for educational purposes.

## References

- Dijkstra, E. W. (1959). "A note on two problems in connexion with graphs"
- Introduction to Algorithms (CLRS) - Chapter 24: Single-Source Shortest Paths
- Pygame Documentation: https://www.pygame.org/docs/
- NumPy Documentation: https://numpy.org/doc/
