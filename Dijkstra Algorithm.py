# coding: utf-8
# Created on Thursday Oct 19 22:09:57 2023
# @author: Monjaraz Briseño Luis Fernando

import pygame as pg
import numpy as np
import heapq
import os

class MapNode:
    """Represents a node in the grid map with position and cost information."""
    
    def __init__(self, position, parent=None):
        self.parent = parent
        self.position = position
        self.g = 0  # Cost from start to this node

    def __eq__(self, other):
        return self.position[0] == other.position[0] and self.position[1] == other.position[1]

    def __lt__(self, other):
        return self.g < other.g
    
    def __hash__(self):
        return hash(self.position)


class DijkstraPathfinder:
    """Implements Dijkstra's shortest path algorithm."""
    
    def __init__(self):
        self.movements = [
            [0, -1, 1],   # Up
            [-1, 0, 1],   # Left
            [1, 0, 1],    # Right
            [0, 1, 1]     # Down
        ]
    
    def run(self, grid_map, start, end):
        """
        Finds the shortest path from start to end using Dijkstra's algorithm.
        
        Args:
            grid_map: 2D numpy array where 0 is obstacle, 1 is walkable
            start: [x, y] starting position
            end: [x, y] goal position
            
        Returns:
            tuple: (path, visited_map) where path is list of positions and 
                   visited_map shows explored nodes
        """
        # Convert coordinates to matrix format (row, column)
        start_node = MapNode(start[::-1])
        end_node = MapNode(end[::-1])
        
        path = []
        open_list = []
        visited = np.zeros(grid_map.shape)
        
        heapq.heappush(open_list, (0, start_node))
        
        current_node = None
        while open_list:
            _, current_node = heapq.heappop(open_list)
            
            if current_node == end_node:
                break
            
            for movement in self.movements:
                new_position = [
                    current_node.position[0] + movement[0],
                    current_node.position[1] + movement[1]
                ]
                
                # Boundary and obstacle validation
                if (new_position[0] < 0 or new_position[1] < 0 or 
                    new_position[1] >= grid_map.shape[1] or 
                    new_position[0] >= grid_map.shape[0]):
                    continue
                
                if visited[new_position[0]][new_position[1]] == 1:
                    continue
                
                if grid_map[new_position[0]][new_position[1]] == 0:
                    continue
                
                # Create adjacent node and update accumulated cost
                adjacent_node = MapNode(new_position, current_node)
                adjacent_node.g = current_node.g + movement[2]
                heapq.heappush(open_list, (adjacent_node.g, adjacent_node))
                visited[new_position[0]][new_position[1]] = 1
        
        # Reconstruct path
        while current_node is not None:
            path.append(current_node.position)
            current_node = current_node.parent
        
        return path, visited


class DijkstraVisualization:
    """Handles Pygame visualization of Dijkstra's algorithm."""
    
    def __init__(self, grid_map, start, goal, tile_size=10, top_padding=50):
        self.grid_map = grid_map
        self.width, self.height = grid_map.shape
        self.start = start
        self.goal = goal
        self.tile_size = tile_size
        self.top_padding = top_padding
        
        # Colors
        self.BLACK = pg.Color('black')
        self.WHITE = pg.Color('white')
        self.GREEN = pg.Color('green')
        self.RED = pg.Color('red')
        self.BLUE = pg.Color('blue')
        self.color_light = (170, 170, 170)
        self.color_dark = (100, 100, 100)
        
        # Initialize Pygame
        pg.init()
        self.screen = pg.display.set_mode(
            (self.width * tile_size, self.height * tile_size + top_padding)
        )
        pg.display.set_caption('Dijkstra Algorithm Visualization')
        self.clock = pg.time.Clock()
        
        # Surfaces
        self.background = pg.Surface((self.width * tile_size, self.height * tile_size))
        self.buttons = pg.Surface((self.width * tile_size, top_padding))
        
        # Font
        self.smallfont = pg.font.SysFont('comicsans', 30)
        self.text = self.smallfont.render('Search', True, self.RED)
        
        # Algorithm
        self.pathfinder = DijkstraPathfinder()
        
        self._draw_initial_map()
    
    def _draw_initial_map(self):
        """Draws the initial state of the map."""
        for y in range(self.height):
            for x in range(self.width):
                rect = pg.Rect(x * self.tile_size, y * self.tile_size, 
                             self.tile_size, self.tile_size)
                color = self.BLACK if self.grid_map[y, x] == 0 else self.WHITE
                if x == self.start[0] and y == self.start[1]: 
                    color = self.GREEN
                if x == self.goal[0] and y == self.goal[1]: 
                    color = self.RED
                pg.draw.rect(self.background, color, rect)
    
    def _draw_path(self, path):
        """Draws the computed path on the background."""
        for point in path:
            rect = pg.Rect(point[1] * self.tile_size, point[0] * self.tile_size, 
                         self.tile_size, self.tile_size)
            pg.draw.rect(self.background, self.BLUE, rect)
    
    def run(self):
        """Main visualization loop."""
        game_exit = False
        
        while not game_exit:
            mouse = pg.mouse.get_pos()
            
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    game_exit = True
                
                if event.type == pg.MOUSEBUTTONDOWN:
                    # Check if Search button was clicked
                    if 10 < mouse[0] < 150 and 10 < mouse[1] <= 40:
                        path, visited_map = self.pathfinder.run(
                            self.grid_map, self.start, self.goal
                        )
                        self._draw_path(path)
            
            # Render buttons with hover effect
            if 10 <= mouse[0] <= 150 and 10 < mouse[1] <= 40:
                pg.draw.rect(self.buttons, self.color_light, [10, 10, 140, 30])
            else:
                pg.draw.rect(self.buttons, self.color_dark, [10, 10, 140, 30])
            
            # Update screen
            self.screen.fill((0, 0, 0))
            self.screen.blit(self.buttons, (0, 0))
            self.screen.blit(self.background, (0, self.top_padding))
            self.screen.blit(self.text, (10, 10))
            pg.display.flip()
            self.clock.tick(30)
        
        pg.display.quit()


def main():
    """Main entry point for the application."""
    map_file = 'mapaProfundidad.npy'
    
    if not os.path.exists(map_file):
        print(f"Error: Map file '{map_file}' not found.")
        return
    
    try:
        grid_map = np.load(map_file)
        start = [14, 6]
        goal = [46, 39]
        
        visualizer = DijkstraVisualization(grid_map, start, goal)
        visualizer.run()
        
    except Exception as e:
        print(f"Error loading or running visualization: {e}")


if __name__ == "__main__":
    main()