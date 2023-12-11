import pygame
import random
import heapq
import math

# Define simulation parameters
grid_size = 20  # Number of grid cells in each dimension
cell_size = 30  # Size of each grid cell in pixels
population_density = 0.2  # Percentage of population density
evacuation_speed = 100  # Speed at which people evacuate (lower is faster)
cyclone_path = [(0, 0), (5, 10), (10, 15), (15, 19), (19, 19)]  # Example cyclone path

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((grid_size * cell_size, grid_size * cell_size))
pygame.display.set_caption("Cyclone Evacuation Simulation")

# Define grid representing the map
grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

# Define colors
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
RED = (255, 0, 0)

# Define cell states
EMPTY = 0
RESIDENT = 1
CYCLONE = 2
SHELTER = 3

# Define class for residents
class Resident(pygame.sprite.Sprite):
    def __init__(self, x, y):
        super().__init__()
        self.image = pygame.Surface((cell_size, cell_size))
        self.image.fill(BLUE)
        self.rect = self.image.get_rect()
        self.rect.x = x * cell_size
        self.rect.y = y * cell_size

    def update(self):
        pass

# Create groups for sprites
all_sprites = pygame.sprite.Group()
residents = pygame.sprite.Group()
shelters = []

# Populate grid with population based on density
for i in range(grid_size):
    for j in range(grid_size):
        if random.random() < population_density:
            resident = Resident(j, i)
            all_sprites.add(resident)
            residents.add(resident)
            grid[i][j] = RESIDENT

# Main simulation loop
running = True
clock = pygame.time.Clock()

def heuristic(node, goal):
    return math.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

def astar(grid, start, goal):
    open_list = []
    closed_set = set()
    heapq.heappush(open_list, (0, start))
    came_from = {}

    g_score = {(node[0], node[1]): float('inf') for node in grid}
    g_score[start] = 0
    f_score = {(node[0], node[1]): float('inf') for node in grid}
    f_score[start] = heuristic(start, goal)

    while open_list:
        current = heapq.heappop(open_list)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        closed_set.add(current)
        for neighbor in get_neighbors(current):
            if neighbor in closed_set:
                continue

            tentative_g_score = g_score[(current[0], current[1])] + 1

            if neighbor not in [i[1] for i in open_list]:
                heapq.heappush(open_list, (tentative_g_score + heuristic(neighbor, goal), neighbor))
            elif tentative_g_score >= g_score[(neighbor[0], neighbor[1])]:
                continue

            came_from[neighbor] = current
            g_score[(neighbor[0], neighbor[1])] = tentative_g_score
            f_score[(neighbor[0], neighbor[1])] = g_score[(neighbor[0], neighbor[1])] + heuristic(neighbor, goal)

    return None

def get_neighbors(node):
    neighbors = []
    x, y = node
    if x > 0:
        neighbors.append((x - 1, y))
    if x < grid_size - 1:
        neighbors.append((x + 1, y))
    if y > 0:
        neighbors.append((x, y - 1))
    if y < grid_size - 1:
        neighbors.append((x, y + 1))
    return neighbors

def find_nearest_shelter(resident, shelters):
    if not shelters:
        return None

    resident_position = (resident.rect.y // cell_size, resident.rect.x // cell_size)
    nearest_shelter = min(shelters, key=lambda shelter: len(astar(grid, resident_position, shelter)))
    return nearest_shelter

# Define shelter locations
shelters = [(5, 5), (15, 5)]  # Example shelter locations

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False

    # Simulate cyclone progression
    if len(cyclone_path) > 0:
        cyclone_position = cyclone_path.pop(0)
        grid[cyclone_position[0]][cyclone_position[1]] = CYCLONE

    # Evacuation simulation
    for resident in residents:
        nearest_shelter = find_nearest_shelter(resident, shelters)
        if nearest_shelter:
            path = astar(grid, (resident.rect.y // cell_size, resident.rect.x // cell_size), nearest_shelter)
            if path:
                for cell in path:
                    pygame.draw.rect(screen, WHITE, (cell[1] * cell_size, cell[0] * cell_size, cell_size, cell_size))
                    pygame.display.update()
                    pygame.time.wait(evacuation_speed)
                shelter_x, shelter_y = nearest_shelter
                resident.rect.x = shelter_y * cell_size
                resident.rect.y = shelter_x * cell_size

    # Clear the screen
    screen.fill((255, 255, 255))

    # Draw the grid
    for i in range(grid_size):
        for j in range(grid_size):
            if grid[i][j] == RESIDENT:
                pygame.draw.rect(screen, BLUE, (j * cell_size, i * cell_size, cell_size, cell_size))
            elif grid[i][j] == CYCLONE:
                pygame.draw.rect(screen, RED, (j * cell_size, i * cell_size, cell_size, cell_size))
            elif grid[i][j] == SHELTER:
                pygame.draw.rect(screen, (0, 255, 0), (j * cell_size, i * cell_size, cell_size, cell_size))

    pygame.display.flip()
    clock.tick(5)  # Adjust the simulation speed here

pygame.quit()
