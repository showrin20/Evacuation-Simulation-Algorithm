import pygame
import random
import heapq
import math
import sys

# Define simulation parameters
grid_size = 20  # Number of grid cells in each dimension
cell_size = 30  # Size of each grid cell in pixels
population_density = 0.2  # Percentage of population density
evacuation_speed = 1000  # Speed at which people evacuate (lower is slower)
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
GREEN = (0, 255, 0)

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
        self.path = None  # Current evacuation path

    def update(self):
        if self.path and len(self.path) > 0:
            next_cell = self.path.pop(0)
            self.rect.x = next_cell[1] * cell_size
            self.rect.y = next_cell[0] * cell_size


# Define class for shelters
class Shelter(pygame.sprite.Sprite):
    def __init__(self, x, y):
        super().__init__()
        self.image = pygame.Surface((cell_size, cell_size))
        self.image.fill(GREEN)
        self.rect = self.image.get_rect()
        self.rect.x = x * cell_size
        self.rect.y = y * cell_size
        self.capacity = 100  # Shelter capacity
        self.residents = 0  # Number of residents currently in the shelter


# Create groups for sprites
all_sprites = pygame.sprite.Group()
residents = pygame.sprite.Group()
shelters = pygame.sprite.Group()

# Populate grid with population based on density
for i in range(grid_size):
    for j in range(grid_size):
        if random.random() < population_density:
            resident = Resident(j, i)
            residents.add(resident)
            all_sprites.add(resident)
            grid[i][j] = RESIDENT

# Define shelter locations
shelters = [Shelter(5, 5), Shelter(15, 5)]  # Example shelter locations

# Define simulation duration (in seconds)
simulation_duration = 200  # 2 minutes

# Calculate the number of frames needed to achieve the desired duration
frames_per_second =  40# Adjust this value to control the simulation speed
total_frames = simulation_duration * frames_per_second


# Define heuristic function for A*
def heuristic(node, goal):
    return math.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)


# Find nearest shelter using A*
def find_nearest_shelter_a_star(resident):
    nearest_shelter = None
    min_distance = float("inf")
    for shelter in shelters:
        if shelter.capacity <= shelter.residents:
            continue
        path = astar(
            grid,
            (resident.rect.y // cell_size, resident.rect.x // cell_size),
            (shelter.rect.y // cell_size, shelter.rect.x // cell_size),
        )

        if path:
            distance = len(path)
            if distance < min_distance:
                nearest_shelter = shelter
                min_distance = distance

    if nearest_shelter:
        resident.path = path
        nearest_shelter.residents += 1
    return nearest_shelter


# A* algorithm implementation
def astar(grid, start, goal):
    open_list = []
    closed_set = set()
    came_from = {}

    g_score = {(node[0], node[1]): float("inf") for node in grid}
    g_score[start] = 0
    f_score = {(node[0], node[1]): float("inf") for node in grid}
    f_score[start] = heuristic(start, goal)

    heapq.heappush(open_list, (f_score[start], start))

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

            tentative_g_score = g_score[current] + 1

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)

                if neighbor not in [(i[1][0], i[1][1]) for i in open_list]:
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return None


# Get possible neighboring cells
def get_neighbors(node):
    x, y = node
    possible_neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
    neighbors = []

    for neighbor in possible_neighbors:
        x, y = neighbor
        if 0 <= x < grid_size and 0 <= y < grid_size:
            neighbors.append(neighbor)


# Simulation loop
running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Simulate cyclone progression
    if len(cyclone_path) > 0:
        cyclone_position = cyclone_path.pop(0)
        grid[cyclone_position[0]][cyclone_position[1]] = CYCLONE

    # Update residents
    for resident in residents:
        if resident.path and len(resident.path) == 0:
            find_nearest_shelter_a_star(resident)
        resident.update()

    # Clear the screen
    screen.fill((255, 255, 255))

    # Draw the grid
    for i in range(grid_size):
        for j in range(grid_size):
            if grid[i][j] == RESIDENT:
                pygame.draw.rect(
                    screen, BLUE, (j * cell_size, i * cell_size, cell_size, cell_size)
                )
            elif grid[i][j] == CYCLONE:
                pygame.draw.rect(
                    screen, RED, (j * cell_size, i * cell_size, cell_size, cell_size)
                )
            elif grid[i][j] == SHELTER:
                pygame.draw.rect(
                    screen, GREEN, (j * cell_size, i * cell_size, cell_size, cell_size)
                )

    # Draw paths
    for resident in residents:
        if resident.path:
            for cell in resident.path:
                pygame.draw.rect(
                    screen,
                    WHITE,
                    (cell[1] * cell_size, cell[0] * cell_size, cell_size, cell_size),
                )

    # Update sprites
    all_sprites.update()
    all_sprites.draw(screen)

    pygame.display.flip()
    clock.tick(frames_per_second)  # Adjust the frame rate as needed

# Quit pygame
pygame.quit()
sys.exit()
