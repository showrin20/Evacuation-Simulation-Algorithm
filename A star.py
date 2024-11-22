import pygame
import random
import heapq
import math
import sys

# Initialize pygame
pygame.init()

# Constants
GRID_SIZE = 20
CELL_SIZE = 30
SCREEN_WIDTH = GRID_SIZE * CELL_SIZE
SCREEN_HEIGHT = GRID_SIZE * CELL_SIZE
POPULATION_DENSITY = 0.2
FRAMES_PER_SECOND = 5  # Simulation speed

# Colors
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# Cell states
EMPTY = 0
RESIDENT = 1
CYCLONE = 2
SHELTER = 3

# Pygame setup
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Multi-Agent Cyclone Evacuation Simulation")

# Load background image
background = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
background.fill(WHITE)

# Grid setup
grid = [[EMPTY for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

# Resident class
class Resident(pygame.sprite.Sprite):
    def __init__(self, x, y):
        super().__init__()
        self.image = pygame.Surface((CELL_SIZE - 4, CELL_SIZE - 4), pygame.SRCALPHA)
        pygame.draw.circle(self.image, RED, (CELL_SIZE // 2, CELL_SIZE // 2), CELL_SIZE // 4)
        self.rect = self.image.get_rect()
        self.rect.x = x * CELL_SIZE
        self.rect.y = y * CELL_SIZE
        self.path = None
        self.reached_shelter = False

    def update(self):
        """Update resident's position along the path."""
        if self.path and not self.reached_shelter:
            next_cell = self.path.pop(0)
            self.rect.x = next_cell[1] * CELL_SIZE
            self.rect.y = next_cell[0] * CELL_SIZE
            if not self.path:  # Path completed
                self.reached_shelter = True

# Shelter class
class Shelter(pygame.sprite.Sprite):
    def __init__(self, x, y, capacity=50):
        super().__init__()
        self.image = pygame.Surface((CELL_SIZE, CELL_SIZE))
        self.image.fill(GREEN)
        self.rect = self.image.get_rect()
        self.rect.x = x * CELL_SIZE
        self.rect.y = y * CELL_SIZE
        self.capacity = capacity
        self.residents = 0

# Functions for pathfinding and simulation
def heuristic(node, goal):
    """Heuristic function for A* (Euclidean distance)."""
    return math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)

def get_neighbors(node):
    """Get valid neighbors of a node."""
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    neighbors = []
    for dx, dy in directions:
        x, y = node[0] + dx, node[1] + dy
        if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
            neighbors.append((x, y))
    return neighbors

def astar(grid, start, goal):
    """A* algorithm for pathfinding."""
    open_list = []
    closed_set = set()
    came_from = {}

    g_score = {node: float("inf") for node in [(x, y) for x in range(GRID_SIZE) for y in range(GRID_SIZE)]}
    f_score = {node: float("inf") for node in g_score}
    g_score[start] = 0
    f_score[start] = heuristic(start, goal)

    heapq.heappush(open_list, (f_score[start], start))

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        closed_set.add(current)

        for neighbor in get_neighbors(current):
            if neighbor in closed_set or grid[neighbor[0]][neighbor[1]] == CYCLONE:
                continue

            tentative_g_score = g_score[current] + 1

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)

                if neighbor not in [n for _, n in open_list]:
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return None

def find_nearest_shelter(resident_pos):
    """Find the nearest shelter using A*."""
    nearest_shelter = None
    min_distance = float("inf")
    best_path = None

    for shelter in shelters:
        if shelter.capacity > shelter.residents:
            shelter_pos = (shelter.rect.y // CELL_SIZE, shelter.rect.x // CELL_SIZE)
            path = astar(grid, resident_pos, shelter_pos)
            if path and len(path) < min_distance:
                nearest_shelter = shelter
                min_distance = len(path)
                best_path = path

    if nearest_shelter:
        nearest_shelter.residents += 1
        return best_path
    return None

def update_cyclone_path():
    """Simulate cyclone movement."""
    global cyclone_path
    last_pos = cyclone_path[-1]
    next_pos = (last_pos[0] + random.choice([-1, 0, 1]), last_pos[1] + random.choice([-1, 0, 1]))
    if 0 <= next_pos[0] < GRID_SIZE and 0 <= next_pos[1] < GRID_SIZE:
        cyclone_path.append(next_pos)
        grid[next_pos[0]][next_pos[1]] = CYCLONE

# Initialize residents and shelters
all_sprites = pygame.sprite.Group()
residents = pygame.sprite.Group()
shelters = pygame.sprite.Group()

for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
        if random.random() < POPULATION_DENSITY:
            resident = Resident(j, i)
            residents.add(resident)
            all_sprites.add(resident)
            grid[i][j] = RESIDENT

shelter_locations = [(15, 15), (5, 5), (10, 10)]
for loc in shelter_locations:
    shelter = Shelter(*loc)
    shelters.add(shelter)
    all_sprites.add(shelter)
    grid[loc[0]][loc[1]] = SHELTER

cyclone_path = [(0, 0)]
grid[0][0] = CYCLONE

# Simulation loop
running = True
clock = pygame.time.Clock()
while running:
    screen.blit(background, (0, 0))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    update_cyclone_path()

    for resident in residents:
        if not resident.path and not resident.reached_shelter:
            resident_pos = (resident.rect.y // CELL_SIZE, resident.rect.x // CELL_SIZE)
            resident.path = find_nearest_shelter(resident_pos)
        resident.update()

    all_sprites.update()
    all_sprites.draw(screen)
    pygame.display.flip()
    clock.tick(FRAMES_PER_SECOND)

pygame.quit()
sys.exit()
