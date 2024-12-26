import pygame
import random
import time
import heapq
import threading
import math

# Constants for visualization
WIDTH, HEIGHT = 800, 600
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
ORANGE = (255, 165, 0)
BLUE = (0, 0, 255)
DANGER_THRESHOLD = 0.5  # Danger level above which paths will be drawn in red
BACKGROUND_COLOR = (240, 240, 240)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
font = pygame.font.Font(None, 32)
pygame.display.set_caption("Firefighting Robot Simulation")

# Load the fire emoji image
fire_emoji_image = pygame.image.load("fire_emoji.png")
fire_emoji_image = pygame.transform.scale(fire_emoji_image, (30, 30))


def initialize_graph():
    #Initialize the graph
    nodes = {}
    node_count = random.randint(8,12)
    positions = {}
    for i in range(node_count):
        node_name = chr(65 + i)
        nodes[node_name] = {
            "priority": random.randint(1, 3),
            "danger": round(random.uniform(0, 1), 2),
            "connections": [],
            "fire_extinguished": False,
        }
        positions[node_name] = (random.randint(100, WIDTH - 100), random.randint(100, HEIGHT - 100))

    edges = {}
    for node in nodes:
        connections = random.sample(list(nodes.keys()), random.randint(1, min(3, node_count - 3)))
        connections = [conn for conn in connections if conn != node]  # Avoid self-loops
        nodes[node]["connections"] = connections

        for conn in connections:
            if (node, conn) not in edges and (conn, node) not in edges:
                edges[(node, conn)] = random.randint(1, 10)

    return nodes, edges, positions


def dijkstra(nodes, edges, start, goal, dangerous_edges):
    #Dijkstra's algorithm for shortest path
    pq = []
    heapq.heappush(pq, (0, start))
    distances = {node: float('inf') for node in nodes}
    distances[start] = 0
    previous_nodes = {node: None for node in nodes}

    while pq:
        current_cost, current_node = heapq.heappop(pq)
        if current_node == goal:
            break

        for neighbor in nodes[current_node]["connections"]:
            # Skip dangerous edges
            edge_key = (current_node, neighbor) if (current_node, neighbor) in edges else (neighbor, current_node)
            if edge_key in dangerous_edges or (neighbor, current_node) in dangerous_edges:
                continue  # Avoid dangerous edges

            edge_weight = edges.get(edge_key, float('inf'))
            new_cost = current_cost + edge_weight
            if new_cost < distances[neighbor]:
                distances[neighbor] = new_cost
                previous_nodes[neighbor] = current_node
                heapq.heappush(pq, (new_cost, neighbor))

    path = []
    current = goal
    while current:
        path.append(current)
        current = previous_nodes[current]
    path.reverse()

    return path, distances[goal]


def rtaa_star(nodes, edges, start, goal, danger_threshold):
    #RTAA* algorithm for pathfinding with dynamic updates
    def heuristic(node):
        return random.uniform(0, 1)
    pq = []
    heapq.heappush(pq, (0 + heuristic(start), 0, start))
    visited = set()
    while pq:
        f_cost, g_cost, current_node = heapq.heappop(pq)
        if current_node == goal:
            return g_cost
        visited.add(current_node)
        for neighbor in nodes[current_node]["connections"]:
            if neighbor in visited:
                continue
            edge_key = (current_node, neighbor) if (current_node, neighbor) in edges else (neighbor, current_node)
            edge_weight = edges.get(edge_key, float('inf'))

            if nodes[neighbor]["danger"] > danger_threshold:
                continue
            heapq.heappush(pq, (g_cost + edge_weight + heuristic(neighbor), g_cost + edge_weight, neighbor))
    return float('inf')


def limit_dangerous_edges(nodes, edges):
    #Limit the number of dangerous edges crossing the threshold and ensure they are not adjacent
    dangerous_edges = random.randint(1, len(edges) // 3)
    dangerous_edges_list = []

    for _ in range(dangerous_edges):
        while True:
            edge = random.choice(list(edges.keys()))
            node1, node2 = edge

            if edge not in dangerous_edges_list:
                adjacent = any(
                    (node1 in e or node2 in e) for e in dangerous_edges_list
                )

                if not adjacent:
                    dangerous_edges_list.append(edge)
                    break

    for node in nodes:
        if all((node, neighbor) in dangerous_edges_list or (neighbor, node) in dangerous_edges_list for neighbor in nodes[node]["connections"]):
            # If all edges are dangerous, we need to add one safe edge
            for neighbor in nodes[node]["connections"]:
                if (node, neighbor) not in dangerous_edges_list and (neighbor, node) not in dangerous_edges_list:
                    dangerous_edges_list.remove((node, neighbor))
                    break

    return dangerous_edges_list


def update_graph(nodes, edges, danger_threshold=0.6, max_dangerous_paths=3):
    #Update the graph dynamically and track dangerous paths
    updates = random.randint(5, 8)

    # Update node priorities
    for node, properties in nodes.items():
        properties["priority"] = random.randint(1, 3)

    for _ in range(updates):
        node = random.choice(list(nodes.keys()))
        if not nodes[node]["fire_extinguished"]:
            nodes[node]["danger"] = round(random.uniform(0, 1), 2)

    return


def draw_graph(nodes, edges, positions, robot_position=None, path_highlighted=None, message=None, dangerous_edges=None):
    #Draw the graph with nodes and edges
    screen.fill(BACKGROUND_COLOR)

    if dangerous_edges is None:
        dangerous_edges = []

    for (node1, node2), weight in edges.items():
        color = GREEN
        thickness = 3

        # Check if the edge is in the dangerous edges list
        if (node1, node2) in dangerous_edges or (node2, node1) in dangerous_edges:
            color = RED
            thickness = 6

        if path_highlighted and ((node1, node2) in path_highlighted or (node2, node1) in path_highlighted):
            color = BLUE
            thickness = 6

        pygame.draw.line(screen, color, positions[node1], positions[node2], thickness)

    for node, properties in nodes.items():
        if properties.get("fire_extinguished", False):
            color = GREEN
        else:
            color = None

        if properties["danger"] > 0:
            screen.blit(fire_emoji_image, (positions[node][0] - 15, positions[node][1] - 20))
        else:
            pygame.draw.circle(screen, GREEN, positions[node], 20)

        # Display priority number
        text = font.render(f"{properties['priority']}", True, BLACK)
        screen.blit(text, (positions[node][0] - 20, positions[node][1] - 40))

    # Indicate robot's position
    if robot_position:
        pygame.draw.circle(screen, BLUE, positions[robot_position], 25, 3)

    if message:
        message_text = font.render(message, True, BLACK)
        screen.blit(message_text, (WIDTH // 2 - message_text.get_width() // 2, HEIGHT - 50))

    pygame.display.flip()


# Define the distance function
def distance(point1, point2):
    #Calculate the Euclidean distance between two points
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def move_robot(nodes, edges, positions, start, goal, dangerous_edges=None):
    #Simulate robot movement
    robot_position = start
    path = []

    while any(node["danger"] > 0 for node in nodes.values()):
        fire_nodes = [node for node, properties in nodes.items() if properties["danger"] > 0]
        if not fire_nodes:
            break

        # Sort nodes by priority and then by distance to robot's position
        fire_nodes.sort(key=lambda node: (-nodes[node]["priority"], distance(positions[robot_position], positions[node])))

        # Pick the highest priority and nearest node
        next_node = fire_nodes[0]

        # Calculate path to the next node using RTAA* or Dijkstra
        path, _ = dijkstra(nodes, edges, robot_position, next_node, dangerous_edges)

        for step in path:
            draw_graph(nodes, edges, positions, robot_position=step, path_highlighted=set(zip(path, path[1:])), dangerous_edges=dangerous_edges)
            robot_position = step
            time.sleep(0.5)

        nodes[next_node]["fire_extinguished"] = True
        nodes[next_node]["danger"] = 0

    # Check if all fires have been extinguished and display the message
    draw_graph(nodes, edges, positions, robot_position=robot_position, message="Fire extinguished successfully!")


def periodic_update(nodes, edges, danger_threshold=0.6):
    #Update the graph periodically with new dangerous paths and node changes
    update_count = 0
    max_updates = random.randint(3, 5)

    while update_count < max_updates:
        update_graph(nodes, edges, danger_threshold)
        time.sleep(3)
        update_count += 1


def main():
    nodes, edges, positions = initialize_graph()
    start = list(nodes.keys())[0]
    goal = list(nodes.keys())[-1]

    dangerous_edges = limit_dangerous_edges(nodes, edges)

    # Create the update thread to keep the graph dynamic
    update_thread = threading.Thread(target=periodic_update, args=(nodes, edges))
    update_thread.daemon = True
    update_thread.start()

    # Simulate robot movement and extinguishing fires
    move_robot(nodes, edges, positions, start, goal, dangerous_edges=dangerous_edges)


if __name__ == "__main__":
    main()
