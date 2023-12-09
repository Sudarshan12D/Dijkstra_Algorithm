import threading
import socket
import time
import json
import sys

INFINITY = 999
BROADCAST_COUNTER = 5

class Router:
    
    # Initializes the router with ID, port, and configuration settings.
    def __init__(self, router_id, port, config_file):
        self.router_id = router_id
        self.port = port
        self.neighbors = {}
        self.total_nodes = 0
        self.cost_map = []
        self.load_config(config_file)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(("localhost", self.port))
        print("Connection established. waiting....")


    # Loads router configuration from the specified file.
    def load_config(self, config_file):
        with open(config_file, 'r') as f:
            self.total_nodes = int(f.readline().strip())
            self.cost_map = [[INFINITY for _ in range(self.total_nodes)] for _ in range(self.total_nodes)]
            for i in range(self.total_nodes):
                self.cost_map[i][i] = 0
            for line in f:
                parts = line.strip().split()  
                if parts:  # Checks if line is not empty
                    if len(parts) == 4:
                        neighbor_label, neighbor_id, cost, neighbor_port = parts
                        neighbor_id = int(neighbor_id)
                        cost = int(cost)
                        neighbor_port = int(neighbor_port)
                        self.neighbors[neighbor_id] = (cost, neighbor_port)
                        self.cost_map[self.router_id][neighbor_id] = cost
                    else:
                        raise ValueError(f"Invalid line in config file: {line.strip()}")
            
                
    # Sends link state information to all neighboring routers.
    def send_link_state_info(self):
        while True:
            for neighbor_id, (cost, neighbor_port) in self.neighbors.items():
                message = json.dumps(self.cost_map[self.router_id])
                self.udp_socket.sendto(message.encode(), ("localhost", neighbor_port))
            time.sleep(1)


    # Receives link state information from neighbors and updates routing info.
    def receive_link_state_info(self):
        while True:
            try:
                message, addr = self.udp_socket.recvfrom(1024)
                neighbor_state = json.loads(message.decode())
                neighbor_id = self.get_neighbor_id_by_port(addr[1])
                if neighbor_id is not None:
                    self.cost_map[neighbor_id] = neighbor_state
                    self.broadcast_link_state(neighbor_state, BROADCAST_COUNTER, addr[1])
            except socket.timeout:
                continue
            except ConnectionResetError:
                break


    # Returns the neighbor ID corresponding to the given port number.
    def get_neighbor_id_by_port(self, port):
        for neighbor_id, (cost, neighbor_port) in self.neighbors.items():
            if neighbor_port == port:
                return neighbor_id
        return None
    
    
    # Broadcasts link state information to all neighbors except the source.
    def broadcast_link_state(self, link_state_info, counter, source_port):
        if counter > 0:
            for neighbor_id, (cost, neighbor_port) in self.neighbors.items():
                if neighbor_port != source_port:
                    message = json.dumps(link_state_info)
                    self.udp_socket.sendto(message.encode(), ("localhost", neighbor_port))


    # Regularly computes the shortest paths to all nodes using Dijkstra's algorithm.
    def compute_paths(self):
        while True:
            time.sleep(10)
            self.dijkstra()


    # Runs Dijkstra's algorithm and prints the routing information.
    def dijkstra(self):
        distance, previous = self.run_dijkstra(self.router_id, self.cost_map)
        self.print_routing_info(distance, previous)


    # Implements Dijkstra's algorithm to find the shortest path from the source.
    def run_dijkstra(self, source, cost_map):
        num_nodes = len(cost_map)
        visited = [False] * num_nodes
        distance = [INFINITY] * num_nodes
        previous = [source] * num_nodes 
        distance[source] = 0

        for _ in range(num_nodes):
            min_distance = INFINITY
            min_index = -1
            for i in range(num_nodes):
                if not visited[i] and distance[i] < min_distance:
                    min_distance = distance[i]
                    min_index = i
            if min_index == -1:
                break
            visited[min_index] = True

            for i in range(num_nodes):
                if not visited[i] and cost_map[min_index][i] != INFINITY and distance[min_index] + cost_map[min_index][i] < distance[i]:
                    distance[i] = distance[min_index] + cost_map[min_index][i]
                    previous[i] = min_index

        return distance, previous


    # Prints the results of Dijkstra's algorithm and the forwarding table.
    def print_routing_info(self, distance, previous):
        
        print(f"Dijkstra's algorithm for Router {self.router_id}:")
        print("Destination_RouterID | Distance | Previous_nodeID")
        for i in range(len(distance)):
            prev_node_id = previous[i] if previous[i] is not None else 0 
            print(f"          {i}          |\t  {distance[i]}\t|        {prev_node_id}")

        print("\nThe Forwarding table in B is printed as follows:", self.router_id)
        print("Destination_RouterID | Next_hop_routerlabel")
        for i in range(len(distance)):
            if i != self.router_id:
                if distance[i] != INFINITY:
                    next_hop_router_label = chr(65 + i)
                else:
                    next_hop_router_label = "N/A"  #print N/A if routers are not neighbours 
                print(f"          {i}          |\t    {next_hop_router_label}")


    # Get the next hop on the path to the destination
    def get_next_hop(self, destination, previous):
        
        if previous[destination] is None:
            return None
        next_hop = destination
        while previous[next_hop] != self.router_id:
            next_hop = previous[next_hop]
            if next_hop is None:
                return None
        return next_hop


if __name__ == "__main__":
    
    if len(sys.argv) != 4:
        print("Usage: python Router.py <router_id> <router_port> <config_file>")
        sys.exit(1)

    router_id = int(sys.argv[1])
    port = int(sys.argv[2])
    config_file = sys.argv[3]

    router = Router(router_id, port, config_file)

    threading.Thread(target=router.send_link_state_info, daemon=True).start()
    threading.Thread(target=router.receive_link_state_info, daemon=True).start()
    threading.Thread(target=router.compute_paths, daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("KeyBoard Interuppted. Router is shutting down...")
        router.udp_socket.close()
        sys.exit(0)
