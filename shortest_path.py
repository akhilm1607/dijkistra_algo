import numpy as np

inf = np.inf

# def dijkstra(adjacency_matrix, node_list, start_node, target_node):
def dijkstra(adjacency_matrix, node_list, start_node):
    no_of_nodes = len(adjacency_matrix)
    node_u = node_list.index(start_node)
    visited = [False] * no_of_nodes
    distance = [inf] * no_of_nodes
    parent_node = [-1] * no_of_nodes

    distance[node_u] = 0

    for _ in range(no_of_nodes):
        # Find the vertex with the minimum distance value among unvisited vertices
        min_distance = inf
        min_node = -1
        for v in range(no_of_nodes):
            if not visited[v] and distance[v] < min_distance:
                min_distance = distance[v]
                min_node = v

        if min_node == -1:
            break  # All remaining vertices are unreachable

        visited[min_node] = True
        # Update the distance and parent_node for adjacent vertices
        for v in range(no_of_nodes):
            if not visited[v] and adjacency_matrix[min_node][v] > 0:
                new_distance = distance[min_node] + adjacency_matrix[min_node][v]
                if new_distance < distance[v]:
                    distance[v] = new_distance
                    parent_node[v] = min_node

    # node_v = node_list.index(target_node)
    print("Shortest parent_nodes from vertex", start_node)
    for v in range(no_of_nodes):
        if v != node_list.index(start_node):
            short_path = get_parent_node(v, parent_node, node_list)
            if short_path[0] == node_list[v]:
                short_path = ['There is no path from ' + start_node + ' to ' + node_list[v]]
                print(f"Vertex {node_list[v]}: Distance = {distance[v]}, Path = {short_path[0]}")
            else:
                print(f"Vertex {node_list[v]}: Distance = {distance[v]}, Path = {short_path}")
    # print(f"Vertex {target_node}: Distance = {distance[node_v]}, parent_node = {get_parent_node(node_v, parent_node, node_list)}")

# To reconstruct the parent_node from starting node to all other nodes
def get_parent_node(end, parent_node, node_list):
    result = []
    while end != -1:
        result.insert(0, node_list[end])
        end = parent_node[end]
    return result