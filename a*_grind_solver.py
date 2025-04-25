import json
import heapq
import math
import sys

def euclidean_distance(node1_coords, node2_coords):
    """Calculates the Euclidean distance between two points."""
    return math.sqrt((node1_coords['x_m'] - node2_coords['x_m'])**2 +
                     (node1_coords['y_m'] - node2_coords['y_m'])**2)

def heuristic_time(node_id, target_id, nodes_coords, speed_m_per_s):
    """
    Heuristic function (h) for A*: Estimates the minimum time from node_id to target_id.
    Uses Euclidean distance divided by speed. Assumes speed is positive.
    """
    if node_id not in nodes_coords or target_id not in nodes_coords:
        return float('inf') # Should not happen if nodes exist
    if speed_m_per_s <= 0:
        return float('inf') # Avoid division by zero or negative time

    distance = euclidean_distance(nodes_coords[node_id], nodes_coords[target_id])
    # The heuristic must not overestimate the actual cost.
    # Since actual paths might be longer and might include penalties,
    # the direct distance / speed is a safe lower bound (admissible heuristic).
    return distance / speed_m_per_s

def calculate_fastest_path_astar(data, speed_m_per_s, blockade_penalty_s):
    """
    Calculates the fastest path using the A* algorithm.

    Args:
        data (dict): The JSON object containing nodes and edges.
        speed_m_per_s (float): The travel speed in meters per second.
        blockade_penalty_s (float): The additional time in seconds for
                                     passing an edge with a blockade (state=1).

    Returns:
        dict: A JSON object with the path (nodes and edges) and total time,
              or an error message if no path is found.
    """
    # --- Data Extraction and Initial Checks ---
    try:
        nodes_data = {node['id']: node for node in data['nodes']}
        edges_data = data['edges']
        start_node_id = data['start_id']
        target_node_id = data['target_id']
        # Store coordinates separately for quick lookup in heuristic
        nodes_coords = {node['id']: {'x_m': node['x_m'], 'y_m': node['y_m']}
                        for node in data['nodes'] if 'x_m' in node and 'y_m' in node}

    except KeyError as e:
        return {"error": f"Fehlender Schlüssel im JSON: {e}", "total_time_s": None, "path_nodes": [], "path_edges": []}
    except TypeError as e:
         return {"error": f"Ungültige Datenstruktur im JSON: {e}", "total_time_s": None, "path_nodes": [], "path_edges": []}

    if speed_m_per_s <= 0:
        return {"error": "Geschwindigkeit muss positiv sein.", "total_time_s": None, "path_nodes": [], "path_edges": []}

    # --- Identify Forbidden Nodes ---
    forbidden_node_ids = {node['id'] for node in data['nodes'] if node.get('locked', False)}

    # Check if start or target node is valid and not forbidden
    if start_node_id not in nodes_data:
         return {"error": f"Startknoten ID {start_node_id} nicht in den Knotendaten gefunden.", "total_time_s": None, "path_nodes": [], "path_edges": []}
    if target_node_id not in nodes_data:
         return {"error": f"Zielknoten ID {target_node_id} nicht in den Knotendaten gefunden.", "total_time_s": None, "path_nodes": [], "path_edges": []}
    if start_node_id in forbidden_node_ids:
        return {"error": f"Startknoten {start_node_id} ist gesperrt.", "total_time_s": None, "path_nodes": [], "path_edges": []}
    if target_node_id in forbidden_node_ids:
        return {"error": f"Zielknoten {target_node_id} ist gesperrt.", "total_time_s": None, "path_nodes": [], "path_edges": []}
    if start_node_id not in nodes_coords or target_node_id not in nodes_coords:
         return {"error": f"Koordinaten ('x_m', 'y_m') fehlen für Start- oder Zielknoten.", "total_time_s": None, "path_nodes": [], "path_edges": []}


    # --- Build Graph Representation (Adjacency List) ---
    # Stores neighbors and the *actual time cost* to travel the edge.
    graph = {node_id: [] for node_id in nodes_data if node_id not in forbidden_node_ids}
    edge_details = {} # To look up edge IDs for path reconstruction

    for edge in edges_data:
        try:
            from_node = edge['from']
            to_node = edge['to']
            edge_id = edge['id']
            state = edge.get('state', 0)
            length_m = edge['length_m']

            # Ignore edges connected to forbidden nodes
            if from_node in forbidden_node_ids or to_node in forbidden_node_ids:
                continue
            # Ignore edges with state 2
            if state == 2:
                 continue

            # Calculate actual time cost for this edge
            time_cost = length_m / speed_m_per_s
            if state == 1: # Add penalty for blockades
                time_cost += blockade_penalty_s

            # Add edge to the graph (undirected)
            if from_node in graph:
                graph[from_node].append({'neighbor': to_node, 'time': time_cost, 'edge_id': edge_id})
            if to_node in graph:
                graph[to_node].append({'neighbor': from_node, 'time': time_cost, 'edge_id': edge_id})

            # Store edge details for path reconstruction
            edge_details[(from_node, to_node)] = edge_id
            edge_details[(to_node, from_node)] = edge_id

        except KeyError as e:
            print(f"Warnung: Fehlender Schlüssel in Kante {edge.get('id', 'Unbekannt')}: {e}. Kante wird ignoriert.")
            continue
        except TypeError as e:
            print(f"Warnung: Typfehler in Kante {edge.get('id', 'Unbekannt')}: {e}. Kante wird ignoriert.")
            continue


    # --- A* Algorithm Initialization ---
    # g_score: Actual cost (time) from start node to a given node.
    g_score = {node_id: float('inf') for node_id in graph}
    g_score[start_node_id] = 0

    # f_score: Estimated total cost (time) from start to target through a given node (f = g + h).
    f_score = {node_id: float('inf') for node_id in graph}
    f_score[start_node_id] = heuristic_time(start_node_id, target_node_id, nodes_coords, speed_m_per_s)

    # Priority queue (min-heap). Stores tuples: (f_score_value, node_id)
    # We prioritize nodes with the lowest estimated total cost.
    pq = [(f_score[start_node_id], start_node_id)]

    # Predecessors dictionary to reconstruct the path
    predecessors = {node_id: None for node_id in graph}

    # --- A* Algorithm Main Loop ---
    while pq:
        # Get the node in the priority queue with the lowest f_score
        current_f_score, current_node_id = heapq.heappop(pq)

        # If we reached the target, reconstruct and return the path
        if current_node_id == target_node_id:
            # --- Path Reconstruction --- (Identical to Dijkstra's part)
            path_nodes = []
            path_edges = []
            total_time = g_score[target_node_id] # The final g_score is the actual shortest time

            step = target_node_id
            while step is not None:
                path_nodes.append(step)
                prev_step = predecessors[step]
                if prev_step is not None:
                    edge_id = edge_details.get((prev_step, step))
                    if edge_id is None:
                         print(f"Warnung: Kante zwischen {prev_step} und {step} nicht in edge_details gefunden.")
                         path_edges.append(None)
                    else:
                        path_edges.append(edge_id)
                step = prev_step

            path_nodes.reverse()
            path_edges.reverse()

            node_labels = {node['id']: node.get('label', f"ID_{node['id']}") for node in data['nodes']}
            path_node_labels = [node_labels.get(node_id, f"ID_{node_id}") for node_id in path_nodes]

            return {
                "message": "Schnellster Weg gefunden (A*).",
                "model": "A*",
                "total_time_s": round(total_time, 3),
                "path_node_ids": path_nodes,
                "path_node_labels": path_node_labels,
                "path_edge_ids": path_edges
            }

        # Optimization: If we found a shorter path to `current_node_id` already
        # after this node was added to pq, ignore this path.
        # Note: Comparing f_scores can be tricky with heuristics. Comparing g_scores
        # might be more robust if the heuristic isn't perfectly consistent,
        # but for admissible heuristics, checking the f_score in pq vs calculated f_score works.
        # A simpler check: if current_f_score > f_score[current_node_id]: continue
        # However, standard A* often just processes neighbors. Let's stick to that.


        # Explore neighbors
        if current_node_id in graph:
            for edge_info in graph[current_node_id]:
                neighbor_id = edge_info['neighbor']
                edge_time = edge_info['time'] # Actual time cost of the edge

                # Calculate tentative g_score for the neighbor through the current node
                tentative_g_score = g_score[current_node_id] + edge_time

                # If this path to the neighbor is better than any previous path
                if tentative_g_score < g_score.get(neighbor_id, float('inf')):
                    # This path is better! Record it.
                    predecessors[neighbor_id] = current_node_id
                    g_score[neighbor_id] = tentative_g_score
                    # Calculate f_score for the neighbor
                    h_value = heuristic_time(neighbor_id, target_node_id, nodes_coords, speed_m_per_s)
                    f_score[neighbor_id] = tentative_g_score + h_value

                    # Add neighbor to the priority queue
                    # Check if neighbor is already in pq conceptually; heapq handles duplicates efficiently.
                    heapq.heappush(pq, (f_score[neighbor_id], neighbor_id))

    # --- Target Not Reachable ---
    # If the loop finishes without reaching the target
    return {"error": "Zielknoten ist nicht erreichbar (A*).", "total_time_s": None, "path_nodes": [], "path_edges": []}


# --- Main execution block ---
if __name__ == "__main__":
    # --- Configuration ---
    config_filename = "grid_config.json"
    output_filename = "path_solution.json" # <<< Name der Ausgabedatei geändert
    fahrgeschwindigkeit_mps = 1.0  # Beispiel: 1 Meter pro Sekunde
    zeitstrafe_blockade_s = 5.0   # Beispiel: 5 Sekunden extra Zeit für Blockaden

    # --- Load Grid Data from JSON file ---
    try:
        with open(config_filename, 'r', encoding='utf-8') as f:
            grid_data = json.load(f)
        print(f"Gitterkonfiguration erfolgreich aus '{config_filename}' geladen.")
    except FileNotFoundError:
        print(f"Fehler: Die Konfigurationsdatei '{config_filename}' wurde nicht gefunden.")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Fehler: Die Konfigurationsdatei '{config_filename}' enthält ungültiges JSON: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Ein unerwarteter Fehler ist beim Lesen der Datei '{config_filename}' aufgetreten: {e}")
        sys.exit(1)


    # --- Perform Calculation using A* ---
    print("\nBerechne Pfad mit A*...")
    result_path = calculate_fastest_path_astar(grid_data, fahrgeschwindigkeit_mps, zeitstrafe_blockade_s)
    print("Berechnung abgeschlossen.")

    # --- Output Result to Console (optional) ---
    print("\n--- Ergebnis der Pfadfindung (Konsole) ---")
    print(json.dumps(result_path, indent=2, ensure_ascii=False))

    # --- Save Result to JSON file ---
    try:
        with open(output_filename, 'w', encoding='utf-8') as outfile:
            json.dump(result_path, outfile, indent=2, ensure_ascii=False)
        print(f"\nErgebnis erfolgreich in '{output_filename}' gespeichert.")
    except IOError as e:
        print(f"\nFehler: Das Ergebnis konnte nicht in '{output_filename}' geschrieben werden: {e}")
    except Exception as e:
        print(f"\nEin unerwarteter Fehler ist beim Schreiben der Datei '{output_filename}' aufgetreten: {e}")


    # --- Example of how to use the result (from the dictionary) ---
    if "error" not in result_path:
        print("\n--- Pfad Details (aus Ergebnis-Dictionary) ---")
        print(f"Gesamtzeit: {result_path['total_time_s']} Sekunden")
        node_label_path = " -> ".join(map(str, result_path['path_node_labels']))
        print(f"Knoten (Labels): {node_label_path}")
        print(f"Knoten (IDs): {result_path['path_node_ids']}")
        print(f"Kanten (IDs): {result_path['path_edge_ids']}")

