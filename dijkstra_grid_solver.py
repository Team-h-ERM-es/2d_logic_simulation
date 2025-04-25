import json
import heapq
import math
import sys # Import sys to handle script exit on error

def calculate_fastest_path(data, speed_m_per_s, blockade_penalty_s):
    """
    Calculates the fastest path using Dijkstra's algorithm.

    Args:
        data (dict): The JSON object containing nodes and edges.
        speed_m_per_s (float): The travel speed in meters per second.
        blockade_penalty_s (float): The additional time in seconds for
                                     passing an edge with a blockade (state=1).

    Returns:
        dict: A JSON object with the path (nodes and edges) and total time,
              or an error message if no path is found.
    """
    # --- Data Extraction ---
    try:
        nodes_data = {node['id']: node for node in data['nodes']}
        edges_data = data['edges']
        start_node_id = data['start_id']
        target_node_id = data['target_id']
    except KeyError as e:
        return {"error": f"Fehlender Schlüssel im JSON: {e}", "total_time_s": None, "path_nodes": [], "path_edges": []}
    except TypeError as e:
         return {"error": f"Ungültige Datenstruktur im JSON: {e}", "total_time_s": None, "path_nodes": [], "path_edges": []}


    # --- Identify Forbidden Nodes ---
    # Nodes marked as 'locked' cannot be traversed.
    forbidden_node_ids = {node['id'] for node in data['nodes'] if node.get('locked', False)}

    # Check if start or target node is forbidden or doesn't exist
    if start_node_id not in nodes_data:
         return {"error": f"Startknoten ID {start_node_id} nicht in den Knotendaten gefunden.", "total_time_s": None, "path_nodes": [], "path_edges": []}
    if target_node_id not in nodes_data:
         return {"error": f"Zielknoten ID {target_node_id} nicht in den Knotendaten gefunden.", "total_time_s": None, "path_nodes": [], "path_edges": []}
    if start_node_id in forbidden_node_ids:
        return {"error": f"Startknoten {start_node_id} ist gesperrt.", "total_time_s": None, "path_nodes": [], "path_edges": []}
    if target_node_id in forbidden_node_ids:
        return {"error": f"Zielknoten {target_node_id} ist gesperrt.", "total_time_s": None, "path_nodes": [], "path_edges": []}


    # --- Build Graph for Dijkstra ---
    # Use time as edge weight.
    # The graph is represented as an adjacency list: {node_id: [neighbor_info, ...]}
    # neighbor_info = {'neighbor': neighbor_id, 'time': time_cost, 'edge_id': edge_id}
    graph = {node_id: [] for node_id in nodes_data if node_id not in forbidden_node_ids}
    edge_details = {} # To look up edge IDs later for path reconstruction

    for edge in edges_data:
        try:
            from_node = edge['from']
            to_node = edge['to']
            edge_id = edge['id']
            state = edge.get('state', 0) # Default state is 0 if missing
            length_m = edge['length_m']

            # Ignore edges connected to forbidden nodes
            if from_node in forbidden_node_ids or to_node in forbidden_node_ids:
                continue

            # Ignore edges with state 1 ("nicht entfernte Linien" / non-removable lines)
            if state == 2:
                 continue

            # Calculate time cost for this edge
            if speed_m_per_s <= 0:
                 return {"error": "Geschwindigkeit muss positiv sein.", "total_time_s": None, "path_nodes": [], "path_edges": []}
            time_cost = length_m / speed_m_per_s
            if state == 1: # Add penalty for blockades
                time_cost += blockade_penalty_s

            # Add edge to the graph (undirected)
            # Ensure nodes exist in our filtered graph before adding edges
            if from_node in graph:
                graph[from_node].append({'neighbor': to_node, 'time': time_cost, 'edge_id': edge_id})
            if to_node in graph:
                graph[to_node].append({'neighbor': from_node, 'time': time_cost, 'edge_id': edge_id})

            # Store edge details for path reconstruction (store for both directions)
            edge_details[(from_node, to_node)] = edge_id
            edge_details[(to_node, from_node)] = edge_id

        except KeyError as e:
            print(f"Warnung: Fehlender Schlüssel in Kante {edge.get('id', 'Unbekannt')}: {e}. Kante wird ignoriert.")
            continue
        except TypeError as e:
            print(f"Warnung: Typfehler in Kante {edge.get('id', 'Unbekannt')}: {e}. Kante wird ignoriert.")
            continue


    # --- Dijkstra Algorithm ---
    # Initialize times (infinity, except for start node)
    times = {node_id: float('inf') for node_id in graph}
    times[start_node_id] = 0

    # Initialize predecessors (to reconstruct the path)
    predecessors = {node_id: None for node_id in graph}

    # Priority queue (min-heap)
    # Stores tuples: (current_time_to_node, node_id)
    pq = [(0, start_node_id)]

    while pq:
        current_time, current_node_id = heapq.heappop(pq)

        # If we pull a node from PQ that already has a shorter path found, skip it.
        if current_time > times[current_node_id]:
            continue

        # Target reached?
        if current_node_id == target_node_id:
            break # Found the shortest path to the target

        # Explore neighbors
        if current_node_id in graph: # Ensure the node is in the processed graph
            for edge_info in graph[current_node_id]:
                neighbor_id = edge_info['neighbor']
                edge_time = edge_info['time']

                # Check if neighbor exists in times dict (it might be a locked node removed earlier)
                if neighbor_id not in times:
                    continue

                new_time = current_time + edge_time

                # Relaxation: If the new path is shorter
                if new_time < times[neighbor_id]:
                    times[neighbor_id] = new_time
                    predecessors[neighbor_id] = current_node_id
                    heapq.heappush(pq, (new_time, neighbor_id))

    # --- Path Reconstruction ---
    path_nodes = []
    path_edges = []
    total_time = times.get(target_node_id, float('inf'))

    if total_time == float('inf'):
        # Check if target node was initially valid but became unreachable
        if target_node_id in nodes_data and target_node_id not in forbidden_node_ids:
             return {"error": "Zielknoten ist nicht erreichbar (möglicherweise durch gesperrte Knoten/Linien isoliert).", "total_time_s": None, "path_nodes": [], "path_edges": []}
        else:
             # This case should be caught earlier, but as a fallback:
             return {"error": "Zielknoten ist ungültig oder gesperrt.", "total_time_s": None, "path_nodes": [], "path_edges": []}

    else:
        # Backtrack from target to start
        step = target_node_id
        while step is not None:
            path_nodes.append(step)
            prev_step = predecessors[step]
            if prev_step is not None:
                # Find the edge ID connecting these two nodes
                edge_id = edge_details.get((prev_step, step))
                if edge_id is None:
                     # This shouldn't happen if edge_details was populated correctly
                     print(f"Warnung: Kante zwischen {prev_step} und {step} nicht in edge_details gefunden.")
                     path_edges.append(None) # Add a placeholder
                else:
                    path_edges.append(edge_id)
            step = prev_step

        # Reverse to get the path from start to target
        path_nodes.reverse()
        path_edges.reverse()

        # Optional: Get node labels for better readability
        node_labels = {node['id']: node.get('label', f"ID_{node['id']}") for node in data['nodes']}
        path_node_labels = [node_labels.get(node_id, f"ID_{node_id}") for node_id in path_nodes]


        return {
            "message": "Schnellster Weg gefunden.",
            "model": "Dijkstra",
            "total_time_s": round(total_time, 3),
            "path_node_ids": path_nodes,
            "path_node_labels": path_node_labels, # Additional info with labels
            "path_edge_ids": path_edges
        }

# --- Main execution block ---
if __name__ == "__main__":
    # --- Configuration ---
    config_filename = "grid_config.json"
    output_filename = "path_solution.json" # <<< Name der Ausgabedatei
    fahrgeschwindigkeit_mps = 1.0  # Beispiel: 1 Meter pro Sekunde
    zeitstrafe_blockade_s = 5.0   # Beispiel: 5 Sekunden extra Zeit für Blockaden

    # --- Load Grid Data from JSON file ---
    try:
        with open(config_filename, 'r', encoding='utf-8') as f:
            grid_data = json.load(f)
        print(f"Gitterkonfiguration erfolgreich aus '{config_filename}' geladen.")
    except FileNotFoundError:
        print(f"Fehler: Die Konfigurationsdatei '{config_filename}' wurde nicht gefunden.")
        sys.exit(1) # Exit script if config is missing
    except json.JSONDecodeError as e:
        print(f"Fehler: Die Konfigurationsdatei '{config_filename}' enthält ungültiges JSON: {e}")
        sys.exit(1) # Exit script if JSON is invalid
    except Exception as e:
        print(f"Ein unerwarteter Fehler ist beim Lesen der Datei '{config_filename}' aufgetreten: {e}")
        sys.exit(1)


    # --- Perform Calculation ---
    result_path = calculate_fastest_path(grid_data, fahrgeschwindigkeit_mps, zeitstrafe_blockade_s)

    # --- Output Result to Console (optional) ---
    print("\n--- Ergebnis der Pfadfindung (Konsole) ---")
    print(json.dumps(result_path, indent=2, ensure_ascii=False))

    # --- Save Result to JSON file ---
    try:
        with open(output_filename, 'w', encoding='utf-8') as outfile:
            # Use json.dump to write the dictionary directly to the file
            # indent=2 for pretty printing, ensure_ascii=False for correct special characters
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
        # Ensure labels are converted to strings before joining
        node_label_path = " -> ".join(map(str, result_path['path_node_labels']))
        print(f"Knoten (Labels): {node_label_path}")
        print(f"Knoten (IDs): {result_path['path_node_ids']}")
        print(f"Kanten (IDs): {result_path['path_edge_ids']}")

