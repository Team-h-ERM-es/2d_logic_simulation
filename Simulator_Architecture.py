import pygame
import json # Import JSON library
import os   # Import OS library for file existence check
from math import hypot
# --- Import the solver function ---
# Correct function name and import
try:
    from dijkstra_grid_solver import calculate_fastest_path
except ImportError:
    print("WARNUNG: dijkstra_grid_solver.py nicht gefunden oder 'calculate_fastest_path' Funktion fehlt.")
    print("Die 'Solve Path' Funktionalität wird nicht verfügbar sein.")
    calculate_fastest_path = None # Define it as None so checks later don't crash

"""
PREN Wegnetz‑Simulator – Constraint Dragging, Toggle Lock, Target Selection, JSON Load/Save & Path Display
----------------------------------------------------------------------------------------------------------
Version: 8.1 (UI Buttons & Dijkstra Integration)

Features:
1. Slider‑Bedienung
2. Live‑Rescale
3. Constraint Dragging (Kantenlänge 0.5m - 2.0m)
4. Toggle Lock (Knoten sperren/entsperren per Klick)
5. Target Selection (A, B, C, N Tasten)
6. Config Menu Toggle ('M' Taste)
7. JSON Grid Export ('S' Taste): Speichert Grid-Zustand in Konsole & Datei (grid_config.json).
8. JSON Grid Import ('L' Taste): Lädt Grid-Zustand aus Datei (grid_config.json).
9. JSON Path Solution Import ('P' Taste): Lädt Pfadlösung aus Datei (path_solution.json).
10. Path Display Toggle ('H' Taste): Zeigt/Verbirgt den geladenen Pfad (grüne Linie) und die Zeit.
"""

# ---------- Konstanten ----------
SCREEN_SIZE = (1000, 820)
FPS = 60
NODE_RADIUS = 18
EDGE_COLORS = {0: (200, 200, 200), 1: (200, 60, 60), 2: (80, 80, 80)} # Normal, Hindernis, Deaktiviert
BG_COLOR = (30, 30, 30)
LOCK_COLOR = (200, 60, 60) # Farbe für gesperrte Knoten
START_COLOR = (60, 200, 60) # Farbe für Startknoten
TARGET_BORDER_COLOR = (255, 255, 0) # Gelb für Zielknoten-Rand
NODE_BORDER_COLOR = (255, 255, 255) # Standard Randfarbe (Weiss)
NUM_COLOR = (0, 0, 0) # Farbe für Knotennummern
INFO_COLOR = (255, 255, 255) # Farbe für Textinfos (Zeit)
PATH_COLOR = (0, 255, 0) # Grün für Lösungspfad
PATH_WIDTH = 5 # Breite der Lösungspfadlinie
FONT_SIZE = 14
INFO_FONT_SIZE = 18 # Grössere Schrift für Zeit
EDGE_CLICK_TOL = 8 # Toleranz für Klick auf Kante
DEFAULT_SCALE = 110 # Standard-Skalierungsfaktor (Pixel pro Meter)
DEFAULT_DRAG_THRESHOLD = 5 # Pixel-Distanz, um Klick von Ziehen zu unterscheiden
LABEL = lambda idx: str(idx + 1) # Funktion zur Beschriftung der Knoten (beginnend bei 1)
MIN_EDGE_LEN = 0.5 # Minimale Kantenlänge (Meter)
MAX_EDGE_LEN = 2.0 # Maximale Kantenlänge (Meter)
CONSTRAINT_VIOLATION_COLOR = (255, 165, 0) # Orange für verletzte Constraints (visuelles Feedback)
DEFAULT_CONFIG_FILENAME = "grid_config.json" # Standard-Dateiname für Grid-Konfig
PATH_SOLUTION_FILENAME = "path_solution.json" # Standard-Dateiname für Pfadlösung

# --- Path Solver Parameters ---
SPEED_MPS = 1.0            # Speed in meters per second for Dijkstra
BLOCKADE_PENALTY_S = 5.0   # Time penalty in seconds for state=1 edges

# Zielknoten-Mapping (Taste -> Knoten-Index)
TARGET_MAP = {
    pygame.K_a: 5, # Taste 'A' -> Knoten 6 (Index 5)
    pygame.K_b: 6, # Taste 'B' -> Knoten 7 (Index 6)
    pygame.K_c: 7, # Taste 'C' -> Knoten 8 (Index 7)
}

# ---------- Hilfsfunktionen ----------

def pld(px, py, x1, y1, x2, y2):
    """ Berechnet Distanz Punkt zu Liniensegment. """
    if (x1, y1) == (x2, y2):
        return hypot(px - x1, py - y1)
    den = (x2 - x1)**2 + (y2 - y1)**2
    if den == 0:
         return hypot(px - x1, py - y1)
    t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / den
    t = max(0, min(1, t))
    projx = x1 + t * (x2 - x1)
    projy = y1 + t * (y2 - y1)
    return hypot(px - projx, py - projy)

# ---------- Data classes ----------
class Node:
    """ Repräsentiert einen Knotenpunkt. """
    def __init__(self, idx, logic):
        self.id = idx
        self.logic = list(logic) # [mx, my]
        self.pos = [0, 0] # [px, py]
        self.locked = False
        self.is_start = (idx == 0)
        self.is_target = False
        self.connected_edges = []

    def update_xy(self, cx, cy, scale):
        """ Berechnet Bildschirmkoordinaten. """
        mx, my = self.logic
        self.pos = [int(cx + mx * scale), int(cy + my * scale)]

    def draw(self, surf, font):
        """ Zeichnet den Knoten. """
        color = START_COLOR if self.is_start else BG_COLOR
        if self.locked: color = LOCK_COLOR
        pygame.draw.circle(surf, color, self.pos, NODE_RADIUS)
        border_color = TARGET_BORDER_COLOR if self.is_target else NODE_BORDER_COLOR
        pygame.draw.circle(surf, border_color, self.pos, NODE_RADIUS, 2)
        label_render = font.render(LABEL(self.id), True, NUM_COLOR)
        label_rect = label_render.get_rect(center=self.pos)
        surf.blit(label_render, label_rect)

    def to_dict(self):
        """ Konvertiert Knoten in Dictionary für JSON. """
        return {
            "id": self.id,
            "label": int(LABEL(self.id)),
            "x_px": self.pos[0],
            "y_px": self.pos[1],
            "x_m": round(self.logic[0], 3),
            "y_m": round(self.logic[1], 3),
            "locked": self.locked,
            "is_start": self.is_start,
            "is_target": self.is_target
        }

class Edge:
    """ Repräsentiert eine Kante. """
    def __init__(self, idx, a, b):
        self.id = idx # Kanten-ID (1-basiert)
        self.n_from = a
        self.n_to = b
        self.state = 0 # 0=Normal, 1=Hindernis, 2=Deaktiviert

    def cycle(self):
        """ Wechselt Kantenzustand. """
        self.state = (self.state + 1) % 3

    def get_length_px(self, nodes):
        """ Berechnet Länge in Pixeln. """
        if self.n_from not in nodes or self.n_to not in nodes: return 0
        node_a = nodes[self.n_from]
        node_b = nodes[self.n_to]
        return hypot(node_a.pos[0] - node_b.pos[0], node_a.pos[1] - node_b.pos[1])

    def get_length_m(self, nodes):
        """ Berechnet Länge in Metern. """
        if self.n_from not in nodes or self.n_to not in nodes: return 0
        node_a = nodes[self.n_from]
        node_b = nodes[self.n_to]
        return hypot(node_a.logic[0] - node_b.logic[0], node_a.logic[1] - node_b.logic[1])

    def draw(self, surf, nodes, font):
        """ Zeichnet die Kante (normal). """
        if self.n_from not in nodes or self.n_to not in nodes: return
        node_a = nodes[self.n_from]
        node_b = nodes[self.n_to]
        p1, p2 = node_a.pos, node_b.pos

        dist_m = self.get_length_m(nodes)
        color = EDGE_COLORS[self.state]
        width = 4

        # Visuelles Feedback für ungültige Längen
        if not (MIN_EDGE_LEN <= dist_m <= MAX_EDGE_LEN) and self.state != 2:
             color = CONSTRAINT_VIOLATION_COLOR
             width = 2

        # Zeichnen basierend auf Zustand
        if self.state == 2:
            self._dash(surf, p1, p2, color, width)
        else:
            pygame.draw.line(surf, color, p1, p2, width)

    def _dash(self, surf, p1, p2, color, width, d=10):
        """ Zeichnet gestrichelte Linie. """
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        L = hypot(dx, dy)
        if L == 0: return
        vx, vy = dx / L, dy / L
        for i in range(0, int(L // d), 2):
            s = (int(p1[0] + vx * d * i), int(p1[1] + vy * d * i))
            e = (int(p1[0] + vx * d * (i + 1)), int(p1[1] + vy * d * (i + 1)))
            pygame.draw.line(surf, color, s, e, width)

    def to_dict(self, nodes):
        """ Konvertiert Kante in Dictionary für JSON. """
        length_px = self.get_length_px(nodes)
        length_m = self.get_length_m(nodes)
        return {
            "id": self.id,
            "from": self.n_from,
            "to": self.n_to,
            "state": self.state,
            "length_px": round(length_px, 1),
            "length_m": round(length_m, 2)
        }

# ---------- Slider UI ----------
class Slider:
    """ Ein einfacher Schieberegler. """
    def __init__(self, rect, min_v, max_v, val, label, step=1):
        self.rect = pygame.Rect(rect)
        self.min = min_v
        self.max = max_v
        self.val = val
        self.label = label
        self.step = step
        self.drag = False

    def _set_from_x(self, x):
        """ Setzt Wert basierend auf X-Koordinate. """
        if self.rect.w == 0: return
        pct = max(0, min(1, (x - self.rect.x) / self.rect.w))
        raw_val = self.min + pct * (self.max - self.min)
        if self.step == 0:
            self.val = raw_val
        else:
            diff = max(0, raw_val - self.min)
            self.val = self.min + round(diff / self.step) * self.step
        self.val = max(self.min, min(self.max, self.val))

    def handle(self, ev):
        """ Verarbeitet Events für den Slider. """
        if ev.type == pygame.MOUSEBUTTONDOWN and self.rect.collidepoint(ev.pos):
            self.drag = True; self._set_from_x(ev.pos[0])
        elif ev.type == pygame.MOUSEBUTTONUP:
            self.drag = False
        elif ev.type == pygame.MOUSEMOTION and self.drag:
            self._set_from_x(ev.pos[0])

    def draw(self, surf, font):
        """ Zeichnet den Slider. """
        pygame.draw.rect(surf, (70, 70, 70), self.rect)
        range_val = self.max - self.min
        pct = (self.val - self.min) / range_val if range_val != 0 else 0
        knob_x = int(self.rect.x + pct * self.rect.w)
        pygame.draw.circle(surf, (200, 200, 200), (knob_x, self.rect.centery), 8)
        value_str = f"{self.val:.0f}" if isinstance(self.val, (int, float)) and self.val == int(self.val) else f"{self.val:.1f}"
        label_render = font.render(f"{self.label}: {value_str}", True, (255, 255, 255))
        surf.blit(label_render, (self.rect.x, self.rect.y - 22))

class ConfigMenu:
    """ Konfigurationsmenü. """
    def __init__(self, sim):
        self.sim = sim
        self.show = False
        self.panel = pygame.Surface((340, 200)); self.panel.set_alpha(230)
        self.scale_slider = Slider((50, 70, 240, 10), 50, 160, sim.scale, "Scale", 5)
        self.drag_slider = Slider((50, 140, 240, 10), 1, 20, sim.drag_thr, "Drag Threshold", 1)

    def handle(self, ev):
        """ Verarbeitet Events für das Menü. """
        if not self.show: return
        prev_scale = self.scale_slider.val
        self.scale_slider.handle(ev); self.drag_slider.handle(ev)
        self.sim.drag_thr = int(self.drag_slider.val)
        if self.scale_slider.val != prev_scale:
            self.sim.update_scale(self.scale_slider.val)

    def draw(self, screen, font):
        """ Zeichnet das Menü. """
        if not self.show: return
        self.panel.fill((20, 20, 20))
        self.scale_slider.draw(self.panel, font)
        self.drag_slider.draw(self.panel, font)
        screen.blit(self.panel, (20, 20))

    def update_sliders(self):
        """ Aktualisiert Slider-Werte. """
        self.scale_slider.val = self.sim.scale
        self.drag_slider.val = self.sim.drag_thr

# ---------- Button UI ----------
class Button:
    """ A clickable button with text. """
    def __init__(self, rect, text, action_id, font,
                 color=(70, 70, 70), hover_color=(100, 100, 100),
                 disabled_color=(40, 40, 40), text_color=(255, 255, 255),
                 disabled_text_color=(100, 100, 100)):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.action_id = action_id # Identifier for the action this button triggers
        self.font = font
        self.colors = {
            "normal": color,
            "hover": hover_color,
            "disabled": disabled_color
        }
        self.text_colors = {
            "normal": text_color,
            "disabled": disabled_text_color
        }
        self.is_hovered = False
        self.is_disabled = False # New state for disabling

    def draw(self, surf):
        """ Draws the button. """
        current_color = self.colors["disabled"] if self.is_disabled else (self.colors["hover"] if self.is_hovered else self.colors["normal"])
        current_text_color = self.text_colors["disabled"] if self.is_disabled else self.text_colors["normal"]

        pygame.draw.rect(surf, current_color, self.rect, border_radius=5)
        label_render = self.font.render(self.text, True, current_text_color)
        label_rect = label_render.get_rect(center=self.rect.center)
        surf.blit(label_render, label_rect)

    def handle_event(self, event):
        """ Handles mouse events for the button. Returns action_id if clicked. """
        if self.is_disabled:
            self.is_hovered = False
            return None # Ignore events if disabled

        if event.type == pygame.MOUSEMOTION:
            self.is_hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1 and self.is_hovered:
                return self.action_id # Signal that the button was clicked
        return None # No action triggered

# ---------- Simulator ----------
class Simulator:
    """ Hauptklasse für die Simulation. """
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("PREN Simulator – v8.1 UI Buttons & Dijkstra")
        self.scr = pygame.display.set_mode(SCREEN_SIZE)
        self.font = pygame.font.SysFont("arial", FONT_SIZE)
        self.info_font = pygame.font.SysFont("arial", INFO_FONT_SIZE, bold=True)
        self.button_font = pygame.font.SysFont("arial", 12) # Font for buttons
        self.clock = pygame.time.Clock()
        self.scale = DEFAULT_SCALE
        self.drag_thr = DEFAULT_DRAG_THRESHOLD
        self.cx = SCREEN_SIZE[0] // 2
        # Adjusted cy to make space for buttons at bottom and potentially top
        self.cy = SCREEN_SIZE[1] // 2 - 50 # Center grid more vertically
        self.target_node_id = None

        # Grid initialisieren
        self._initialize_grid()

        # Dragging-Status
        self.dragging = None
        self.drag_start_pos = (0, 0)
        self.node_start_pos = (0, 0)
        self.node_initial_logic = (0,0)

        # Pfadlösung-Status
        self.solution_path_nodes = None
        self.solution_path_edges = None
        self.solution_time = None
        self.solution_model = None # Now stores 'Dijkstra' or 'Loaded' etc.
        self.show_solution_path = False

        self.menu = ConfigMenu(self)

        # --- Button Initialization ---
        self.buttons = []
        self._create_buttons() # Call the updated method

        self.update_scale(self.scale) # Initiale Positionierung

    def _create_buttons(self):
        """ Creates and positions the UI buttons with the new layout. """
        self.buttons.clear()
        button_h = 28
        button_w = 90
        padding = 8
        v_padding = 5 # Vertical padding for stacked buttons

        # --- Top-Left: Target Buttons (Vertical) ---
        target_actions = [
            ("Target A", "TARGET_A"), ("Target B", "TARGET_B"),
            ("Target C", "TARGET_C"), ("No Target", "TARGET_N")
        ]
        x_pos = padding
        y_pos = padding
        for text, action_id in target_actions:
            rect = (x_pos, y_pos, button_w, button_h)
            self.buttons.append(Button(rect, text, action_id, self.button_font))
            y_pos += button_h + v_padding

        # --- Below Targets: Path Buttons ---
        y_pos += padding # Add extra space
        path_actions = [
             ("Solve Path (H)", "SOLVE_PATH"), # Changed from TOGGLE_PATH
             ("Load Path (P)", "LOAD_PATH")
        ]
        solve_button_w = button_w + 20 # Make Solve button wider
        for text, action_id in path_actions:
             rect = (x_pos, y_pos, solve_button_w, button_h)
             self.buttons.append(Button(rect, text, action_id, self.button_font))
             y_pos += button_h + v_padding


        # --- Bottom-Right: Utility Buttons (Horizontal) ---
        utility_actions = [("Save Grid (S)", "SAVE_GRID"), ("Load Grid (L)", "LOAD_GRID"), ("Menu (M)", "TOGGLE_MENU")]
        total_utility_width = len(utility_actions) * button_w + (len(utility_actions) - 1) * padding
        x_pos = SCREEN_SIZE[0] - total_utility_width - padding
        y_pos = SCREEN_SIZE[1] - button_h - padding
        for text, action_id in utility_actions:
             rect = (x_pos, y_pos, button_w, button_h)
             self.buttons.append(Button(rect, text, action_id, self.button_font))
             x_pos += button_w + padding

    def _initialize_grid(self):
        """ Setzt Grid auf Standardzustand zurück. """
        self._logic_coords = {
            0: (0, 0), 1: (-1.2, 1.0), 2: (0, 1.0), 3: (0, 2.0),
            4: (1.2, 1.0), 5: (-1.4, 2.0), 6: (0, 3.0), 7: (1.4, 2.0)
        }
        self.nodes = {i: Node(i, logic) for i, logic in self._logic_coords.items()}
        self.edges = {
            idx: Edge(idx, a, b) for idx, (a, b) in enumerate([
                (0, 1), (0, 2), (0, 4), (1, 2), (2, 4), (1, 5), (2, 5),
                (2, 3), (3, 4), (4, 7), (3, 5), (3, 6), (3, 7), (5, 6),
                (6, 7)
            ], 1)
        }
        # Kanten mit Knoten verbinden
        for edge in self.edges.values():
            if edge.n_from in self.nodes and edge.n_to in self.nodes:
                self.nodes[edge.n_from].connected_edges.append(edge)
                self.nodes[edge.n_to].connected_edges.append(edge)
        self.target_node_id = None

    def update_scale(self, new_scale):
        """ Aktualisiert Skalierung und Knotenpositionen. """
        self.scale = new_scale
        for n in self.nodes.values():
            n.update_xy(self.cx, self.cy, self.scale)

    def _node_at(self, pos):
        """ Findet Knoten an Position. """
        for n in self.nodes.values():
            if hypot(pos[0] - n.pos[0], pos[1] - n.pos[1]) <= NODE_RADIUS:
                return n
        return None

    def _edge_at(self, pos):
        """ Findet Kante an Position. """
        for e in self.edges.values():
            if e.n_from not in self.nodes or e.n_to not in self.nodes: continue
            p1 = self.nodes[e.n_from].pos
            p2 = self.nodes[e.n_to].pos
            if pld(pos[0], pos[1], p1[0], p1[1], p2[0], p2[1]) < EDGE_CLICK_TOL:
                return e
        return None

    def _set_target(self, target_id):
        """ Setzt Zielknoten. """
        if self.target_node_id is not None and self.target_node_id in self.nodes:
            self.nodes[self.target_node_id].is_target = False
        self.target_node_id = target_id
        if self.target_node_id is not None:
            if self.target_node_id in self.nodes:
                if not self.nodes[self.target_node_id].is_start:
                    self.nodes[self.target_node_id].is_target = True
                else:
                    print(f"Warnung: Startknoten (ID: {self.target_node_id+1}) kann nicht als Ziel gewählt werden.")
                    self.target_node_id = None
            else:
                 print(f"Warnung: Zielknoten-ID {self.target_node_id} nicht im Grid gefunden.")
                 self.target_node_id = None

    def _check_constraints(self, node, potential_logic_pos):
        """ Prüft Kantenlängen-Constraints. """
        if node.locked or node.is_start: return False
        for edge in node.connected_edges:
            neighbor_node_id = edge.n_from if edge.n_to == node.id else edge.n_to
            if neighbor_node_id not in self.nodes: continue
            neighbor_node = self.nodes[neighbor_node_id]
            dist = hypot(potential_logic_pos[0] - neighbor_node.logic[0],
                         potential_logic_pos[1] - neighbor_node.logic[1])
            if not (MIN_EDGE_LEN <= dist <= MAX_EDGE_LEN): return False
        return True

    def _drag(self, current_mouse_pos):
        """ Aktualisiert Position des gezogenen Knotens. """
        if not self.dragging: return
        dx_screen = current_mouse_pos[0] - self.drag_start_pos[0]
        dy_screen = current_mouse_pos[1] - self.drag_start_pos[1]
        current_scale = self.scale if self.scale != 0 else 1
        if current_scale == 0: return
        logic_dx = dx_screen / current_scale; logic_dy = dy_screen / current_scale
        potential_logic_pos = (self.node_initial_logic[0] + logic_dx, self.node_initial_logic[1] + logic_dy)
        if self._check_constraints(self.dragging, potential_logic_pos):
            self.dragging.pos = [self.node_start_pos[0] + dx_screen, self.node_start_pos[1] + dy_screen]

    def _generate_json_output(self, filename=DEFAULT_CONFIG_FILENAME):
        """ Erzeugt JSON-Output für Grid-Konfiguration und speichert sie. """
        output_data = {
            "nodes": [node.to_dict() for node_id, node in sorted(self.nodes.items())],
            "edges": [edge.to_dict(self.nodes) for edge_id, edge in sorted(self.edges.items())],
            "start_id": 0, "target_id": self.target_node_id,
            "scale_px_per_m": self.scale, "drag_threshold_px": self.drag_thr
        }
        json_string = json.dumps(output_data, indent=2)
        print("-" * 20 + " JSON Output (Grid Config) " + "-" * 20)
        print(json_string); print("-" * 61)
        try:
            with open(filename, 'w') as f: f.write(json_string)
            print(f"Grid-Konfiguration erfolgreich in '{filename}' gespeichert.")
        except IOError as e: print(f"Fehler beim Speichern der Grid-Konfiguration: {e}")

    def _load_json_config(self, filename=DEFAULT_CONFIG_FILENAME):
        """ Lädt Grid-Konfiguration aus JSON-Datei. """
        if not os.path.exists(filename): print(f"Fehler: Konfig-Datei '{filename}' nicht gefunden."); return
        try:
            with open(filename, 'r') as f: data = json.load(f)
        except Exception as e: print(f"Fehler beim Laden/Parsen der Konfig-Datei '{filename}': {e}"); return

        required_keys = ["nodes", "edges", "scale_px_per_m", "drag_threshold_px", "target_id"]
        if not all(key in data for key in required_keys): print(f"Fehler: Fehlende Schlüssel in '{filename}'."); return
        if not isinstance(data["nodes"], list) or not isinstance(data["edges"], list): print(f"Fehler: 'nodes'/'edges' müssen Listen sein."); return

        try:
            self.scale = data["scale_px_per_m"]; self.drag_thr = data["drag_threshold_px"]
            loaded_node_ids = set()
            for node_data in data["nodes"]:
                node_id = node_data.get("id")
                if node_id is not None and node_id in self.nodes:
                    self.nodes[node_id].logic = [node_data.get("x_m", 0.0), node_data.get("y_m", 0.0)]
                    self.nodes[node_id].locked = node_data.get("locked", False)
                    loaded_node_ids.add(node_id)
            if len(loaded_node_ids) != len(self.nodes): print(f"Warnung: Knotenanzahl stimmt nicht überein.")

            loaded_edge_ids = set()
            for edge_data in data["edges"]:
                edge_id = edge_data.get("id")
                if edge_id is not None and edge_id in self.edges:
                    self.edges[edge_id].state = edge_data.get("state", 0)
                    loaded_edge_ids.add(edge_id)
            if len(loaded_edge_ids) != len(self.edges): print(f"Warnung: Kantenanzahl stimmt nicht überein.")

            self._set_target(data.get("target_id"))
            self.update_scale(self.scale)
            self.menu.update_sliders()
            print(f"Grid-Konfiguration erfolgreich aus '{filename}' geladen.")
        except Exception as e: print(f"Fehler beim Anwenden der Grid-Konfiguration: {e}")

    # --- Neu: Pfadlösung laden ---
    def _load_path_solution(self, filename=PATH_SOLUTION_FILENAME):
        """ Lädt eine Pfadlösung aus einer JSON-Datei. """
        if not os.path.exists(filename):
            print(f"Fehler: Pfadlösungsdatei '{filename}' nicht gefunden.")
            self.solution_path_nodes = None; self.solution_path_edges = None; self.solution_time = None; self.solution_model = None
            self.show_solution_path = False
            return

        try:
            with open(filename, 'r') as f:
                data = json.load(f)
        except json.JSONDecodeError as e:
            print(f"Fehler: Ungültiges JSON-Format in '{filename}': {e}")
            return
        except IOError as e:
            print(f"Fehler beim Lesen der Pfadlösungsdatei '{filename}': {e}")
            return

        # Validieren der erwarteten Schlüssel
        required_keys = ["path_node_ids", "path_edge_ids", "total_time_s", "model"]
        if not all(key in data for key in required_keys):
            print(f"Fehler: Fehlende Schlüssel in '{filename}'. Erwartet: {required_keys}")
            self.solution_path_nodes = None; self.solution_path_edges = None; self.solution_time = None; self.solution_model = None
            self.show_solution_path = False
            return

        try:
            # Daten speichern und Anzeige aktivieren
            self.solution_path_nodes = data["path_node_ids"]
            self.solution_path_edges = data["path_edge_ids"]
            self.solution_time = data["total_time_s"]
            self.solution_model = data["model"]
            self.show_solution_path = True # Pfad nach Laden automatisch anzeigen
            print(f"Pfadlösung erfolgreich aus '{filename}' aus {self.solution_model} Model geladen (Zeit: {self.solution_time:.2f}s).")
            print(f"  Knoten: {data.get('path_node_labels', self.solution_path_nodes)}") # Labels bevorzugen, wenn vorhanden
            print(f"  Kanten: {self.solution_path_edges}")

        except KeyError as e:
            print(f"Fehler: Fehlender Schlüssel '{e}' beim Verarbeiten der Pfaddaten aus '{filename}'.")
            self.solution_path_nodes = None; self.solution_path_edges = None; self.solution_time = None; self.solution_model = None
            self.show_solution_path = False
        except TypeError as e:
             print(f"Fehler: Unerwarteter Datentyp beim Verarbeiten der Pfaddaten aus '{filename}': {e}")
             self.solution_path_nodes = None; self.solution_path_edges = None; self.solution_time = None; self.solution_model = None
             self.show_solution_path = False

    # --- Neu: Pfad zeichnen ---
    def _draw_solution_path(self):
        """ Zeichnet den geladenen Lösungspfad (grüne Linie). """
        if not self.show_solution_path or self.solution_path_edges is None:
            return

        for edge_id in self.solution_path_edges:
            if edge_id in self.edges:
                edge = self.edges[edge_id]
                # Stelle sicher, dass die verbundenen Knoten existieren
                if edge.n_from in self.nodes and edge.n_to in self.nodes:
                    node_a = self.nodes[edge.n_from]
                    node_b = self.nodes[edge.n_to]
                    pygame.draw.line(self.scr, PATH_COLOR, node_a.pos, node_b.pos, PATH_WIDTH)
            else:
                print(f"Warnung: Kanten-ID {edge_id} aus Pfadlösung nicht im Grid gefunden.")

    # --- Neu: Zeit anzeigen ---
    def _draw_solution_time(self):
        """ Zeigt die geladene Lösungszeit an. """
        if not self.show_solution_path or self.solution_time is None:
            return

        time_text = f"Zeit: {self.solution_time:.2f} s"
        text_render = self.info_font.render(time_text, True, INFO_COLOR)
        # Position oben rechts mit etwas Abstand
        text_rect = text_render.get_rect(topright=(SCREEN_SIZE[0] - 15, 15))
        self.scr.blit(text_render, text_rect)

    # --- Neu: Model anzeigen ---
    def _draw_solution_model(self):
        """ Zeigt das geladene Model an. """
        if not self.show_solution_path or self.solution_model is None:
            return

        model_text = f"Model: {self.solution_model}"
        model_render = self.info_font.render(model_text, True, INFO_COLOR)
        # Position oben rechts mit etwas Abstand
        model_rect = model_render.get_rect(topright=(SCREEN_SIZE[0] - 15, 45))
        self.scr.blit(model_render, model_rect)

    # --- Hauptschleife ---
    def run(self):
        """ Startet die Hauptschleife. """
        run = True
        while run:
            # --- Event Handling ---
            triggered_button_action = None # Store action from button click

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    run = False

                # Give events to menu first
                self.menu.handle(ev)

                # Handle Button Events (only if menu is not showing)
                if not self.menu.show:
                    for button in self.buttons:
                        # Disable button interaction when menu is shown (Button class handles drawing)
                        button.is_disabled = self.menu.show
                        if not button.is_disabled:
                            action = button.handle_event(ev)
                            if action:
                                triggered_button_action = action
                                break # Process only one button click per event cycle

                # Keyboard Events (process regardless of menu, except for simulation keys)
                if ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_m: # Menu-Toggle (always available)
                        # Action triggered via button/action logic below
                        triggered_button_action = "TOGGLE_MENU"

                    elif not self.menu.show: # Only process these if menu is not active
                        if ev.key == pygame.K_a: triggered_button_action = "TARGET_A"
                        elif ev.key == pygame.K_b: triggered_button_action = "TARGET_B"
                        elif ev.key == pygame.K_c: triggered_button_action = "TARGET_C"
                        elif ev.key == pygame.K_n: triggered_button_action = "TARGET_N"
                        elif ev.key == pygame.K_s: triggered_button_action = "SAVE_GRID"
                        elif ev.key == pygame.K_l: triggered_button_action = "LOAD_GRID"
                        elif ev.key == pygame.K_p: triggered_button_action = "LOAD_PATH"
                        elif ev.key == pygame.K_h: triggered_button_action = "SOLVE_PATH" # Changed from TOGGLE_PATH


                # Simulations-Events (dragging, edge clicks - only if menu is not active and no button was clicked)
                if not self.menu.show and not triggered_button_action:
                    # Check if the mouse event is over any button before processing simulation interactions
                    mouse_pos = pygame.mouse.get_pos()
                    is_over_button = any(button.rect.collidepoint(mouse_pos) for button in self.buttons)

                    if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1 and not is_over_button: # Linksklick
                        n = self._node_at(ev.pos)
                        if n and not n.is_start: # Klick auf Knoten
                            self.dragging = n; self.drag_start_pos = ev.pos
                            self.node_start_pos = list(n.pos); self.node_initial_logic = list(n.logic)
                        elif not n: # Klick auf Kante/Leerraum
                             e = self._edge_at(ev.pos)
                             if e: e.cycle()

                    elif ev.type == pygame.MOUSEMOTION and self.dragging:
                        self._drag(ev.pos) # Knoten ziehen

                    elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1: # Linke Maustaste losgelassen
                        if self.dragging: # Only process drag release if a node was being dragged
                            # Check if release happens over a button - if so, discard drag action? Or complete it? Let's complete it.
                            # is_release_on_button = any(button.rect.collidepoint(ev.pos) for button in self.buttons)
                            # if not is_release_on_button: # Complete drag only if not releasing on a button

                            drag_dist = hypot(ev.pos[0] - self.drag_start_pos[0], ev.pos[1] - self.drag_start_pos[1])
                            was_locked = self.dragging.locked

                            if drag_dist < self.drag_thr: # Kurzer Klick -> Lock umschalten
                                self.dragging.locked = not self.dragging.locked
                            elif not was_locked: # Längeres Ziehen & nicht gesperrt
                                dx_screen = ev.pos[0] - self.drag_start_pos[0]; dy_screen = ev.pos[1] - self.drag_start_pos[1]
                                current_scale = self.scale if self.scale != 0 else 1
                                if current_scale != 0:
                                    logic_dx = dx_screen / current_scale; logic_dy = dy_screen / current_scale
                                    final_logic_pos = (self.node_initial_logic[0] + logic_dx, self.node_initial_logic[1] + logic_dy)
                                    if self._check_constraints(self.dragging, final_logic_pos):
                                        self.dragging.logic = list(final_logic_pos)
                            self.dragging.update_xy(self.cx, self.cy, self.scale)
                            self.dragging = None


            # --- Perform Action Triggered by Button or Key ---
            if triggered_button_action:
                print(f"Action triggered: {triggered_button_action}") # Debug print
                if triggered_button_action == "TARGET_A": self._set_target(TARGET_MAP[pygame.K_a])
                elif triggered_button_action == "TARGET_B": self._set_target(TARGET_MAP[pygame.K_b])
                elif triggered_button_action == "TARGET_C": self._set_target(TARGET_MAP[pygame.K_c])
                elif triggered_button_action == "TARGET_N": self._set_target(None)
                elif triggered_button_action == "SAVE_GRID": self._generate_json_output()
                elif triggered_button_action == "LOAD_GRID": self._load_json_config()
                elif triggered_button_action == "LOAD_PATH":
                    self._load_path_solution() # Keep existing load functionality
                    # Ensure model name reflects loading if successful
                    if self.solution_path_edges is not None:
                         self.solution_model = "Loaded JSON"
                    else:
                         self.solution_model = None # Reset if loading failed

                # --- SOLVE PATH Action (Updated) ---
                elif triggered_button_action == "SOLVE_PATH":
                    if calculate_fastest_path is None:
                         print("Fehler: Dijkstra Solver ('calculate_fastest_path') ist nicht verfügbar.")
                    elif self.target_node_id is None:
                         print("Fehler: Bitte zuerst einen Zielknoten auswählen (A, B, C oder Buttons).")
                         self.solution_path_nodes = None; self.solution_path_edges = None; self.solution_time = None; self.solution_model = None
                         self.show_solution_path = False
                    else:
                        print(f"Starte Pfadberechnung von Knoten 0 nach {self.target_node_id}...")

                        # --- Prepare data for calculate_fastest_path ---
                        grid_data_for_solver = {
                            "nodes": [node.to_dict() for node_id, node in sorted(self.nodes.items())],
                            "edges": [edge.to_dict(self.nodes) for edge_id, edge in sorted(self.edges.items())],
                            "start_id": 0, # Assuming start is always node 0
                            "target_id": self.target_node_id,
                            # Add other info the solver might implicitly use from config, if any
                            "scale_px_per_m": self.scale,
                            "drag_threshold_px": self.drag_thr
                        }

                        try:
                             # Call the correct function with the structured data and parameters
                             result_dict = calculate_fastest_path(
                                 grid_data_for_solver,
                                 SPEED_MPS,
                                 BLOCKADE_PENALTY_S
                             )

                             # --- Process the result dictionary ---
                             if "error" in result_dict and result_dict["error"] is not None:
                                 print(f"Fehler bei der Pfadfindung: {result_dict['error']}")
                                 self.solution_path_nodes = None
                                 self.solution_path_edges = None
                                 self.solution_time = None
                                 self.solution_model = "Dijkstra (Error)"
                                 self.show_solution_path = False
                             elif "path_node_ids" in result_dict: # Check for success keys
                                 self.solution_path_nodes = result_dict["path_node_ids"]
                                 self.solution_path_edges = result_dict["path_edge_ids"]
                                 self.solution_time = result_dict["total_time_s"]
                                 self.solution_model = result_dict.get("model", "Dijkstra") # Use model from result if available
                                 self.show_solution_path = True
                                 print(f"Pfadberechnung erfolgreich! Modell: {self.solution_model}. Zeit: {self.solution_time:.2f}s")
                                 print(f"  Knoten-IDs: {self.solution_path_nodes}")
                                 print(f"  Kanten-IDs: {self.solution_path_edges}")
                                 # Optionally print labels if available
                                 if "path_node_labels" in result_dict:
                                     print(f"  Knoten-Labels: {result_dict['path_node_labels']}")
                             else:
                                 # Should be caught by the "error" check, but just in case
                                 print("Pfadfindung fehlgeschlagen (Unerwartetes Ergebnisformat).")
                                 self.solution_path_nodes = None; self.solution_path_edges = None; self.solution_time = None; self.solution_model = "Dijkstra (Failed)"
                                 self.show_solution_path = False

                        except Exception as e:
                            print(f"Fehler während des Aufrufs von 'calculate_fastest_path': {e}")
                            import traceback
                            traceback.print_exc() # Print detailed traceback
                            self.solution_path_nodes = None; self.solution_path_edges = None; self.solution_time = None; self.solution_model = "Dijkstra (Crash)"
                            self.show_solution_path = False


                elif triggered_button_action == "TOGGLE_MENU":
                     self.menu.show = not self.menu.show
                     # Update button disabled state (redundant with check at event handling start, but safe)
                     for btn in self.buttons:
                          btn.is_disabled = self.menu.show


            # --- Alles zeichnen ---
            self.scr.fill(BG_COLOR)
            # Kanten zeichnen
            for e in self.edges.values():
                e.draw(self.scr, self.nodes, self.font)
            # Lösungspfad darüber zeichnen (if active)
            self._draw_solution_path()
            # Knoten darüber zeichnen
            for n in self.nodes.values():
                n.draw(self.scr, self.font)

            # --- Buttons zeichnen ---
            for button in self.buttons:
                button.draw(self.scr) # Button draw method handles hover/disabled state

            # Menü darüber zeichnen (if active)
            self.menu.draw(self.scr, self.font)
            # Lösungszeit darüber zeichnen (if active and available)
            self._draw_solution_time()
            # Model darüber zeichnen (if active and available)
            self._draw_solution_model()


            pygame.display.flip()
            self.clock.tick(FPS)

        pygame.quit()

# --- Startpunkt des Programms ---
if __name__ == '__main__':
    # Update the check for the correct function name
    if calculate_fastest_path is None:
         print("-" * 30)
         print("WARNUNG: 'Solve Path' Funktion ist deaktiviert.")
         print("Stellen Sie sicher, dass 'dijkstra_grid_solver.py' im selben Verzeichnis ist")
         print("und eine Funktion 'calculate_fastest_path(data, speed, penalty)' enthält.")
         print("-" * 30)

    sim = Simulator()
    sim.run()
