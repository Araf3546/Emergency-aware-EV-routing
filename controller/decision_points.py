"""Where does a vehicle have a meaningful routing choice? (trigger comparison, 2026-09-12)

Written to compare the current 30-second reroute trigger with junction-triggered rerouting
(`design choices.md`, "Design-choice comparison: the reroute trigger"). The same definition is
used both as the junction trigger and to classify every routing decision in both conditions,
so the two are measured identically. Nothing in the production runners calls this module.

Definition. A vehicle on edge `e` is deciding which edge to take at the junction `J` where `e`
ends. A candidate next edge `c` is a **meaningful alternative** only if all of these hold:

1. **valid** - it is one of E3's own candidates (`route_utils.get_outgoing_edges`): connected
   to `e` by a lane connection in the network, passenger-accessible, in the network's largest
   strongly connected component, and not closed by an active incident for this vehicle;
2. **reaches the destination** - a path exists from `c` to the vehicle's destination;
3. **not a loop** - the shortest path from `c` to the destination does not pass through `J`
   again. Such a path is "take another exit, plus a loop back here" - equivalent to another
   candidate with a detour, not a distinct route;
4. **competitive** - the shortest remaining distance via `c` is at most (1 + tolerance) times
   the shortest via the best candidate. Default tolerance 0.5: an option 50% longer than the
   best is still one a driver might take to avoid a jam; a longer one is not a real choice;
5. **distinct** - candidates ending at the same next junction are the same option, counted once.

A junction offers a **real choice** when at least two distinct meaningful options remain.

Distances are static shortest-path lengths in metres on the same passenger graph E3 draws its
candidates from, ignoring live closures beyond (1) - a stated approximation. They are computed
once per destination with a reverse Dijkstra and cached for the run.
"""

from __future__ import annotations

import heapq
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from algorithms.route_utils import DEFAULT_NETWORK_PATH, passenger_adjacency

DEFAULT_DETOUR_TOLERANCE = 0.5


class DecisionPoints:
    def __init__(self, network_path: Optional[Path] = None,
                 detour_tolerance: float = DEFAULT_DETOUR_TOLERANCE) -> None:
        self.detour_tolerance = detour_tolerance
        self.from_node: Dict[str, str] = {}
        self.to_node: Dict[str, str] = {}
        self.length: Dict[str, float] = {}
        root = ET.parse(Path(network_path) if network_path else DEFAULT_NETWORK_PATH).getroot()
        for edge in root.findall("edge"):
            if edge.get("function") == "internal":
                continue
            edge_id = edge.get("id")
            self.from_node[edge_id] = edge.get("from", "")
            self.to_node[edge_id] = edge.get("to", "")
            lanes = edge.findall("lane")
            self.length[edge_id] = max((float(l.get("length", 0.0)) for l in lanes), default=0.0)
        self.successors = passenger_adjacency()
        self.predecessors: Dict[str, List[str]] = {}
        for edge_id, targets in self.successors.items():
            for target in targets:
                self.predecessors.setdefault(target, []).append(edge_id)
        self._to_destination: Dict[str, Tuple[Dict[str, float], Dict[str, str]]] = {}

    def to_destination(self, destination: str) -> Tuple[Dict[str, float], Dict[str, str]]:
        """(remaining metres from the start of each edge to the end of `destination`,
        next edge on that shortest path). Reverse Dijkstra, cached per destination."""
        cached = self._to_destination.get(destination)
        if cached is not None:
            return cached
        distance: Dict[str, float] = {}
        following: Dict[str, str] = {}
        if destination in self.length:
            distance[destination] = self.length[destination]
            heap = [(distance[destination], destination)]
            while heap:
                d, edge = heapq.heappop(heap)
                if d > distance.get(edge, float("inf")):
                    continue
                for previous in self.predecessors.get(edge, ()):
                    candidate = d + self.length.get(previous, 0.0)
                    if candidate < distance.get(previous, float("inf")):
                        distance[previous] = candidate
                        following[previous] = edge
                        heapq.heappush(heap, (candidate, previous))
        self._to_destination[destination] = (distance, following)
        return distance, following

    def _returns_to(self, start: str, junction: str, following: Dict[str, str], destination: str) -> bool:
        """Does the shortest path from `start` to the destination pass through `junction` again?"""
        edge = following.get(start)
        steps = 0
        while edge is not None and steps < 10000:
            if self.from_node.get(edge) == junction:
                return True
            if edge == destination:
                return False
            edge = following.get(edge)
            steps += 1
        return False

    def classify(self, candidates: Sequence[str], destination: str,
                 tolerance: Optional[float] = None) -> dict:
        """Classify one decision's candidate set. Returns the counts and the meaningful options."""
        tolerance = self.detour_tolerance if tolerance is None else tolerance
        candidates = list(candidates)
        result = {"candidates": len(candidates), "reachable": 0, "loop_free": 0,
                  "meaningful": [], "options": 0}
        if not candidates:
            return result
        junction = self.from_node.get(candidates[0], "")
        distance, following = self.to_destination(destination)
        reachable = [c for c in candidates if c in distance]
        result["reachable"] = len(reachable)
        loop_free = [c for c in reachable if not self._returns_to(c, junction, following, destination)]
        result["loop_free"] = len(loop_free)
        if not loop_free:
            return result
        best = min(distance[c] for c in loop_free)
        meaningful = [c for c in loop_free if distance[c] <= (1.0 + tolerance) * best]
        result["meaningful"] = meaningful
        result["options"] = len({self.to_node.get(c, c) for c in meaningful})
        return result

    def has_real_choice(self, candidates: Sequence[str], destination: str) -> bool:
        return self.classify(candidates, destination)["options"] >= 2

    def route_length(self, edges: Sequence[str]) -> float:
        return sum(self.length.get(edge, 0.0) for edge in edges)
