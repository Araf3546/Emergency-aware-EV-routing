import json
import random
from pathlib import Path

try:
    import yaml
except ImportError:  # Optional dependency for future scenario files.
    yaml = None

import traci

from p3_simulation_config import AMBIENT_EMERGENCY_TYPES

# ===============================
# EMERGENCY CONFIGURATION
# ===============================

EMERGENCY_TYPES = ["accident", "fire"]
EMERGENCY_VEHICLES = ["ambulance", "fire_truck", "police", "relief_vehicle"]

# Road blocking parameters
ROAD_BLOCK_RADIUS = 3          # number of connected road segments affected

# Incident durations come from scenarios/emergency_scenarios.yaml (end_time per event).
# There is deliberately no EMERGENCY_DURATION constant: hardcoding one previously
# overrode every validated incident window (audit finding B5).

# Vehicle classes used when an incident closes an edge. Per experiment_contract.md
# section 11 (decision 13.2) a closure blocks every class except "emergency", so the
# dispatched responders can still reach the incident.
#
# This list is written out explicitly rather than taken from sumolib. sumolib's
# SUMO_VEHICLE_CLASSES also contains deprecated aliases, and one of them
# ("public_emergency") maps onto "emergency" - disallowing the alias silently
# disallowed the emergency class too, which defeated the whole exemption.
SUMO_VEHICLE_CLASSES = (
    "ignoring", "private", "emergency", "authority", "army", "vip", "pedestrian",
    "passenger", "hov", "taxi", "bus", "coach", "delivery", "truck", "trailer",
    "motorcycle", "moped", "bicycle", "evehicle", "tram", "rail_urban", "rail",
    "rail_electric", "rail_fast", "ship", "container", "cable_car", "subway",
    "aircraft", "wheelchair", "scooter", "drone", "custom1", "custom2",
)

EMERGENCY_EXEMPT_VCLASS = "emergency"

# The vehicle class used only by the incident-distance query (2026-09-11). Why it exists:
# while an incident is active its centre edge is closed to every class except emergency, so
# `findRoute` from an ordinary car to that edge returns an EMPTY route - no exception - and the
# distance code read that as "0 metres away". Every ordinary car in the network then reported
# itself within 200 m of the incident for as long as it lasted (124,472 of 132,285 queries in
# a full run returned exactly 0).
#
# The query is now made with a runtime twin of the car's own vehicle type whose only change is
# this class, and this class is exempt from closures alongside emergency. `custom1` was chosen
# because, measured on this network, it has exactly the permissions of `passenger` on all
# 43,982 lanes and all 5 connection-level overrides - so the twin measures precisely the road
# distance a car would drive if the incident edge were open. Using an emergency type instead
# was measured and rejected: bus lanes open to emergency but not to cars made 31 of 532 cars
# look 110-160 m closer than they are.
#
# No vehicle in the scenario uses `custom1` (only `passenger` and `emergency` appear), so
# exempting it from closures changes nothing for real traffic.
DISTANCE_PROBE_VCLASS = "custom1"
CLOSURE_EXEMPT_VCLASSES = (EMERGENCY_EXEMPT_VCLASS, DISTANCE_PROBE_VCLASS)
CLOSURE_DISALLOWED_CLASSES = sorted(
    vehicle_class for vehicle_class in SUMO_VEHICLE_CLASSES if vehicle_class not in CLOSURE_EXEMPT_VCLASSES
)
DISTANCE_TWIN_SUFFIX = "__incident_distance"

# PD-1 question 2, decided by the supervisor on 2026-09-13: "Ambient emergency vehicles should NOT
# retain emergency-only road permissions." The one road permission unique to the emergency class
# on this network is the incident-closure exemption above: every static lane and connection that
# admits `emergency` admits `authority` too, and no other class (measured on all 23,284 lanes and
# every connection override). So at the start of each run the four ambient emergency vTypes are
# switched to `authority`. They lose the closure exemption and nothing else: an incident closes
# its road to them as to a car, while the 20 bus/authority lanes they share with buses stay open
# to them. Their frozen routes need those lanes: 36 of the 240 cross the bus-only edge 1174541722,
# and under car permissions 28 of those 36 could not finish (the user's choice between the two
# options, 2026-09-13, recorded in design choices.md). The three responder types keep `emergency`,
# so a dispatched responder still reaches its closed incident road. Their vehicles enter the
# network only after their incident starts and their routes end on its centre edge, so in
# practice the emergency class, and with it the exemption, is held only by active responders.
AMBIENT_EMERGENCY_ROAD_CLASS = "authority"
# Recorded in every tuning reference, trial and weight file; a file without it predates the rule.
ROAD_PERMISSION_POLICY = (
    "PD-1 Q2 (2026-09-13): closure exemption for dispatched responders only; "
    "ambient emergency vTypes run as vClass authority"
)
# A closure-blocked class -> the closure-exempt class with exactly its permissions on this
# network, used for the incident-distance query twin (see _distance_query_type).
DISTANCE_TWIN_CLASSES = {
    "passenger": DISTANCE_PROBE_VCLASS,
    AMBIENT_EMERGENCY_ROAD_CLASS: EMERGENCY_EXEMPT_VCLASS,
}

# ===============================
# EMERGENCY EVENT CLASS
# ===============================

class EmergencyEvent:
    def __init__(self, event_id, start_time, end_time, emergency_type, edge_id,
                 blocked_edges=None, sumo_action="close_edge", capacity_factor=0.0):
        self.event_id = event_id
        self.start_time = start_time
        # end_time is required and comes from the validated scenario file (audit B5).
        self.end_time = end_time
        self.emergency_type = emergency_type
        self.center_edge = edge_id
        self.blocked_edges = blocked_edges if blocked_edges else []
        self.sumo_action = sumo_action
        self.capacity_factor = capacity_factor
        self.applied_action = None
        self.is_active = False
        self.response_vehicles = set()  # emergency vehicles responding to this event
        self.first_arrival_time = None
        self.first_arrival_vehicle = None
        self.response_time = None
        self.arrival_edge = None
        # Audit M12. SUMO teleports a vehicle that has been jammed too long and drops it
        # further along its route. On 2026-09-08 responder 401 was teleported straight
        # onto incident 401's own centre edge, which satisfies the arrival test in
        # contract decision 13.3 on a trip that was never driven. A response time taken
        # from a teleported responder is an artifact: it is recorded and reported, but it
        # is not a measurement and must never enter an aggregate.
        self.responder_teleported = False
        self.responder_teleport_times = []
        self.response_valid = None
        self.invalid_reason = None
        # True when the responder reached the centre edge after the incident window
        # closed. This is a reporting flag only - it never suppresses the measurement
        # (contract sections 7 and 8 put no window condition on response time).
        self.arrived_after_incident_end = False
        # Taken from the scenario file's dispatch block. Contract section 7: a response
        # time attributed to any vehicle other than this one must not be reported as
        # this incident's response time.
        self.dispatch_vehicle_id = None
        self.metadata = {}
        
    def activate(self):
        """Activate the emergency event"""
        self.is_active = True
        
    def deactivate(self):
        """Deactivate the emergency event"""
        self.is_active = False
        
    def add_response_vehicle(self, veh_id):
        """Track emergency vehicle responding to this event"""
        self.response_vehicles.add(veh_id)


# ===============================
# EMERGENCY MANAGER CLASS
# ===============================

class EmergencyManager:
    # The incident-distance cache (2026-09-12) - see _route_distance. A performance change
    # only: validated as bit-for-bit identical to the uncached code before adoption. Turned
    # off only to re-run that check (scripts/diagnostics/ab_identity.py --distance-cache off).
    cache_incident_distances = True
    # PD-1 question 2 (see AMBIENT_EMERGENCY_ROAD_CLASS). Turned off only to re-run the
    # before/after check (scripts/diagnostics/pd1_q2_verify.py); every runner leaves it on.
    restrict_ambient_road_access = True

    def __init__(self):
        self.events = {}
        self.event_counter = 0
        self.active_events = set()
        self.blocked_edges = set()
        self.generated = False
        self.planned_events = []
        self.scenario_source = None
        # Lane state saved before a closure, so deactivation can restore it exactly.
        self.saved_lane_state = {}
        # Incident application failures are recorded, never silently swallowed
        # (contract section 5 requires route/failure reporting).
        self.incident_errors = []
        # Audit M12: every teleport of a dispatched responder, whenever it happened.
        self.responder_teleports = []
        # vehicle type -> the type its incident-distance query is made with. Per SUMO run.
        self._distance_query_types = {}
        # (from edge, to edge, query type, open incidents) -> route distance. Per SUMO run.
        self._distance_cache = {}
        # PD-1 (supervisor decision, 2026-09-13): the explicit "active responder" flag. The
        # vehicles dispatched to an incident that has started and not yet been reached,
        # refreshed once per step at the end of update() - the same definition, taken at the
        # same point of the step, as the P3 protocol path's `_active_responders`. Vehicle
        # class alone never puts a vehicle in here.
        self.active_responder_ids = frozenset()
        # PD-1 question 2: ambient vTypes not yet switched in this SUMO run, and the class each
        # switched type ended up with (reported by the tuner as evidence the switch happened).
        self._ambient_types_pending = set(AMBIENT_EMERGENCY_TYPES)
        self.ambient_road_classes = {}

    def set_planned_events(self, events):
        """Inject prevalidated emergency scenarios instead of generating random ones."""
        self.planned_events = [self._normalize_event_definition(event) for event in events]
        self.scenario_source = "in_memory"
        self.events = {}
        self.active_events = set()
        self.blocked_edges = set()
        self.generated = False
        # Every runner loads the scenario before starting SUMO, so this is the per-run reset.
        # The distance twins are runtime vehicle types and do not survive into a new SUMO
        # instance - scripts/16_compare runs two arms in one process. The distance cache is
        # dropped with them: its answers belong to one SUMO instance.
        self._distance_query_types = {}
        self._distance_cache = {}
        self.active_responder_ids = frozenset()
        # Per-run records (2026-09-13). These two lists were never reset, so a run inherited the
        # previous run's responder teleports and incident errors in its report whenever two runs
        # shared a process - scripts/16 runs baseline then P3 in one process, and a tuning study
        # runs every trial in one. Response validity was never affected: it comes from each
        # event's own `responder_teleported` flag, and the events are rebuilt above.
        # `saved_lane_state` is deliberately left alone: restore is filtered by incident and
        # each activation overwrites its incident's entries, so a leftover can never be used.
        self.responder_teleports = []
        self.incident_errors = []
        # PD-1 question 2: a vType class change lives in one SUMO instance, so each run switches
        # the ambient types again.
        self._ambient_types_pending = set(AMBIENT_EMERGENCY_TYPES)
        self.ambient_road_classes = {}

    def load_planned_events(self, scenario_file):
        """Load prevalidated emergency scenarios from JSON or YAML."""
        path = Path(scenario_file)
        if not path.is_file():
            raise FileNotFoundError(f"Emergency scenario file not found: {path}")

        text = path.read_text(encoding="utf-8")
        if path.suffix.lower() == ".json":
            document = json.loads(text)
        elif path.suffix.lower() in {".yml", ".yaml"}:
            if yaml is None:
                raise RuntimeError(
                    "PyYAML is required to load YAML emergency scenarios; "
                    "install it or provide a JSON scenario file."
                )
            document = yaml.safe_load(text)
        else:
            raise ValueError(f"Unsupported emergency scenario format: {path.suffix}")

        if isinstance(document, dict):
            events = document.get("events", [])
        else:
            events = document
        if not isinstance(events, list):
            raise ValueError("Emergency scenario file must contain a list of events")

        self.set_planned_events(events)
        self.scenario_source = str(path)

    def _load_default_scenario_file(self):
        """Load the canonical scenario file if it exists in the workspace."""
        candidates = [
            Path.cwd() / "scenarios" / "emergency_scenarios.yaml",
            Path.cwd() / "scenarios" / "emergency_scenarios.json",
            Path(__file__).resolve().parent / "scenarios" / "emergency_scenarios.yaml",
            Path(__file__).resolve().parent / "scenarios" / "emergency_scenarios.json",
        ]
        for candidate in candidates:
            if candidate.is_file():
                self.load_planned_events(candidate)
                return candidate
        return None

    def load_canonical_scenario_file(self):
        """Public wrapper for loading the seeded thesis scenario at startup."""
        return self._load_default_scenario_file()

    @staticmethod
    def _normalize_event_definition(event_definition):
        if not isinstance(event_definition, dict):
            raise ValueError("Each emergency event must be a mapping")

        center_edge = event_definition.get("center_edge") or event_definition.get("edge_id")
        if not center_edge:
            raise ValueError("Emergency event requires 'center_edge' or 'edge_id'")

        start_time = event_definition.get("start_time")
        end_time = event_definition.get("end_time")
        emergency_type = event_definition.get("type") or event_definition.get("emergency_type") or event_definition.get("incident_type")
        if start_time is None or end_time is None or emergency_type is None:
            raise ValueError("Emergency event requires start_time, end_time, and type")

        # The scenario file uses "affected_edges" (contract section 7 wording).
        # "blocked_edges" is accepted as a legacy alias. Falling back to the centre
        # edge silently used to hide multi-edge incidents entirely (audit B6).
        blocked_edges = event_definition.get("affected_edges") or event_definition.get("blocked_edges")
        if not blocked_edges:
            raise ValueError(
                f"Emergency event {event_definition.get('event_id', event_definition.get('id'))} "
                "requires 'affected_edges' (or legacy 'blocked_edges')"
            )

        metadata = {
            key: value
            for key, value in event_definition.items()
            if key not in {
                "event_id", "id", "start_time", "end_time", "type", "emergency_type",
                "center_edge", "edge_id", "blocked_edges", "affected_edges",
                "sumo_action", "capacity_factor",
            }
        }
        return {
            "event_id": event_definition.get("event_id", event_definition.get("id")),
            "start_time": float(start_time),
            "end_time": float(end_time),
            "emergency_type": str(emergency_type),
            "center_edge": str(center_edge),
            "blocked_edges": [str(edge) for edge in blocked_edges],
            "sumo_action": str(event_definition.get("sumo_action", "close_edge")),
            "capacity_factor": float(event_definition.get("capacity_factor", 0.0)),
            "metadata": metadata,
        }

    def _add_event(self, event_definition):
        event_id = event_definition.get("event_id", self.event_counter)
        event = EmergencyEvent(
            event_id=event_id,
            start_time=event_definition["start_time"],
            end_time=event_definition["end_time"],
            emergency_type=event_definition["emergency_type"],
            edge_id=event_definition["center_edge"],
            blocked_edges=event_definition["blocked_edges"],
            sumo_action=event_definition.get("sumo_action", "close_edge"),
            capacity_factor=event_definition.get("capacity_factor", 0.0),
        )
        event.metadata = event_definition.get("metadata", {})
        event.dispatch_vehicle_id = self._dispatch_vehicle_id(event.metadata)
        self.events[event.event_id] = event
        self.event_counter += 1

    @staticmethod
    def _dispatch_vehicle_id(metadata):
        """Return the scenario file's dispatch.vehicle_id for an event, or None."""
        if not isinstance(metadata, dict):
            return None
        dispatch = metadata.get("dispatch")
        if isinstance(dispatch, dict) and dispatch.get("vehicle_id"):
            return str(dispatch["vehicle_id"])
        return None

    def _distance_query_type(self, vehicle_type):
        """The vehicle type an incident-distance query is made with.

        A `passenger`-class type is replaced by a runtime twin identical in every parameter
        except vehicle class `custom1`, which closures exempt - so the query measures the
        road distance this car would drive if the incident edge were open. Every other type
        is used as it is: emergency types are already exempt, and their answers were already
        correct, so responders are untouched by construction.

        PD-1 question 2: an ambient emergency type now runs as `authority`, which closures
        block, so it gets a twin of class `emergency` - its own original class, with exactly
        its permissions and the exemption. Its answers are therefore the ones it got before.

        The twin is a copy of the car's own type, so SUMO treats the query exactly as before
        apart from the closure: one `findRoute` per distance, in the same order, with the same
        speed and speed-factor settings.
        """
        if vehicle_type in self._distance_query_types:
            return self._distance_query_types[vehicle_type]

        query_type = vehicle_type
        try:
            twin_class = DISTANCE_TWIN_CLASSES.get(
                traci.vehicletype.getVehicleClass(vehicle_type) if vehicle_type else None)
            if twin_class:
                twin = f"{vehicle_type}{DISTANCE_TWIN_SUFFIX}"
                if twin not in traci.vehicletype.getIDList():
                    traci.vehicletype.copy(vehicle_type, twin)
                    traci.vehicletype.setVehicleClass(twin, twin_class)
                query_type = twin
        except traci.TraCIException:
            query_type = vehicle_type

        self._distance_query_types[vehicle_type] = query_type
        return query_type

    def _route_distance(self, from_edge, to_edge, vehicle_type=""):
        """Route distance between two edges, or infinity if there is no route - cached.

        The answer is remembered under the key (from edge, to edge, query type, set of open
        incidents) and never recomputed. That key holds everything SUMO's answer depends on,
        so a remembered answer cannot go stale:

        * the road graph - never changes during a run;
        * the vehicle type - permissions and speeds differ by type (1,623 of 5,560 measured
          routes give a different answer for a different type). The key holds the type the
          query is made with, which `_distance_query_type` derives from the vehicle's own type;
        * which incidents are open - a closure changes lane permissions (and would change lane
          speeds if an incident used a capacity factor above 0). `active_events` changes in
          `update()` in the same call that applies or restores the lanes, so the set in the
          key always describes the lanes SUMO is routing over.

        Nothing else can change an answer: there is no `adaptTraveltime`, no `setEffort`, no
        weight file and no rerouting device anywhere in the project, so SUMO routes on static
        travel times; the only runtime changes to lanes or types are the closures above and
        the distance twin, which is created before its first query and never modified.
        Measured before adoption: over 129,309 real queries, 8,145 keys, 0 keys ever gave two
        different answers. The cache lives for one SUMO run (reset in `set_planned_events`).

        With the frozen speed factors of contract 13.13 route queries no longer reach the
        traffic, so asking SUMO less often changes nothing - checked by a bit-for-bit
        comparison of full runs with and without the cache.
        """
        if not self.cache_incident_distances:
            return self._query_route_distance(from_edge, to_edge, vehicle_type)
        key = (from_edge, to_edge, vehicle_type, frozenset(self.active_events))
        distance = self._distance_cache.get(key)
        if distance is None:
            distance = self._query_route_distance(from_edge, to_edge, vehicle_type)
            self._distance_cache[key] = distance
        return distance

    def _query_route_distance(self, from_edge, to_edge, vehicle_type=""):
        """Ask SUMO for the route distance between two edges; infinity if there is no route.

        `vehicle_type` must be a vType *id*, not a vehicle class. Passing the class name
        "passenger" made every call raise "The vehicle type 'passenger' is not known", so
        this always returned infinity - which made `get_nearest_emergency` return None for
        every vehicle that was not already standing on the incident edge, and therefore
        stopped the protocol's yield requests from ever being raised. An empty string is
        SUMO's default vehicle type and is the correct fallback.
        """
        try:
            route = traci.simulation.findRoute(from_edge, to_edge, vehicle_type)
        except traci.TraCIException:
            return float("inf")

        # SUMO does not always raise when there is no route: for a destination this vehicle
        # type may not enter it returns an empty route whose length is 0.0. Reading that as
        # "0 metres away" is the defect found on 2026-09-11. No route means infinitely far,
        # as this docstring always promised.
        if not getattr(route, "edges", None):
            return float("inf")

        distance = getattr(route, "length", None)
        if distance is None:
            distance = getattr(route, "travelTime", None)
        if distance is None:
            return float("inf")
        return float(distance)
        
    def generate_emergencies(self):
        """Load prevalidated emergency events and avoid ad hoc random generation."""
        if self.generated:
            return

        if not self.planned_events:
            self._load_default_scenario_file()

        if self.planned_events:
            for event_definition in self.planned_events:
                self._add_event(event_definition)
            print(f"[EMERGENCY] Loaded {len(self.planned_events)} prevalidated emergency events")
        else:
            print("[EMERGENCY] No prevalidated emergency scenarios loaded; dynamic generation disabled")
        self.generated = True
        
    def get_active_responder_ids(self, sim_time=None):
        """Vehicle ids that are *actively responding to an incident right now*.

        Audit P9. Emergency behaviour used to be keyed on vehicle *type*, so all 240
        ambient emergency-class vehicles required by contract decision 13.1 broadcast
        emergency priority, claimed the emergency corridor weight and made ordinary
        traffic yield - for the whole hour, everywhere. A real ambulance has right of
        way while it is on a call, not while it is driving back to the depot.

        A vehicle is responding from the moment its incident starts until it reaches
        the centre edge (contract decision 13.3), after which the response is over and
        the measurement has been taken. Ambient emergency vehicles are never included,
        so they behave as ordinary traffic for the protocol while remaining part of the
        traffic composition.
        """
        responders = set()
        for event in self.events.values():
            vehicle_id = event.dispatch_vehicle_id
            if not vehicle_id:
                continue
            if event.first_arrival_time is not None:
                continue
            if sim_time is not None and sim_time < event.start_time:
                continue
            responders.add(vehicle_id)
        return responders

    def note_teleports(self, sim_time, teleport_ids):
        """Record any teleport of a dispatched responder. Returns the new records.

        Audit M12. SUMO teleports a vehicle that has waited too long in a jam and
        reinserts it further along its own route. That is a simulator escape hatch, not
        driving, and it can land the responder directly on the incident's centre edge -
        which is exactly the arrival test in contract decision 13.3. The response time
        that follows was never driven.

        A teleport that happens *before* the arrival is recorded invalidates the event's
        response time. A teleport after the arrival cannot have manufactured it, so it is
        counted and reported but changes nothing.
        """
        teleporting = set(teleport_ids or ())
        if not teleporting:
            return []

        new_records = []
        for event_id, event in self.events.items():
            vehicle_id = event.dispatch_vehicle_id
            if not vehicle_id or vehicle_id not in teleporting:
                continue
            before_arrival = event.first_arrival_time is None
            record = {
                "event_id": event_id,
                "vehicle_id": vehicle_id,
                "sim_time": float(sim_time),
                "before_arrival": before_arrival,
            }
            self.responder_teleports.append(record)
            new_records.append(record)
            if before_arrival:
                event.responder_teleported = True
                event.responder_teleport_times.append(float(sim_time))
                print(
                    f"[EMERGENCY] Responder {vehicle_id} for event {event_id} TELEPORTED at "
                    f"t={float(sim_time):.0f} - this event's response time is now invalid (audit M12)"
                )
        return new_records

    def poll_teleports(self, sim_time):
        """Ask SUMO which vehicles started teleporting this step and record responders.

        Called from update(), so every runner that advances the manager gets the check
        without having to remember it. Must run before the arrival test in the same step:
        SUMO can begin and end a teleport within one step, so a responder can be dropped
        on the centre edge and register an arrival at the very time it teleported.
        """
        try:
            starting = traci.simulation.getStartingTeleportIDList()
        except (AttributeError, traci.TraCIException):
            return []
        return self.note_teleports(sim_time, starting)

    def get_teleport_summary(self):
        """Responder teleports and the events they invalidated, for the run report."""
        invalidated = sorted(
            event_id for event_id, event in self.events.items() if event.responder_teleported
        )
        return {
            "responder_teleport_count": len(self.responder_teleports),
            "responder_teleports": list(self.responder_teleports),
            "events_invalidated_by_teleport": invalidated,
        }

    def _get_blocked_edges(self, center_edge, radius=ROAD_BLOCK_RADIUS):
        """
        Get all edges affected by emergency (radius effect on connected roads).
        Returns a list of blocked edges.
        """
        blocked = [center_edge]
        
        try:
            # Get outgoing connections from center edge
            for i in range(radius):
                new_edges = set()
                for edge in blocked:
                    try:
                        # Get junctions at the end of this edge to find outgoing connections
                        # An edge has a 'to' junction where outgoing edges start
                        try:
                            # Get lanes of this edge and find the destination junction
                            lanes = traci.edge.getLaneNumber(edge)
                            if lanes > 0:
                                # Get the last lane to find outgoing edges
                                lane_id = f"{edge}_0"
                                # Get the position and link information
                                # Using the lane's outgoing connections through getLinks
                                outgoing_links = traci.lane.getLinks(lane_id)
                                for link in outgoing_links:
                                    connected_lane = link[0]
                                    if connected_lane:
                                        # Extract edge ID from lane ID (format: "edgeID_laneIndex")
                                        connected_edge = connected_lane.rsplit('_', 1)[0]
                                        if connected_edge not in blocked and connected_edge not in new_edges:
                                            new_edges.add(connected_edge)
                        except (traci.TraCIException, IndexError, ValueError):
                            pass
                    except traci.TraCIException:
                        pass
                        
                blocked.extend(list(new_edges))
                
                if not new_edges:
                    break
                    
        except traci.TraCIException:
            pass
            
        return list(set(blocked))
        
    def _edge_lane_ids(self, edge_id):
        """Return the lane IDs of an edge, or an empty list if the edge is unknown."""
        try:
            lane_count = traci.edge.getLaneNumber(edge_id)
        except traci.TraCIException as error:
            self.incident_errors.append(f"edge {edge_id}: {error}")
            return []
        return [f"{edge_id}_{index}" for index in range(lane_count)]

    def _apply_incident(self, event):
        """Apply the incident to SUMO so it physically affects traffic (audit B4).

        close_edge with capacity_factor 0.0 disallows every vehicle class except
        "emergency". A capacity_factor above 0 reduces the lane speed instead.
        """
        applied_lanes = 0
        for edge_id in event.blocked_edges:
            for lane_id in self._edge_lane_ids(edge_id):
                try:
                    saved = {
                        "disallowed": tuple(traci.lane.getDisallowed(lane_id)),
                        "max_speed": traci.lane.getMaxSpeed(lane_id),
                    }
                    if event.sumo_action == "close_edge" and event.capacity_factor <= 0.0:
                        traci.lane.setDisallowed(lane_id, CLOSURE_DISALLOWED_CLASSES)
                    else:
                        traci.lane.setMaxSpeed(lane_id, saved["max_speed"] * max(event.capacity_factor, 0.01))
                    self.saved_lane_state[(event.event_id, lane_id)] = saved
                    applied_lanes += 1
                except traci.TraCIException as error:
                    self.incident_errors.append(f"event {event.event_id} lane {lane_id}: {error}")

        if event.sumo_action == "close_edge" and event.capacity_factor <= 0.0:
            event.applied_action = f"closed to all classes except '{EMERGENCY_EXEMPT_VCLASS}'"
        else:
            event.applied_action = f"lane speed scaled by {event.capacity_factor}"
        return applied_lanes

    def _restore_incident(self, event):
        """Undo whatever _apply_incident changed, restoring the original lane state."""
        restored_lanes = 0
        for (event_id, lane_id), saved in list(self.saved_lane_state.items()):
            if event_id != event.event_id:
                continue
            try:
                traci.lane.setDisallowed(lane_id, list(saved["disallowed"]))
                traci.lane.setMaxSpeed(lane_id, saved["max_speed"])
                restored_lanes += 1
            except traci.TraCIException as error:
                self.incident_errors.append(f"restore event {event_id} lane {lane_id}: {error}")
            del self.saved_lane_state[(event_id, lane_id)]
        return restored_lanes

    def _restrict_ambient_road_access(self):
        """PD-1 question 2: switch each ambient emergency vType to `authority` once per run.

        Runs at the top of every update() until all four types are switched, so it always
        precedes the first closure, which only update() applies. SUMO reads route files about
        200 s ahead, so each type is switched long before its first vehicle is inserted. The
        switch changes no static permission (`authority` equals `emergency` on this network), so
        its moment matters only relative to closures.
        """
        if not self._ambient_types_pending or not self.restrict_ambient_road_access:
            return
        try:
            known = set(traci.vehicletype.getIDList())
        except traci.TraCIException:
            return
        for vehicle_type in sorted(self._ambient_types_pending & known):
            try:
                if traci.vehicletype.getVehicleClass(vehicle_type) == EMERGENCY_EXEMPT_VCLASS:
                    traci.vehicletype.setVehicleClass(vehicle_type, AMBIENT_EMERGENCY_ROAD_CLASS)
                self.ambient_road_classes[vehicle_type] = traci.vehicletype.getVehicleClass(vehicle_type)
            except traci.TraCIException as error:
                self.incident_errors.append(f"ambient type {vehicle_type}: {error}")
                continue
            self._ambient_types_pending.discard(vehicle_type)
            # A distance query type chosen before the switch would describe the old class.
            self._distance_query_types.pop(vehicle_type, None)

    def update(self, sim_time):
        """Update emergency events based on current simulation time"""
        if not self.generated:
            self.generate_emergencies()

        # PD-1 question 2: before anything can close a road this step.
        self._restrict_ambient_road_access()

        # Audit M12: check for responder teleports first, so a teleport that starts and
        # ends inside this step invalidates the arrival that the same step would record.
        self.poll_teleports(sim_time)

        # Activate events
        for event_id, event in self.events.items():
            if not event.is_active and event.start_time <= sim_time < event.end_time:
                event.activate()
                self.active_events.add(event_id)
                self.blocked_edges.update(event.blocked_edges)
                applied_lanes = self._apply_incident(event)
                print(
                    f"[EMERGENCY] Event {event_id} ACTIVATED at t={sim_time:.0f} - "
                    f"{len(event.blocked_edges)} edge(s), {applied_lanes} lane(s) {event.applied_action}; "
                    f"ends at t={event.end_time:.0f}"
                )

        # Deactivate events
        events_to_remove = []
        for event_id in self.active_events:
            event = self.events[event_id]
            if sim_time >= event.end_time:
                event.deactivate()
                events_to_remove.append(event_id)
                restored_lanes = self._restore_incident(event)
                print(
                    f"[EMERGENCY] Event {event_id} DEACTIVATED at t={sim_time:.0f} - "
                    f"{restored_lanes} lane(s) restored"
                )
                
        for event_id in events_to_remove:
            self.active_events.remove(event_id)
            
        # Update blocked edges
        self.blocked_edges.clear()
        for event_id in self.active_events:
            self.blocked_edges.update(self.events[event_id].blocked_edges)

        # PD-1: refresh the explicit responder flag for this step.
        self.active_responder_ids = frozenset(self.get_active_responder_ids(sim_time))

    def is_active_responder(self, vehicle_id):
        """True only for a vehicle dispatched to an incident it is currently responding to.

        PD-1, decided by the supervisor on 2026-09-13: "the vehicle class alone must not
        trigger emergency routing". Every emergency-*routing* decision reads this flag, never
        the vehicle type. What a vehicle may physically drive is SUMO's vehicle class; since
        PD-1 question 2 (same day) the ambient emergency types run as `authority`, so the
        closure exemption (decision 13.2) is held only by the dispatched responders.
        """
        return vehicle_id in self.active_responder_ids

    def is_edge_blocked(self, edge_id):
        """Check if an edge is currently blocked due to emergency"""
        return edge_id in self.blocked_edges
        
    def get_active_emergency_zones(self):
        """Get list of currently active emergency zones"""
        zones = {}
        for event_id in self.active_events:
            zones[event_id] = self._event_zone_dict(self.events[event_id])
        return zones
        
    def get_nearest_emergency(self, vehicle_id):
        """
        Find the nearest active emergency to a vehicle using route distance when possible.
        Returns (event_id, distance, emergency_data) or None
        """
        if not self.active_events:
            return None
            
        try:
            veh_edge = traci.vehicle.getRoadID(vehicle_id)
            try:
                vehicle_type = traci.vehicle.getTypeID(vehicle_id)
            except traci.TraCIException:
                vehicle_type = ""

            nearest = None
            min_distance = float('inf')
            # The road distance a car would drive to the incident if its edge were open. See
            # DISTANCE_PROBE_VCLASS: without this an ordinary car always got an empty route.
            query_type = self._distance_query_type(vehicle_type)

            for event_id in self.active_events:
                event = self.events[event_id]
                if veh_edge in event.blocked_edges:
                    distance = 0
                else:
                    distance = self._route_distance(veh_edge, event.center_edge, query_type)
                    
                if distance < min_distance:
                    min_distance = distance
                    nearest = (event_id, distance, event)
                    
            return nearest
            
        except traci.TraCIException:
            return None
            
    def register_response_vehicle(self, vehicle_id, event_id, current_time=None):
        """Register an emergency vehicle as responding to an event."""
        if event_id not in self.events:
            return None

        event = self.events[event_id]
        event.add_response_vehicle(vehicle_id)

        if event.first_arrival_time is None and current_time is not None:
            return self._record_arrival(event, vehicle_id, current_time, event.center_edge)

        return None

    @staticmethod
    def _record_arrival(event, vehicle_id, sim_time, arrival_edge):
        """Store the first valid responder arrival for an event, return its response time."""
        event.add_response_vehicle(vehicle_id)
        event.first_arrival_time = float(sim_time)
        event.first_arrival_vehicle = vehicle_id
        event.arrival_edge = arrival_edge
        event.response_time = float(sim_time) - float(event.start_time)
        event.arrived_after_incident_end = float(sim_time) > float(event.end_time)
        # Audit M12: an arrival reached by a responder that SUMO teleported is recorded so
        # it is visible in the report, but it is not a measurement.
        event.response_valid = not event.responder_teleported
        event.invalid_reason = None if event.response_valid else "responder_teleported"
        return event.response_time

    @staticmethod
    def _is_emergency_class(vehicle_id):
        try:
            return traci.vehicle.getVehicleClass(vehicle_id) == EMERGENCY_EXEMPT_VCLASS
        except traci.TraCIException:
            return False

    def _arrival_candidates(self, event, running_ids):
        """Vehicles allowed to supply this event's response time.

        When the scenario file names a dispatch vehicle, only that vehicle counts -
        contract section 7 forbids attributing an incident's response time to any other
        vehicle, which otherwise lets an ambient emergency vehicle driving past the
        incident register a spuriously short response. Randomly generated events have no
        named dispatch vehicle, so those fall back to any emergency-class vehicle.
        """
        if event.dispatch_vehicle_id:
            return [event.dispatch_vehicle_id] if event.dispatch_vehicle_id in running_ids else []
        return [vehicle_id for vehicle_id in sorted(running_ids) if self._is_emergency_class(vehicle_id)]

    def record_responder_arrivals(self, sim_time, vehicle_ids=None, arrived_ids=None):
        """Record responder arrivals at incident centre edges. Returns {event_id: response_time}.

        Deliberately iterates every loaded event, not just self.active_events. Response
        time is defined by contract sections 7 and 8 as incident activation until the
        dispatched responder's road ID equals the incident's centre edge; nothing in that
        definition requires the incident window to still be open. Gating detection on
        active events (audit finding M9) silently dropped every response slower than its
        own incident duration - in the 4,000-vehicle scenario, all three of them.
        """
        running_ids = set(vehicle_ids) if vehicle_ids is not None else set(traci.vehicle.getIDList())
        finished_ids = set(arrived_ids) if arrived_ids is not None else set()
        newly_recorded = {}

        for event_id, event in self.events.items():
            if event.first_arrival_time is not None or sim_time < event.start_time:
                continue

            recorded = None
            for vehicle_id in self._arrival_candidates(event, running_ids):
                try:
                    current_edge = traci.vehicle.getRoadID(vehicle_id)
                except traci.TraCIException:
                    continue
                if current_edge == event.center_edge:
                    recorded = self._record_arrival(event, vehicle_id, sim_time, current_edge)
                    break

            # A responder whose route ends on the centre edge can finish and leave the
            # simulation in the same step, so it is never seen in getIDList() there.
            if recorded is None and event.dispatch_vehicle_id in finished_ids:
                recorded = self._record_arrival(
                    event, event.dispatch_vehicle_id, sim_time, event.center_edge
                )

            # Audit M12: a response reached after the responder teleported is recorded on
            # the event and reported by get_response_summary() as invalid, but it is not
            # returned here - what this returns is what the runners feed to the metrics.
            if recorded is not None and event.response_valid:
                newly_recorded[event_id] = recorded

        return newly_recorded

    def get_event_response_time(self, event_id):
        """
        Get response time for an event (time until first vehicle arrives).
        Returns (response_time, first_vehicle_id) or None if no response yet.
        """
        if event_id not in self.events:
            return None

        event = self.events[event_id]
        if event.first_arrival_time is not None:
            return event.response_time, event.first_arrival_vehicle

        return None

    def get_event_snapshot(self, event_id):
        """Describe one event in the same shape as get_active_emergency_zones() values."""
        if event_id not in self.events:
            return None
        return self._event_zone_dict(self.events[event_id])

    @staticmethod
    def _event_zone_dict(event):
        return {
            "type": event.emergency_type,
            "center_edge": event.center_edge,
            "blocked_edges": event.blocked_edges,
            "start_time": event.start_time,
            "end_time": event.end_time,
            "sumo_action": event.sumo_action,
            "capacity_factor": event.capacity_factor,
            "applied_action": event.applied_action,
        }

    def get_response_summary(self):
        """Per-event response report, including events answered after their window closed.

        Audit M12: `response_time_s` is the reportable measurement and is None when the
        responder was teleported, so anything that filters on it cannot aggregate an
        artifact. The number that was observed is still published, as
        `unvalidated_response_time_s`, next to the reason it does not count.
        """
        summary = {}
        for event_id, event in self.events.items():
            measured = event.first_arrival_time is not None
            valid = measured and not event.responder_teleported
            summary[event_id] = {
                "event_id": event_id,
                "start_time": event.start_time,
                "end_time": event.end_time,
                "center_edge": event.center_edge,
                "dispatch_vehicle_id": event.dispatch_vehicle_id,
                "measured": measured,
                "first_arrival_time": event.first_arrival_time,
                "first_arrival_vehicle": event.first_arrival_vehicle,
                "arrival_edge": event.arrival_edge,
                "response_time_s": event.response_time if valid else None,
                "unvalidated_response_time_s": event.response_time,
                "response_valid": valid,
                "responder_teleported": event.responder_teleported,
                "responder_teleport_times_s": list(event.responder_teleport_times),
                "invalid_reason": (
                    "responder_teleported" if measured and event.responder_teleported else None
                ),
                "arrived_after_incident_end": event.arrived_after_incident_end,
            }
        return summary


# ===============================
# GLOBAL EMERGENCY MANAGER
# ===============================

emergency_manager = EmergencyManager()


def get_emergency_manager():
    """Get or create the global emergency manager instance"""
    return emergency_manager
