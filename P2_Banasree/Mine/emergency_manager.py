import random
import traci

# ===============================
# EMERGENCY CONFIGURATION
# ===============================

EMERGENCY_TYPES = ["accident", "fire"]
EMERGENCY_VEHICLES = ["ambulance", "fire_truck", "police", "relief_vehicle"]

# Simulation parameters
TOTAL_SIM_TIME = 7000          # seconds
EMERGENCY_COUNT = 3            # 3-4 emergencies (we'll use 3 as base)
EMERGENCY_DURATION = 100       # seconds
MIN_EMERGENCY_INTERVAL = 500   # minimum time between emergencies

# Road blocking parameters
ROAD_BLOCK_RADIUS = 3          # number of connected road segments affected
SPEED_REDUCTION_FACTOR = 0.3   # blocked roads: 30% of normal speed

# ===============================
# EMERGENCY EVENT CLASS
# ===============================

class EmergencyEvent:
    def __init__(self, event_id, start_time, emergency_type, edge_id, blocked_edges=None):
        self.event_id = event_id
        self.start_time = start_time
        self.end_time = start_time + EMERGENCY_DURATION
        self.emergency_type = emergency_type
        self.center_edge = edge_id
        self.blocked_edges = blocked_edges if blocked_edges else []
        self.is_active = False
        self.response_vehicles = set()  # emergency vehicles responding to this event
        self.first_arrival_time = None
        self.first_arrival_vehicle = None
        
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
    def __init__(self):
        self.events = {}
        self.event_counter = 0
        self.active_events = set()
        self.blocked_edges = set()
        self.generated = False
        
    def generate_emergencies(self):
        """Generate random emergency events during simulation"""
        if self.generated:
            return
            
        emergency_times = []
        
        # Generate 3-4 random emergency times with minimum interval
        for i in range(EMERGENCY_COUNT):
            if i == 0:
                start_time = random.randint(500, 1500)  # First emergency after 500-1500 sec
            else:
                # Ensure minimum interval between emergencies
                min_time = emergency_times[-1] + MIN_EMERGENCY_INTERVAL
                max_time = min(min_time + 1000, TOTAL_SIM_TIME - EMERGENCY_DURATION - 500)
                
                if max_time > min_time:
                    start_time = random.randint(int(min_time), int(max_time))
                else:
                    continue
                    
            emergency_times.append(start_time)
            
            # Pick random emergency type and location
            emergency_type = random.choice(EMERGENCY_TYPES)
            
            # Get random edge from network (we'll pick from available edges)
            try:
                available_edges = traci.edge.getIDList()
                if available_edges:
                    center_edge = random.choice(available_edges)
                    blocked_edges = self._get_blocked_edges(center_edge)
                    
                    event = EmergencyEvent(
                        event_id=self.event_counter,
                        start_time=start_time,
                        emergency_type=emergency_type,
                        edge_id=center_edge,
                        blocked_edges=blocked_edges
                    )
                    
                    self.events[self.event_counter] = event
                    self.event_counter += 1
                    
                    print(f"[EMERGENCY] Generated event {event.event_id}: {emergency_type} at edge {center_edge} @ {start_time}s")
                    
            except traci.TraCIException:
                pass
                
        self.generated = True
        
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
        
    def update(self, sim_time):
        """Update emergency events based on current simulation time"""
        if not self.generated:
            self.generate_emergencies()
            
        # Activate events
        for event_id, event in self.events.items():
            if not event.is_active and event.start_time <= sim_time < event.end_time:
                event.activate()
                self.active_events.add(event_id)
                self.blocked_edges.update(event.blocked_edges)
                print(f"[EMERGENCY] Event {event_id} ACTIVATED - Blocking {len(event.blocked_edges)} edges")
                
        # Deactivate events
        events_to_remove = []
        for event_id in self.active_events:
            event = self.events[event_id]
            if sim_time >= event.end_time:
                event.deactivate()
                events_to_remove.append(event_id)
                print(f"[EMERGENCY] Event {event_id} DEACTIVATED - Roads cleared")
                
        for event_id in events_to_remove:
            self.active_events.remove(event_id)
            
        # Update blocked edges
        self.blocked_edges.clear()
        for event_id in self.active_events:
            self.blocked_edges.update(self.events[event_id].blocked_edges)
            
    def is_edge_blocked(self, edge_id):
        """Check if an edge is currently blocked due to emergency"""
        return edge_id in self.blocked_edges
        
    def get_active_emergency_zones(self):
        """Get list of currently active emergency zones"""
        zones = {}
        for event_id in self.active_events:
            event = self.events[event_id]
            zones[event_id] = {
                "type": event.emergency_type,
                "center_edge": event.center_edge,
                "blocked_edges": event.blocked_edges,
                "start_time": event.start_time,
                "end_time": event.end_time
            }
        return zones
        
    def get_nearest_emergency(self, vehicle_id):
        """
        Find the nearest active emergency to a vehicle.
        Returns (event_id, distance, emergency_data) or None
        """
        if not self.active_events:
            return None
            
        try:
            veh_edge = traci.vehicle.getRoadID(vehicle_id)
            
            nearest = None
            min_distance = float('inf')
            
            for event_id in self.active_events:
                event = self.events[event_id]
                
                # Simple distance calculation (could be improved with actual routing)
                if veh_edge in event.blocked_edges:
                    distance = 0
                else:
                    # Default distance if not in blocked zone
                    distance = 1000
                    
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
            event.first_arrival_time = current_time
            event.first_arrival_vehicle = vehicle_id
            return event.first_arrival_time - event.start_time

        return None

    def get_event_response_time(self, event_id):
        """
        Get response time for an event (time until first vehicle arrives).
        Returns (response_time, first_vehicle_id) or None if no response yet.
        """
        if event_id not in self.events:
            return None

        event = self.events[event_id]
        if event.first_arrival_time is not None:
            response_time = event.first_arrival_time - event.start_time
            return response_time, event.first_arrival_vehicle

        return None


# ===============================
# GLOBAL EMERGENCY MANAGER
# ===============================

emergency_manager = EmergencyManager()


def get_emergency_manager():
    """Get or create the global emergency manager instance"""
    return emergency_manager
