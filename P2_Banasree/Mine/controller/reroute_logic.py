import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import traci
from emergency_manager import get_emergency_manager

# ===============================
# REROUTE CONFIGURATION
# ===============================

REROUTE_INTERVAL = 30          # seconds
MIN_SPEED_THRESHOLD = 2.0      # m/s (congestion indicator)
QUEUE_THRESHOLD = 3            # vehicles on edge

# Emergency vehicle categories
EMERGENCY_VEHICLE_TYPES = ["ambulance", "fire_truck", "police", "relief_vehicle"]

# ===============================
# CORE LOGIC
# ===============================

def should_reroute(veh_id):
    """
    Decide whether a vehicle should be rerouted.
    Used by ACO, Dijkstra, A*, Hybrid.
    
    For emergency vehicles: Always reroute around blocked roads
    For normal vehicles: Standard congestion-based rerouting
    """

    sim_time = int(traci.simulation.getTime())

    try:
        veh_type = traci.vehicle.getTypeID(veh_id)
        is_emergency = veh_type in EMERGENCY_VEHICLE_TYPES
        current_edge = traci.vehicle.getRoadID(veh_id)
    except traci.TraCIException:
        return False

    # Ignore internal junction edges
    if current_edge.startswith(":"):
        return False

    # For emergency vehicles - check if on blocked road
    if is_emergency:
        emergency_mgr = get_emergency_manager()
        if emergency_mgr.is_edge_blocked(current_edge):
            return True  # Always reroute emergency vehicles away from blocked roads

    # Standard time-based rerouting for all vehicles
    if sim_time % REROUTE_INTERVAL != 0:
        return False

    # 3️⃣ Congestion-based trigger
    try:
        edge_speed = traci.edge.getLastStepMeanSpeed(current_edge)
        edge_queue = traci.edge.getLastStepVehicleNumber(current_edge)
    except traci.TraCIException:
        return False

    if edge_speed < MIN_SPEED_THRESHOLD:
        return True

    if edge_queue >= QUEUE_THRESHOLD:
        return True

    return False


def is_emergency_vehicle(veh_id):
    """Check if a vehicle is an emergency vehicle"""
    try:
        veh_type = traci.vehicle.getTypeID(veh_id)
        return veh_type in EMERGENCY_VEHICLE_TYPES
    except traci.TraCIException:
        return False


def get_vehicle_type(veh_id):
    """Get the type of a vehicle (passenger or emergency type)"""
    try:
        return traci.vehicle.getTypeID(veh_id)
    except traci.TraCIException:
        return "unknown"


def emergency_detected(vehicle_id):
    """
    Check if an emergency is nearby for this vehicle.
    Can be extended later for V2V messages, siren signals, etc.
    """
    emergency_mgr = get_emergency_manager()
    nearest = emergency_mgr.get_nearest_emergency(vehicle_id)
    return nearest is not None

