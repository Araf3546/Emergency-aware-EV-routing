import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import traci
from emergency_manager import get_emergency_manager
from p3_simulation_config import EMERGENCY_VEHICLE_TYPES

# ===============================
# REROUTE CONFIGURATION
# ===============================

REROUTE_INTERVAL = 30          # seconds
MIN_SPEED_THRESHOLD = 2.0      # m/s (congestion indicator)
QUEUE_THRESHOLD = 3            # vehicles on edge

# Emergency vehicle categories come from p3_simulation_config so there is one definition.
# This module used to keep its own four-name copy, which did not include the ambient
# emergency fleet's prefixed vType ids (audit R5, constants duplicated and disagreeing).

# ===============================
# CORE LOGIC
# ===============================

def should_reroute(veh_id):
    """
    Decide whether a vehicle should be rerouted.
    Used by ACO, Dijkstra, A*, Hybrid.
    
    For an ACTIVE RESPONDER on a blocked road: reroute at once, every step.
    For every other vehicle: the congestion-based check on the 30 s clock.

    PD-1 (supervisor decision, 2026-09-13): "the vehicle class alone must not trigger
    emergency routing". The every-step clause used to fire for any emergency-*class* vehicle on
    a closed edge. In practice that was one ambient vehicle, `emergency_00062`, sitting on
    incident 401's closed centre edge: 6 (layer off) and 7 (layer on) off-clock decisions per
    hour at seed 20260721, every one with a single candidate and a rejected route write. It
    now fires only for a vehicle the emergency manager has flagged as an active responder.
    """

    sim_time = int(traci.simulation.getTime())

    try:
        current_edge = traci.vehicle.getRoadID(veh_id)
    except traci.TraCIException:
        return False

    # Ignore internal junction edges
    if current_edge.startswith(":"):
        return False

    # Active responders only - never decided from the vehicle type (PD-1).
    emergency_mgr = get_emergency_manager()
    if emergency_mgr.is_active_responder(veh_id) and emergency_mgr.is_edge_blocked(current_edge):
        return True

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

