#!/usr/bin/env python3
"""
Script to add emergency vehicles to trips.trips.xml
Adds 10% emergency vehicles (ambulance, fire_truck, police, relief_vehicle)
to the existing 90% passenger vehicles
"""

import xml.etree.ElementTree as ET
import random
from pathlib import Path

# Configuration
VEHICLE_TYPES = ["ambulance", "fire_truck", "police", "relief_vehicle"]
EMERGENCY_PERCENTAGE = 0.10  # 10% emergency vehicles
PASSENGER_PERCENTAGE = 0.90  # 90% passenger vehicles
SIMULATION_TIME = 7000  # seconds
EXISTING_PASSENGERS = 3600  # vehicles 0-3599

def add_emergency_vehicles():
    """Add emergency vehicle types and trips to routes file"""
    
    trips_file = Path("trips.trips.xml")
    
    # Parse existing XML
    tree = ET.parse(trips_file)
    root = tree.getroot()
    
    # Find the routes element
    if root.tag != "routes":
        print("Error: Root element is not 'routes'")
        return
    
    # Add emergency vehicle types if not already present
    existing_types = {vtype.get('id') for vtype in root.findall('vType')}
    
    if 'ambulance' not in existing_types:
        ambulance = ET.Element('vType')
        ambulance.set('id', 'ambulance')
        ambulance.set('vClass', 'emergency')
        ambulance.set('color', 'white')
        root.insert(1, ambulance)
        print("Added ambulance vehicle type")
    
    if 'fire_truck' not in existing_types:
        fire_truck = ET.Element('vType')
        fire_truck.set('id', 'fire_truck')
        fire_truck.set('vClass', 'emergency')
        fire_truck.set('color', 'red')
        root.insert(2, fire_truck)
        print("Added fire_truck vehicle type")
    
    if 'police' not in existing_types:
        police = ET.Element('vType')
        police.set('id', 'police')
        police.set('vClass', 'emergency')
        police.set('color', 'blue')
        root.insert(3, police)
        print("Added police vehicle type")
    
    if 'relief_vehicle' not in existing_types:
        relief = ET.Element('vType')
        relief.set('id', 'relief_vehicle')
        relief.set('vClass', 'emergency')
        relief.set('color', 'yellow')
        root.insert(4, relief)
        print("Added relief_vehicle vehicle type")
    
    # Get all existing trips to calculate edge distribution
    trips = list(root.findall('trip'))
    existing_count = len(trips)
    
    # Calculate how many emergency vehicles to add
    total_vehicles = int(existing_count / PASSENGER_PERCENTAGE)
    emergency_count = int(total_vehicles * EMERGENCY_PERCENTAGE)
    
    print(f"Existing passengers: {existing_count}")
    print(f"Total vehicles needed: {total_vehicles}")
    print(f"Emergency vehicles to add: {emergency_count}")
    
    # Create a list of available edges from existing trips
    from_edges = [trip.get('from') for trip in trips]
    to_edges = [trip.get('to') for trip in trips]
    available_edges = list(set(from_edges + to_edges))
    
    print(f"Available edges: {len(available_edges)}")
    
    # Generate emergency vehicle trips
    emergency_id_start = EXISTING_PASSENGERS
    last_trip = trips[-1]
    last_depart_time = float(last_trip.get('depart', 0))
    
    # Spread emergency vehicles throughout the simulation
    for i in range(emergency_count):
        # Random departure time (spread throughout simulation)
        depart_time = random.uniform(0, SIMULATION_TIME - 500)
        
        # Random emergency vehicle type
        veh_type = random.choice(VEHICLE_TYPES)
        
        # Random from and to edges
        from_edge = random.choice(available_edges)
        to_edge = random.choice(available_edges)
        
        # Ensure different from and to
        while to_edge == from_edge:
            to_edge = random.choice(available_edges)
        
        # Create trip element
        trip = ET.Element('trip')
        trip.set('id', str(emergency_id_start + i))
        trip.set('depart', f"{depart_time:.2f}")
        trip.set('from', from_edge)
        trip.set('to', to_edge)
        trip.set('type', veh_type)
        
        root.append(trip)
    
    # Sort trips by departure time
    trips_list = list(root.findall('trip'))
    trips_list.sort(key=lambda x: float(x.get('depart', 0)))
    
    # Remove all trip elements
    for trip in list(root.findall('trip')):
        root.remove(trip)
    
    # Re-add sorted trips
    for trip in trips_list:
        root.append(trip)
    
    # Write back to file
    ET.register_namespace('xsi', 'http://www.w3.org/2001/XMLSchema-instance')
    tree.write(trips_file, encoding='UTF-8', xml_declaration=True)
    print(f"Updated {trips_file} with emergency vehicles")
    print(f"Total trips now: {len(trips_list)}")

if __name__ == "__main__":
    add_emergency_vehicles()
    print("Emergency vehicle addition complete!")
