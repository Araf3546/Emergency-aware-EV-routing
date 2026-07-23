"""
OD Data Importer: Convert real-world OD matrices to SUMO trips
"""

import csv
import json
import random
from typing import List, Dict, Tuple, Optional
from data_processing.zone_mapper import ZoneMapper


class ODDataImporter:
    """Convert Origin-Destination matrix to SUMO trips"""
    
    def __init__(self, network_file: str, zone_mapper: Optional[ZoneMapper] = None):
        """
        Initialize OD importer
        
        Args:
            network_file: Path to network.net.xml
            zone_mapper: Optional pre-configured ZoneMapper
        """
        self.mapper = zone_mapper or ZoneMapper(network_file)
        self.od_data = []
        self.trips = []
    
    def load_od_matrix_csv(self, filepath: str):
        """
        Load OD matrix from CSV file
        
        CSV format (Option 1 - One trip per row):
        origin_zone,destination_zone,vehicle_count,departure_time,vehicle_type
        zone_1,zone_2,5,0,passenger
        zone_1,zone_3,3,100,passenger
        
        Args:
            filepath: Path to CSV file
        """
        self.od_data = []
        
        try:
            with open(filepath, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    entry = {
                        'origin_zone': row['origin_zone'].strip(),
                        'dest_zone': row['destination_zone'].strip(),
                        'vehicle_count': int(row.get('vehicle_count', 1)),
                        'departure_time': int(row.get('departure_time', 0)),
                        'vehicle_type': row.get('vehicle_type', 'passenger').strip()
                    }
                    self.od_data.append(entry)
            
            print(f"✓ Loaded {len(self.od_data)} OD entries from CSV")
        except Exception as e:
            print(f"Error loading CSV: {e}")
    
    def load_od_matrix_json(self, filepath: str):
        """
        Load OD matrix from JSON file
        
        JSON format:
        {
          "demand": [
            {
              "from": "zone_1",
              "to": "zone_2",
              "count": 10,
              "departure_time": 3600,
              "type": "passenger"
            }
          ]
        }
        
        Args:
            filepath: Path to JSON file
        """
        self.od_data = []
        
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            for demand in data.get('demand', []):
                entry = {
                    'origin_zone': demand['from'].strip(),
                    'dest_zone': demand['to'].strip(),
                    'vehicle_count': int(demand.get('count', 1)),
                    'departure_time': int(demand.get('departure_time', 0)),
                    'vehicle_type': demand.get('type', 'passenger').strip()
                }
                self.od_data.append(entry)
            
            print(f"✓ Loaded {len(self.od_data)} OD entries from JSON")
        except Exception as e:
            print(f"Error loading JSON: {e}")
    
    def load_hourly_distribution_json(self, filepath: str):
        """
        Load OD matrix with hourly distributions
        
        JSON format with hourly breakdown:
        {
          "demand": [
            {
              "from": "zone_1",
              "to": "zone_2",
              "hourly_distribution": [5, 3, 2, 1, 0, ...],
              "type": "passenger"
            }
          ]
        }
        
        Args:
            filepath: Path to JSON file
        """
        self.od_data = []
        
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            for demand in data.get('demand', []):
                hourly = demand.get('hourly_distribution', [])
                
                # Convert hourly distribution to individual trips
                for hour, count in enumerate(hourly):
                    for i in range(count):
                        # Distribute vehicles randomly within the hour
                        departure_time = (hour * 3600) + random.randint(0, 3599)
                        
                        entry = {
                            'origin_zone': demand['from'].strip(),
                            'dest_zone': demand['to'].strip(),
                            'vehicle_count': 1,
                            'departure_time': departure_time,
                            'vehicle_type': demand.get('type', 'passenger').strip()
                        }
                        self.od_data.append(entry)
            
            print(f"✓ Loaded {len(self.od_data)} trips from hourly distribution")
        except Exception as e:
            print(f"Error loading hourly distribution: {e}")
    
    def load_matrix_format(self, filepath: str, zone_list: List[str]):
        """
        Load OD matrix from matrix format
        
        Format:
              zone_1  zone_2  zone_3
        zone_1  0       5       3
        zone_2  4       0       2
        zone_3  1       2       0
        
        Args:
            filepath: Path to matrix file
            zone_list: List of zone IDs in order
        """
        self.od_data = []
        
        try:
            with open(filepath, 'r') as f:
                lines = f.readlines()
            
            # Skip header line if present
            start_idx = 1 if lines[0].strip().startswith(zone_list[0]) else 0
            
            for i, line in enumerate(lines[start_idx:]):
                values = line.strip().split()
                origin = values[0]
                
                for j, count in enumerate(values[1:]):
                    count = int(count)
                    if count > 0:
                        entry = {
                            'origin_zone': origin,
                            'dest_zone': zone_list[j] if j < len(zone_list) else None,
                            'vehicle_count': count,
                            'departure_time': 0,
                            'vehicle_type': 'passenger'
                        }
                        self.od_data.append(entry)
            
            print(f"✓ Loaded matrix format with {len(self.od_data)} OD pairs")
        except Exception as e:
            print(f"Error loading matrix: {e}")
    
    def generate_trips(self, time_window: Tuple[int, int] = (0, 7000)) -> List[Dict]:
        """
        Convert OD data to SUMO trips
        
        Args:
            time_window: (start_time, end_time) in seconds
            
        Returns:
            List of trip dictionaries
        """
        self.trips = []
        trip_id = 0
        errors = 0
        
        for od in self.od_data:
            origin_zone = od['origin_zone']
            dest_zone = od['dest_zone']
            vehicle_count = od['vehicle_count']
            base_time = od['departure_time']
            veh_type = od['vehicle_type']
            
            # Get edges for origin and destination zones
            origin_edges = self.mapper.zone_to_edges(origin_zone)
            dest_edges = self.mapper.zone_to_edges(dest_zone)
            
            if not origin_edges or not dest_edges:
                errors += 1
                continue
            
            # Create individual trips with slight time variations
            for i in range(vehicle_count):
                origin_edge = random.choice(origin_edges)
                dest_edge = random.choice(dest_edges)
                
                # Add some time variation to spread out departures
                depart_time = base_time + random.uniform(-30, 30)
                depart_time = max(time_window[0], min(time_window[1], depart_time))
                
                trip = {
                    'id': str(trip_id),
                    'from': origin_edge,
                    'to': dest_edge,
                    'depart': depart_time,
                    'type': veh_type
                }
                
                self.trips.append(trip)
                trip_id += 1
        
        print(f"✓ Generated {len(self.trips)} trips")
        if errors > 0:
            print(f"  ⚠ {errors} OD pairs skipped (zone/edge not found)")
        
        return self.trips
    
    def write_trips_xml(self, output_file: str, add_emergency_vehicles: bool = True):
        """
        Write trips to SUMO XML format
        
        Args:
            output_file: Path to output trips XML file
            add_emergency_vehicles: Whether to add emergency vehicle types
        """
        try:
            with open(output_file, 'w') as f:
                f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
                f.write('<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" ')
                f.write('xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n')
                
                # Add vehicle types
                f.write('    <vType id="passenger" vClass="passenger"/>\n')
                
                if add_emergency_vehicles:
                    f.write('    <vType id="ambulance" vClass="emergency" color="white"/>\n')
                    f.write('    <vType id="fire_truck" vClass="emergency" color="red"/>\n')
                    f.write('    <vType id="police" vClass="emergency" color="blue"/>\n')
                    f.write('    <vType id="relief_vehicle" vClass="emergency" color="yellow"/>\n')
                
                # Add trips
                for trip in self.trips:
                    f.write(f'    <trip id="{trip["id"]}" depart="{trip["depart"]:.2f}" ')
                    f.write(f'from="{trip["from"]}" to="{trip["to"]}" type="{trip["type"]}"/>\n')
                
                f.write('</routes>\n')
            
            print(f"✓ Wrote trips to {output_file}")
        except Exception as e:
            print(f"Error writing trips XML: {e}")
    
    def print_summary(self):
        """Print summary of loaded data and generated trips"""
        print("\n" + "="*60)
        print("OD DATA IMPORT SUMMARY")
        print("="*60)
        print(f"OD entries loaded: {len(self.od_data)}")
        print(f"Trips generated: {len(self.trips)}")
        
        if self.trips:
            vehicle_types = {}
            min_time = float('inf')
            max_time = 0
            
            for trip in self.trips:
                vtype = trip['type']
                vehicle_types[vtype] = vehicle_types.get(vtype, 0) + 1
                min_time = min(min_time, trip['depart'])
                max_time = max(max_time, trip['depart'])
            
            print(f"\nVehicle types:")
            for vtype, count in sorted(vehicle_types.items()):
                print(f"  {vtype}: {count}")
            
            print(f"\nDeparture time range: {min_time:.1f}s - {max_time:.1f}s")
        
        print("="*60 + "\n")


if __name__ == "__main__":
    # Example usage
    from data_processing.zone_mapper import ZoneMapper
    
    # Create mapper and add zones
    mapper = ZoneMapper('network.net.xml')
    
    # Load zones (you would do this from your zone definitions)
    # mapper.import_zones_from_csv('zones.csv')
    
    # Create importer
    importer = ODDataImporter('network.net.xml', mapper)
    
    # Load OD data (choose one format)
    # importer.load_od_matrix_csv('od_data.csv')
    # importer.load_od_matrix_json('od_data.json')
    # importer.load_hourly_distribution_json('od_hourly.json')
    
    # Generate trips and write to file
    # importer.generate_trips()
    # importer.write_trips_xml('trips_real_world.trips.xml')
    # importer.print_summary()
