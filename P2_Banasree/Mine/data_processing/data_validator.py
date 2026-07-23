"""
Data Validation: Validate real-world data conversions
"""

import xml.etree.ElementTree as ET
from typing import List, Dict, Tuple
import csv


class DataValidator:
    """Validate converted real-world data"""
    
    def __init__(self, network_file: str):
        """
        Initialize validator
        
        Args:
            network_file: Path to network.net.xml
        """
        self.network_file = network_file
        self.valid_edges = self._load_valid_edges()
        self.issues = []
    
    def _load_valid_edges(self) -> set:
        """Load all valid edge IDs from network"""
        try:
            tree = ET.parse(self.network_file)
            root = tree.getroot()
        except Exception as e:
            print(f"Error loading network: {e}")
            return set()
        
        edges = set()
        for edge in root.findall('.//edge[@id]'):
            edge_id = edge.get('id')
            if not edge_id.startswith(':'):  # Skip internal edges
                edges.add(edge_id)
        
        return edges
    
    def validate_trips(self, trips_file: str) -> Tuple[bool, Dict]:
        """
        Validate trips XML file
        
        Args:
            trips_file: Path to trips XML file
            
        Returns:
            (is_valid, report_dict)
        """
        self.issues = []
        report = {
            'total_trips': 0,
            'valid_trips': 0,
            'invalid_from_edges': [],
            'invalid_to_edges': [],
            'invalid_types': [],
            'negative_times': [],
            'duplicate_ids': set()
        }
        
        try:
            tree = ET.parse(trips_file)
            root = tree.getroot()
        except Exception as e:
            print(f"Error parsing trips file: {e}")
            return False, report
        
        seen_ids = set()
        vehicle_types = set()
        
        for trip in root.findall('.//trip'):
            report['total_trips'] += 1
            trip_id = trip.get('id')
            from_edge = trip.get('from')
            to_edge = trip.get('to')
            depart_time = float(trip.get('depart', 0))
            trip_type = trip.get('type', 'passenger')
            
            is_valid = True
            
            # Check for duplicate IDs
            if trip_id in seen_ids:
                report['duplicate_ids'].add(trip_id)
                is_valid = False
            seen_ids.add(trip_id)
            
            # Check edges
            if from_edge not in self.valid_edges:
                report['invalid_from_edges'].append(from_edge)
                is_valid = False
            
            if to_edge not in self.valid_edges:
                report['invalid_to_edges'].append(to_edge)
                is_valid = False
            
            # Check departure time
            if depart_time < 0:
                report['negative_times'].append(depart_time)
                is_valid = False
            
            # Track vehicle types
            vehicle_types.add(trip_type)
            
            if is_valid:
                report['valid_trips'] += 1
        
        report['vehicle_types'] = list(vehicle_types)
        report['unique_invalid_from'] = len(set(report['invalid_from_edges']))
        report['unique_invalid_to'] = len(set(report['invalid_to_edges']))
        
        is_valid = (
            report['valid_trips'] == report['total_trips'] and
            len(report['duplicate_ids']) == 0
        )
        
        return is_valid, report
    
    def validate_routes(self, routes_file: str) -> Tuple[bool, Dict]:
        """
        Validate routes XML file
        
        Args:
            routes_file: Path to routes XML file
            
        Returns:
            (is_valid, report_dict)
        """
        self.issues = []
        report = {
            'total_vehicles': 0,
            'valid_vehicles': 0,
            'invalid_edges': [],
            'incomplete_routes': [],
            'missing_routes': []
        }
        
        try:
            tree = ET.parse(routes_file)
            root = tree.getroot()
        except Exception as e:
            print(f"Error parsing routes file: {e}")
            return False, report
        
        for vehicle in root.findall('.//vehicle'):
            report['total_vehicles'] += 1
            vehicle_id = vehicle.get('id')
            
            route = vehicle.find('route')
            if route is None:
                report['missing_routes'].append(vehicle_id)
                continue
            
            edges_str = route.get('edges', '')
            if not edges_str:
                report['incomplete_routes'].append(vehicle_id)
                continue
            
            edges = edges_str.split()
            is_valid = True
            
            for edge in edges:
                if edge not in self.valid_edges:
                    report['invalid_edges'].append(edge)
                    is_valid = False
            
            if is_valid:
                report['valid_vehicles'] += 1
        
        is_valid = (
            len(report['invalid_edges']) == 0 and
            len(report['missing_routes']) == 0 and
            len(report['incomplete_routes']) == 0
        )
        
        return is_valid, report
    
    def validate_od_matrix(self, od_file: str) -> Tuple[bool, Dict]:
        """
        Validate OD matrix file
        
        Args:
            od_file: Path to OD matrix CSV file
            
        Returns:
            (is_valid, report_dict)
        """
        report = {
            'total_entries': 0,
            'valid_entries': 0,
            'missing_zones': [],
            'negative_counts': [],
            'invalid_types': []
        }
        
        try:
            with open(od_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    report['total_entries'] += 1
                    
                    try:
                        count = int(row.get('vehicle_count', 0))
                        if count < 0:
                            report['negative_counts'].append(count)
                    except ValueError:
                        report['invalid_types'].append(row.get('vehicle_count'))
                    
                    if count >= 0:
                        report['valid_entries'] += 1
        
        except Exception as e:
            print(f"Error reading OD file: {e}")
            return False, report
        
        is_valid = (
            len(report['negative_counts']) == 0 and
            len(report['invalid_types']) == 0
        )
        
        return is_valid, report
    
    def print_validation_report(self, report: Dict, report_type: str = "trips"):
        """Print formatted validation report"""
        print("\n" + "="*60)
        print(f"VALIDATION REPORT - {report_type.upper()}")
        print("="*60)
        
        if report_type == "trips":
            print(f"Total trips: {report.get('total_trips', 0)}")
            print(f"Valid trips: {report.get('valid_trips', 0)}")
            print(f"Vehicle types: {report.get('vehicle_types', [])}")
            
            if report.get('invalid_from_edges'):
                print(f"\n⚠ Invalid 'from' edges: {report['unique_invalid_from']}")
                print(f"  Examples: {list(set(report['invalid_from_edges']))[:5]}")
            
            if report.get('invalid_to_edges'):
                print(f"⚠ Invalid 'to' edges: {report['unique_invalid_to']}")
                print(f"  Examples: {list(set(report['invalid_to_edges']))[:5]}")
            
            if report.get('duplicate_ids'):
                print(f"⚠ Duplicate IDs: {len(report['duplicate_ids'])}")
            
            if report.get('negative_times'):
                print(f"⚠ Negative departure times: {len(report['negative_times'])}")
        
        elif report_type == "routes":
            print(f"Total vehicles: {report.get('total_vehicles', 0)}")
            print(f"Valid vehicles: {report.get('valid_vehicles', 0)}")
            
            if report.get('missing_routes'):
                print(f"\n⚠ Missing routes: {len(report['missing_routes'])}")
            
            if report.get('incomplete_routes'):
                print(f"⚠ Incomplete routes: {len(report['incomplete_routes'])}")
            
            if report.get('invalid_edges'):
                print(f"⚠ Invalid edges: {len(set(report['invalid_edges']))}")
                print(f"  Examples: {list(set(report['invalid_edges']))[:5]}")
        
        elif report_type == "od_matrix":
            print(f"Total entries: {report.get('total_entries', 0)}")
            print(f"Valid entries: {report.get('valid_entries', 0)}")
            
            if report.get('negative_counts'):
                print(f"\n⚠ Negative vehicle counts: {len(report['negative_counts'])}")
            
            if report.get('invalid_types'):
                print(f"⚠ Invalid data types: {len(report['invalid_types'])}")
        
        print("="*60 + "\n")


class DataComparison:
    """Compare metrics between random and real-world data"""
    
    @staticmethod
    def compare_trip_distributions(random_trips: str, realworld_trips: str) -> Dict:
        """
        Compare trip distributions
        
        Args:
            random_trips: Path to random trips XML
            realworld_trips: Path to real-world trips XML
            
        Returns:
            Comparison dictionary
        """
        def extract_distribution(trips_file):
            tree = ET.parse(trips_file)
            root = tree.getroot()
            
            departures = []
            types = {}
            
            for trip in root.findall('.//trip'):
                depart = float(trip.get('depart', 0))
                trip_type = trip.get('type', 'passenger')
                departures.append(depart)
                types[trip_type] = types.get(trip_type, 0) + 1
            
            return departures, types
        
        random_deps, random_types = extract_distribution(random_trips)
        real_deps, real_types = extract_distribution(realworld_trips)
        
        return {
            'random_count': len(random_deps),
            'realworld_count': len(real_deps),
            'random_types': random_types,
            'realworld_types': real_types,
            'random_avg_depart': sum(random_deps) / len(random_deps) if random_deps else 0,
            'realworld_avg_depart': sum(real_deps) / len(real_deps) if real_deps else 0,
        }


if __name__ == "__main__":
    # Example usage
    validator = DataValidator('network.net.xml')
    
    # Validate trips
    is_valid, report = validator.validate_trips('trips.trips.xml')
    validator.print_validation_report(report, "trips")
    
    # Validate routes
    is_valid, report = validator.validate_routes('routes.rou.xml')
    validator.print_validation_report(report, "routes")
