"""
Zone Mapper: Maps geographic zones to SUMO network edges
Allows conversion of zone-based OD data to edge-based SUMO trips
"""

import xml.etree.ElementTree as ET
import random
from typing import List, Dict, Set, Optional


class ZoneMapper:
    """Maps geographic zones/districts to SUMO edges"""
    
    def __init__(self, network_file: str):
        """
        Initialize zone mapper with SUMO network
        
        Args:
            network_file: Path to network.net.xml
        """
        self.edges = self._load_edges(network_file)
        self.zones = {}  # zone_id -> list of edge_ids
        self.edge_to_zone = {}  # edge_id -> zone_id
    
    def _load_edges(self, network_file: str) -> Dict:
        """Load all edges from SUMO network"""
        try:
            tree = ET.parse(network_file)
            root = tree.getroot()
        except Exception as e:
            print(f"Error loading network file: {e}")
            return {}
        
        edges = {}
        
        for edge in root.findall('.//edge[@id]'):
            edge_id = edge.get('id')
            
            # Skip internal edges (junctions)
            if edge_id.startswith(':'):
                continue
            
            edges[edge_id] = {
                'id': edge_id,
                'from': edge.get('from'),
                'to': edge.get('to'),
            }
        
        print(f"✓ Loaded {len(edges)} edges from network")
        return edges
    
    def add_zone(self, zone_id: str, edge_ids: List[str]):
        """
        Manually add a zone with its edges
        
        Args:
            zone_id: Zone identifier
            edge_ids: List of edge IDs in this zone
        """
        self.zones[zone_id] = edge_ids
        for edge_id in edge_ids:
            self.edge_to_zone[edge_id] = zone_id
        print(f"✓ Added zone '{zone_id}' with {len(edge_ids)} edges")
    
    def create_zone_from_edges(self, zone_id: str, edge_ids: List[str]):
        """Create a zone from a list of edge IDs"""
        self.add_zone(zone_id, edge_ids)
    
    def zone_to_edges(self, zone_id: str) -> List[str]:
        """
        Get all edges belonging to a zone
        
        Args:
            zone_id: Zone identifier
            
        Returns:
            List of edge IDs in the zone
        """
        return self.zones.get(zone_id, [])
    
    def get_random_edge_in_zone(self, zone_id: str) -> Optional[str]:
        """
        Get a random edge within a zone
        
        Args:
            zone_id: Zone identifier
            
        Returns:
            Random edge ID from the zone, or None if zone doesn't exist
        """
        edges = self.zone_to_edges(zone_id)
        if edges:
            return random.choice(edges)
        return None
    
    def get_zone_of_edge(self, edge_id: str) -> Optional[str]:
        """Get the zone an edge belongs to"""
        return self.edge_to_zone.get(edge_id)
    
    def get_all_zones(self) -> List[str]:
        """Get list of all defined zones"""
        return list(self.zones.keys())
    
    def import_zones_from_csv(self, csv_file: str):
        """
        Import zone definitions from CSV file
        
        CSV format:
        zone_id,edge_id
        zone_1,edge_a
        zone_1,edge_b
        zone_2,edge_c
        
        Args:
            csv_file: Path to CSV file with zone-edge mappings
        """
        import csv
        
        try:
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    zone_id = row['zone_id'].strip()
                    edge_id = row['edge_id'].strip()
                    
                    if zone_id not in self.zones:
                        self.zones[zone_id] = []
                    
                    self.zones[zone_id].append(edge_id)
                    self.edge_to_zone[edge_id] = zone_id
            
            print(f"✓ Imported zones from {csv_file}")
        except Exception as e:
            print(f"Error importing zones: {e}")
    
    def export_zones_to_csv(self, output_file: str):
        """Export zone definitions to CSV"""
        import csv
        
        try:
            with open(output_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['zone_id', 'edge_id'])
                
                for zone_id, edge_ids in self.zones.items():
                    for edge_id in edge_ids:
                        writer.writerow([zone_id, edge_id])
            
            print(f"✓ Exported zones to {output_file}")
        except Exception as e:
            print(f"Error exporting zones: {e}")
    
    def auto_create_grid_zones(self, rows: int, cols: int):
        """
        Automatically divide network into grid zones
        
        Args:
            rows: Number of rows
            cols: Number of columns
        """
        if not self.edges:
            print("No edges loaded!")
            return
        
        # Get boundary coordinates
        min_x = min_y = float('inf')
        max_x = max_y = float('-inf')
        
        # Simple approach: use edge IDs to estimate grid
        zone_count = 0
        for edge_id in self.edges.keys():
            zone_id = f"zone_{zone_count % (rows * cols)}"
            if zone_id not in self.zones:
                self.zones[zone_id] = []
            self.zones[zone_id].append(edge_id)
            self.edge_to_zone[edge_id] = zone_id
            zone_count += 1
        
        print(f"✓ Created {rows}x{cols} grid zones")
    
    def print_summary(self):
        """Print summary of zone definitions"""
        print("\n" + "="*50)
        print("ZONE MAPPING SUMMARY")
        print("="*50)
        print(f"Total zones: {len(self.zones)}")
        print(f"Total edges: {len(self.edges)}")
        
        for zone_id in sorted(self.zones.keys()):
            edge_count = len(self.zones[zone_id])
            print(f"  {zone_id}: {edge_count} edges")
        
        print("="*50 + "\n")


if __name__ == "__main__":
    # Example usage
    mapper = ZoneMapper('network.net.xml')
    
    # Option 1: Load zones from CSV
    # mapper.import_zones_from_csv('zone_definitions.csv')
    
    # Option 2: Create zones manually
    # mapper.add_zone('downtown', ['edge1', 'edge2', 'edge3'])
    # mapper.add_zone('residential', ['edge4', 'edge5'])
    
    # Option 3: Auto-create grid zones
    mapper.auto_create_grid_zones(rows=4, cols=4)
    
    mapper.print_summary()
