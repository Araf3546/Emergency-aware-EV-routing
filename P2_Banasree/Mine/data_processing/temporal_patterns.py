"""
Temporal Patterns: Apply realistic daily/hourly patterns to trips
"""

import random
from typing import List, Dict


class TemporalDemandProfile:
    """Apply temporal patterns to OD matrix"""
    
    def __init__(self, daily_profile: List[float] = None):
        """
        Initialize temporal profile
        
        Args:
            daily_profile: List of 24 hourly factors (0.0 to 1.0)
                          where 1.0 = peak demand
                          Example: [0.2, 0.15, 0.1, 0.3, 0.7, 0.95, 1.0, ...]
        """
        # Default profile: typical urban traffic
        self.daily_profile = daily_profile or [
            0.15,  # 00:00-01:00
            0.10,  # 01:00-02:00
            0.08,  # 02:00-03:00
            0.10,  # 03:00-04:00
            0.20,  # 04:00-05:00
            0.40,  # 05:00-06:00
            0.75,  # 06:00-07:00 (morning rush starts)
            0.95,  # 07:00-08:00 (morning peak)
            1.00,  # 08:00-09:00 (morning peak)
            0.85,  # 09:00-10:00
            0.70,  # 10:00-11:00
            0.75,  # 11:00-12:00 (lunch)
            0.80,  # 12:00-13:00 (lunch)
            0.75,  # 13:00-14:00
            0.70,  # 14:00-15:00
            0.80,  # 15:00-16:00 (afternoon)
            0.95,  # 16:00-17:00 (evening rush starts)
            1.00,  # 17:00-18:00 (evening peak)
            0.85,  # 18:00-19:00
            0.60,  # 19:00-20:00
            0.45,  # 20:00-21:00
            0.35,  # 21:00-22:00
            0.25,  # 22:00-23:00
            0.18,  # 23:00-00:00
        ]
    
    @staticmethod
    def get_rush_hour_profile() -> List[float]:
        """Standard urban rush hour profile"""
        return [
            0.15,  # 00:00
            0.10,  # 01:00
            0.08,  # 02:00
            0.10,  # 03:00
            0.20,  # 04:00
            0.40,  # 05:00
            0.75,  # 06:00 (morning rush starts)
            0.95,  # 07:00 (morning peak)
            1.00,  # 08:00 (morning peak)
            0.85,  # 09:00
            0.70,  # 10:00
            0.75,  # 11:00
            0.80,  # 12:00 (lunch)
            0.75,  # 13:00
            0.70,  # 14:00
            0.80,  # 15:00
            0.95,  # 16:00 (evening rush starts)
            1.00,  # 17:00 (evening peak)
            0.85,  # 18:00
            0.60,  # 19:00
            0.45,  # 20:00
            0.35,  # 21:00
            0.25,  # 22:00
            0.18,  # 23:00
        ]
    
    @staticmethod
    def get_uniform_profile() -> List[float]:
        """Uniform distribution (no temporal variation)"""
        return [0.5] * 24
    
    @staticmethod
    def get_night_shift_profile() -> List[float]:
        """Night shift dominant (opposite of rush hour)"""
        return [
            0.80,  # 00:00
            0.85,  # 01:00
            0.90,  # 02:00
            0.85,  # 03:00
            0.70,  # 04:00
            0.45,  # 05:00
            0.25,  # 06:00
            0.15,  # 07:00
            0.10,  # 08:00
            0.20,  # 09:00
            0.35,  # 10:00
            0.30,  # 11:00
            0.25,  # 12:00
            0.30,  # 13:00
            0.35,  # 14:00
            0.25,  # 15:00
            0.20,  # 16:00
            0.15,  # 17:00
            0.30,  # 18:00
            0.50,  # 19:00
            0.65,  # 20:00
            0.75,  # 21:00
            0.80,  # 22:00
            0.85,  # 23:00
        ]
    
    def set_profile(self, profile: List[float]):
        """Set custom hourly profile"""
        if len(profile) != 24:
            raise ValueError("Profile must have exactly 24 values (one per hour)")
        self.daily_profile = profile
    
    def apply_scaling(self, trips: List[Dict], 
                     simulation_start_hour: int = 0) -> List[Dict]:
        """
        Apply temporal scaling to trips
        
        Args:
            trips: List of trip dictionaries with 'depart' field
            simulation_start_hour: Starting hour for simulation (0-23)
            
        Returns:
            Scaled trips
        """
        scaled_trips = []
        
        for trip in trips:
            # Extract hour from departure time
            depart_seconds = trip['depart']
            hour = int((depart_seconds // 3600) % 24)
            
            # Get scaling factor for this hour
            scale_factor = self.daily_profile[hour]
            
            # Apply stochastic scaling
            if random.random() < scale_factor:
                scaled_trips.append(trip)
        
        return scaled_trips
    
    def scale_od_matrix(self, od_data: List[Dict]) -> List[Dict]:
        """
        Apply temporal scaling to OD matrix
        
        Args:
            od_data: List of OD entries with 'departure_time' field
            
        Returns:
            Scaled OD data
        """
        scaled_data = []
        
        for od in od_data:
            # Extract hour from departure time
            depart_seconds = od['departure_time']
            hour = int((depart_seconds // 3600) % 24)
            
            # Get scaling factor
            scale_factor = self.daily_profile[hour]
            
            # Scale vehicle count
            scaled_count = int(od['vehicle_count'] * scale_factor)
            
            # Add variation
            if random.random() < (scale_factor * od['vehicle_count'] - scaled_count):
                scaled_count += 1
            
            if scaled_count > 0:
                scaled_od = od.copy()
                scaled_od['vehicle_count'] = scaled_count
                scaled_data.append(scaled_od)
        
        return scaled_data
    
    def print_profile(self):
        """Print visual representation of daily profile"""
        print("\nTEMPORAL PROFILE")
        print("="*50)
        print("Hour  | Factor | Traffic Level")
        print("-"*50)
        
        for hour, factor in enumerate(self.daily_profile):
            level = "█" * int(factor * 20)
            print(f"{hour:02d}:00 | {factor:5.2f}  | {level}")
        
        print("="*50)


class SeasonalProfile:
    """Handle seasonal variations in traffic"""
    
    def __init__(self):
        """Initialize seasonal factors"""
        # Traffic variation by month (1.0 = average)
        self.monthly_factors = {
            1: 0.95,   # January - winter
            2: 0.93,   # February - winter
            3: 0.97,   # March - spring
            4: 0.98,   # April - spring
            5: 1.05,   # May - spring holidays
            6: 1.10,   # June - summer holidays start
            7: 1.15,   # July - peak summer holidays
            8: 1.12,   # August - summer holidays
            9: 1.00,   # September - back to school
            10: 0.98,  # October - fall
            11: 0.95,  # November - fall
            12: 1.08,  # December - winter holidays
        }
        
        # Day of week factors
        self.dow_factors = {
            0: 1.00,  # Monday
            1: 1.00,  # Tuesday
            2: 1.00,  # Wednesday
            3: 1.00,  # Thursday
            4: 1.10,  # Friday - more traffic
            5: 0.80,  # Saturday - less traffic
            6: 0.70,  # Sunday - minimal traffic
        }
    
    def get_factor(self, month: int, day_of_week: int) -> float:
        """
        Get combined seasonal factor
        
        Args:
            month: Month (1-12)
            day_of_week: Day of week (0=Monday, 6=Sunday)
            
        Returns:
            Combined factor
        """
        month_factor = self.monthly_factors.get(month, 1.0)
        dow_factor = self.dow_factors.get(day_of_week, 1.0)
        return month_factor * dow_factor
    
    def apply_seasonal_scaling(self, trips: List[Dict], 
                              month: int, day_of_week: int) -> List[Dict]:
        """
        Apply seasonal scaling to trips
        
        Args:
            trips: List of trips
            month: Current month (1-12)
            day_of_week: Current day of week (0-6)
            
        Returns:
            Scaled trips
        """
        factor = self.get_factor(month, day_of_week)
        
        scaled_trips = []
        for trip in trips:
            if random.random() < factor:
                scaled_trips.append(trip)
        
        return scaled_trips


class WeatherAdjustment:
    """Adjust traffic patterns based on weather"""
    
    # Weather condition factors (relative to normal)
    WEATHER_FACTORS = {
        'clear': 1.0,
        'cloudy': 1.0,
        'rain': 0.85,          # Rain reduces traffic
        'heavy_rain': 0.70,    # Heavy rain reduces traffic more
        'fog': 0.75,
        'snow': 0.60,          # Snow significantly impacts traffic
        'ice': 0.50,           # Icy roads reduce traffic significantly
    }
    
    @staticmethod
    def apply_weather_scaling(trips: List[Dict], 
                             weather_condition: str) -> List[Dict]:
        """
        Apply weather-based scaling
        
        Args:
            trips: List of trips
            weather_condition: Weather condition type
            
        Returns:
            Adjusted trips
        """
        factor = WeatherAdjustment.WEATHER_FACTORS.get(weather_condition, 1.0)
        
        scaled_trips = []
        for trip in trips:
            if random.random() < factor:
                scaled_trips.append(trip)
        
        return scaled_trips


if __name__ == "__main__":
    # Example usage
    profile = TemporalDemandProfile.get_rush_hour_profile()
    temporal = TemporalDemandProfile(profile)
    temporal.print_profile()
