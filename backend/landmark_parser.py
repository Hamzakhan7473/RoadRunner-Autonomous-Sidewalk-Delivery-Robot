#!/usr/bin/env python3
"""
Indian Address Landmark Parser
Converts Indian-style addresses with landmarks to GPS coordinates
"""

import re
import json
import requests
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from geopy.geocoders import Nominatim
from geopy.distance import geodesic
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class AddressComponents:
    """Structure for parsed address components"""
    pin_code: Optional[str] = None
    city: Optional[str] = None
    state: Optional[str] = None
    landmark: Optional[str] = None
    building: Optional[str] = None
    floor: Optional[str] = None
    apartment: Optional[str] = None
    street: Optional[str] = None
    area: Optional[str] = None
    raw_address: str = ""

@dataclass
class ParsedLocation:
    """Structure for parsed location result"""
    coordinates: Tuple[float, float]
    confidence: float
    address_components: AddressComponents
    landmark_info: Dict
    alternative_locations: List[Tuple[float, float]]

class IndianLandmarkParser:
    """Parser for Indian addresses with landmark-based navigation"""
    
    def __init__(self):
        self.geolocator = Nominatim(user_agent="india_delivery_robot")
        
        # Indian PIN code pattern
        self.pin_pattern = re.compile(r'\b\d{6}\b')
        
        # Common Indian landmarks
        self.landmark_keywords = {
            'religious': ['temple', 'mosque', 'church', 'gurudwara', 'mandir', 'masjid'],
            'commercial': ['market', 'mall', 'shop', 'store', 'bazaar', 'chowk'],
            'food': ['restaurant', 'hotel', 'dhaba', 'chai', 'tea', 'coffee', 'stall'],
            'transport': ['station', 'metro', 'bus', 'railway', 'airport', 'stand'],
            'education': ['school', 'college', 'university', 'institute', 'campus'],
            'healthcare': ['hospital', 'clinic', 'medical', 'pharmacy', 'chemist'],
            'government': ['office', 'court', 'police', 'post', 'bank', 'atm'],
            'residential': ['society', 'apartment', 'building', 'complex', 'colony']
        }
        
        # Common Indian address patterns
        self.address_patterns = [
            r'(\d+)\s*,\s*([^,]+)\s*,\s*near\s+([^,]+)',
            r'([^,]+)\s*,\s*near\s+([^,]+)\s*,\s*([^,]+)',
            r'([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*(\d{6})',
            r'(\d+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)',
        ]
        
        # Load landmark database (in production, this would be a proper database)
        self.landmark_database = self._load_landmark_database()
    
    def _load_landmark_database(self) -> Dict:
        """Load landmark database with known locations"""
        return {
            'delhi': {
                'chandni_chowk': {'lat': 28.6562, 'lon': 77.2310, 'type': 'market'},
                'connaught_place': {'lat': 28.6315, 'lon': 77.2167, 'type': 'commercial'},
                'red_fort': {'lat': 28.6562, 'lon': 77.2410, 'type': 'monument'},
                'india_gate': {'lat': 28.6129, 'lon': 77.2295, 'type': 'monument'},
                'rajiv_chowk': {'lat': 28.6315, 'lon': 77.2167, 'type': 'metro_station'}
            },
            'mumbai': {
                'crawford_market': {'lat': 18.9444, 'lon': 72.8333, 'type': 'market'},
                'gateway_of_india': {'lat': 18.9220, 'lon': 72.8347, 'type': 'monument'},
                'marine_drive': {'lat': 18.9444, 'lon': 72.8167, 'type': 'promenade'},
                'bandra_station': {'lat': 19.0544, 'lon': 72.8406, 'type': 'railway_station'}
            },
            'bangalore': {
                'commercial_street': {'lat': 12.9716, 'lon': 77.5946, 'type': 'market'},
                'mg_road': {'lat': 12.9716, 'lon': 77.5946, 'type': 'commercial'},
                'cubbon_park': {'lat': 12.9716, 'lon': 77.5946, 'type': 'park'},
                'vidhana_soudha': {'lat': 12.9791, 'lon': 77.5913, 'type': 'government'}
            }
        }
    
    def parse_address(self, address: str) -> AddressComponents:
        """Parse Indian address into components"""
        address = address.strip().lower()
        
        components = AddressComponents(raw_address=address)
        
        # Extract PIN code
        pin_match = self.pin_pattern.search(address)
        if pin_match:
            components.pin_code = pin_match.group()
        
        # Extract landmark
        landmark = self._extract_landmark(address)
        if landmark:
            components.landmark = landmark
        
        # Extract building/apartment info
        building_info = self._extract_building_info(address)
        if building_info:
            components.building = building_info.get('building')
            components.floor = building_info.get('floor')
            components.apartment = building_info.get('apartment')
        
        # Extract area/street
        area_info = self._extract_area_info(address)
        if area_info:
            components.area = area_info.get('area')
            components.street = area_info.get('street')
        
        # Extract city/state
        city_state = self._extract_city_state(address)
        if city_state:
            components.city = city_state.get('city')
            components.state = city_state.get('state')
        
        return components
    
    def _extract_landmark(self, address: str) -> Optional[str]:
        """Extract landmark from address"""
        # Look for "near" keyword
        near_pattern = r'near\s+([^,]+)'
        near_match = re.search(near_pattern, address)
        if near_match:
            return near_match.group(1).strip()
        
        # Look for common landmark keywords
        for category, keywords in self.landmark_keywords.items():
            for keyword in keywords:
                if keyword in address:
                    # Extract the phrase containing the keyword
                    pattern = rf'([^,]*{keyword}[^,]*)'
                    match = re.search(pattern, address)
                    if match:
                        return match.group(1).strip()
        
        return None
    
    def _extract_building_info(self, address: str) -> Optional[Dict]:
        """Extract building information"""
        building_patterns = [
            r'(\d+)\s*,\s*([^,]+)',  # "123, Building Name"
            r'([^,]+)\s*,\s*floor\s+(\d+)',  # "Building Name, Floor 2"
            r'([^,]+)\s*,\s*flat\s+(\d+)',  # "Building Name, Flat 101"
            r'([^,]+)\s*,\s*apartment\s+(\d+)',  # "Building Name, Apartment 5"
        ]
        
        for pattern in building_patterns:
            match = re.search(pattern, address)
            if match:
                return {
                    'building': match.group(1).strip(),
                    'floor': match.group(2) if len(match.groups()) > 1 else None,
                    'apartment': match.group(2) if 'flat' in pattern or 'apartment' in pattern else None
                }
        
        return None
    
    def _extract_area_info(self, address: str) -> Optional[Dict]:
        """Extract area/street information"""
        area_patterns = [
            r'([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)',  # "Street, Area, City"
            r'([^,]+)\s*,\s*([^,]+)',  # "Area, City"
        ]
        
        for pattern in area_patterns:
            match = re.search(pattern, address)
            if match:
                groups = match.groups()
                if len(groups) >= 2:
                    return {
                        'street': groups[0].strip(),
                        'area': groups[1].strip()
                    }
        
        return None
    
    def _extract_city_state(self, address: str) -> Optional[Dict]:
        """Extract city and state information"""
        # Common Indian cities
        cities = ['delhi', 'mumbai', 'bangalore', 'chennai', 'kolkata', 'hyderabad', 'pune', 'ahmedabad']
        
        for city in cities:
            if city in address:
                return {'city': city}
        
        return None
    
    def geocode_address(self, address: str) -> ParsedLocation:
        """Convert address to coordinates with confidence score"""
        try:
            # Parse address components
            components = self.parse_address(address)
            
            # Try different geocoding strategies
            coordinates, confidence = self._geocode_with_strategies(components)
            
            # Get landmark information
            landmark_info = self._get_landmark_info(components.landmark)
            
            # Find alternative locations
            alternatives = self._find_alternative_locations(components)
            
            return ParsedLocation(
                coordinates=coordinates,
                confidence=confidence,
                address_components=components,
                landmark_info=landmark_info,
                alternative_locations=alternatives
            )
            
        except Exception as e:
            logger.error(f"Error geocoding address: {e}")
            return ParsedLocation(
                coordinates=(0.0, 0.0),
                confidence=0.0,
                address_components=AddressComponents(raw_address=address),
                landmark_info={},
                alternative_locations=[]
            )
    
    def _geocode_with_strategies(self, components: AddressComponents) -> Tuple[Tuple[float, float], float]:
        """Try multiple geocoding strategies"""
        strategies = [
            self._geocode_by_landmark,
            self._geocode_by_pin_code,
            self._geocode_by_city_area,
            self._geocode_full_address
        ]
        
        for strategy in strategies:
            try:
                coords, confidence = strategy(components)
                if confidence > 0.5:
                    return coords, confidence
            except Exception as e:
                logger.warning(f"Strategy failed: {e}")
                continue
        
        return (0.0, 0.0), 0.0
    
    def _geocode_by_landmark(self, components: AddressComponents) -> Tuple[Tuple[float, float], float]:
        """Geocode using landmark information"""
        if not components.landmark:
            return (0.0, 0.0), 0.0
        
        # Check landmark database
        landmark_info = self._get_landmark_info(components.landmark)
        if landmark_info:
            return (landmark_info['lat'], landmark_info['lon']), 0.9
        
        # Try geocoding landmark directly
        try:
            location = self.geolocator.geocode(f"{components.landmark}, India")
            if location:
                return (location.latitude, location.longitude), 0.8
        except Exception as e:
            logger.warning(f"Landmark geocoding failed: {e}")
        
        return (0.0, 0.0), 0.0
    
    def _geocode_by_pin_code(self, components: AddressComponents) -> Tuple[Tuple[float, float], float]:
        """Geocode using PIN code"""
        if not components.pin_code:
            return (0.0, 0.0), 0.0
        
        try:
            location = self.geolocator.geocode(f"{components.pin_code}, India")
            if location:
                return (location.latitude, location.longitude), 0.7
        except Exception as e:
            logger.warning(f"PIN code geocoding failed: {e}")
        
        return (0.0, 0.0), 0.0
    
    def _geocode_by_city_area(self, components: AddressComponents) -> Tuple[Tuple[float, float], float]:
        """Geocode using city and area"""
        if not components.city:
            return (0.0, 0.0), 0.0
        
        query_parts = [components.city]
        if components.area:
            query_parts.append(components.area)
        if components.state:
            query_parts.append(components.state)
        
        query = ", ".join(query_parts) + ", India"
        
        try:
            location = self.geolocator.geocode(query)
            if location:
                return (location.latitude, location.longitude), 0.6
        except Exception as e:
            logger.warning(f"City/area geocoding failed: {e}")
        
        return (0.0, 0.0), 0.0
    
    def _geocode_full_address(self, components: AddressComponents) -> Tuple[Tuple[float, float], float]:
        """Geocode using full address"""
        try:
            location = self.geolocator.geocode(f"{components.raw_address}, India")
            if location:
                return (location.latitude, location.longitude), 0.5
        except Exception as e:
            logger.warning(f"Full address geocoding failed: {e}")
        
        return (0.0, 0.0), 0.0
    
    def _get_landmark_info(self, landmark: Optional[str]) -> Dict:
        """Get landmark information from database"""
        if not landmark:
            return {}
        
        landmark_lower = landmark.lower().replace(' ', '_')
        
        # Search in landmark database
        for city, landmarks in self.landmark_database.items():
            for landmark_key, info in landmarks.items():
                if landmark_lower in landmark_key or landmark_key in landmark_lower:
                    return info
        
        return {}
    
    def _find_alternative_locations(self, components: AddressComponents) -> List[Tuple[float, float]]:
        """Find alternative locations based on address components"""
        alternatives = []
        
        # If we have a landmark, find nearby locations
        if components.landmark:
            landmark_info = self._get_landmark_info(components.landmark)
            if landmark_info:
                # Add some nearby locations (simulated)
                base_lat, base_lon = landmark_info['lat'], landmark_info['lon']
                for i in range(3):
                    offset_lat = base_lat + (i - 1) * 0.001
                    offset_lon = base_lon + (i - 1) * 0.001
                    alternatives.append((offset_lat, offset_lon))
        
        return alternatives
    
    def validate_coordinates(self, coordinates: Tuple[float, float]) -> bool:
        """Validate if coordinates are within India"""
        lat, lon = coordinates
        
        # India's approximate boundaries
        india_bounds = {
            'min_lat': 6.0,
            'max_lat': 37.0,
            'min_lon': 68.0,
            'max_lon': 97.0
        }
        
        return (india_bounds['min_lat'] <= lat <= india_bounds['max_lat'] and
                india_bounds['min_lon'] <= lon <= india_bounds['max_lon'])
    
    def get_delivery_instructions(self, parsed_location: ParsedLocation) -> Dict:
        """Generate delivery instructions based on parsed location"""
        instructions = {
            'primary_location': parsed_location.coordinates,
            'confidence': parsed_location.confidence,
            'landmark_guidance': '',
            'building_info': '',
            'delivery_notes': ''
        }
        
        # Add landmark guidance
        if parsed_location.landmark_info:
            landmark_type = parsed_location.landmark_info.get('type', 'unknown')
            instructions['landmark_guidance'] = f"Navigate to {parsed_location.address_components.landmark} ({landmark_type})"
        
        # Add building information
        if parsed_location.address_components.building:
            instructions['building_info'] = f"Building: {parsed_location.address_components.building}"
            if parsed_location.address_components.floor:
                instructions['building_info'] += f", Floor: {parsed_location.address_components.floor}"
            if parsed_location.address_components.apartment:
                instructions['building_info'] += f", Apartment: {parsed_location.address_components.apartment}"
        
        # Add delivery notes
        if parsed_location.confidence < 0.7:
            instructions['delivery_notes'] = "Low confidence location - may require human verification"
        
        return instructions

# Example usage and testing
def main():
    """Test the landmark parser with sample Indian addresses"""
    parser = IndianLandmarkParser()
    
    # Test addresses
    test_addresses = [
        "123, Near Chandni Chowk, Delhi, 110006",
        "Flat 101, Building A, Near MG Road, Bangalore, 560001",
        "Near Crawford Market, Mumbai, 400001",
        "Apartment 5, Floor 2, Near Connaught Place, Delhi",
        "Near Rajiv Chowk Metro Station, Delhi, 110001"
    ]
    
    print("🇮🇳 Indian Address Landmark Parser Test Results")
    print("=" * 60)
    
    for address in test_addresses:
        print(f"\n📍 Address: {address}")
        result = parser.geocode_address(address)
        
        print(f"   Coordinates: {result.coordinates}")
        print(f"   Confidence: {result.confidence:.2f}")
        print(f"   Landmark: {result.address_components.landmark}")
        print(f"   Building: {result.address_components.building}")
        print(f"   PIN Code: {result.address_components.pin_code}")
        
        if result.landmark_info:
            print(f"   Landmark Type: {result.landmark_info.get('type', 'unknown')}")
        
        instructions = parser.get_delivery_instructions(result)
        print(f"   Delivery Notes: {instructions['delivery_notes']}")

if __name__ == "__main__":
    main()
