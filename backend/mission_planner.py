#!/usr/bin/env python3
"""
Mission Planner for Indian Delivery Robot
Handles delivery mission assignment, route planning, and execution
"""

import rospy
import json
import time
import threading
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict
from datetime import datetime, timedelta
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
import numpy as np
from landmark_parser import IndianLandmarkParser, ParsedLocation

@dataclass
class DeliveryMission:
    """Structure for delivery mission"""
    mission_id: str
    customer_name: str
    customer_phone: str
    delivery_address: str
    parsed_location: ParsedLocation
    order_items: List[Dict]
    priority: int  # 1=high, 2=medium, 3=low
    estimated_delivery_time: datetime
    actual_delivery_time: Optional[datetime] = None
    status: str = 'pending'  # pending, in_progress, completed, failed
    assigned_robot: Optional[str] = None
    special_instructions: str = ""
    delivery_fee: float = 0.0

@dataclass
class RouteSegment:
    """Structure for route segment"""
    start_pose: Tuple[float, float, float]
    end_pose: Tuple[float, float, float]
    segment_type: str  # 'sidewalk', 'road', 'crossing', 'building'
    difficulty_score: float  # 0.0 (easy) to 1.0 (difficult)
    estimated_time: float  # seconds
    obstacles: List[str]
    weather_impact: float

@dataclass
class DeliveryRoute:
    """Structure for complete delivery route"""
    route_id: str
    mission_id: str
    segments: List[RouteSegment]
    total_distance: float
    total_time: float
    difficulty_score: float
    weather_adjustment: float
    alternative_routes: List[List[RouteSegment]]

class MissionPlanner:
    """Mission planner for Indian delivery robot"""
    
    def __init__(self):
        rospy.init_node('mission_planner')
        
        # ROS Publishers and Subscribers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher('/mission_status', String, queue_size=1)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Mission management
        self.active_missions: Dict[str, DeliveryMission] = {}
        self.completed_missions: List[DeliveryMission] = []
        self.current_mission: Optional[DeliveryMission] = None
        self.current_route: Optional[DeliveryRoute] = None
        
        # Robot state
        self.robot_pose = (0.0, 0.0, 0.0)
        self.robot_status = 'idle'  # idle, navigating, delivering, returning
        self.battery_level = 100.0
        self.cargo_capacity = 10.0  # kg
        self.current_load = 0.0
        
        # Indian-specific parameters
        self.landmark_parser = IndianLandmarkParser()
        self.indian_traffic_patterns = self._load_traffic_patterns()
        self.weather_conditions = self._load_weather_data()
        
        # Route planning parameters
        self.max_delivery_distance = 5.0  # km
        self.sidewalk_preference = 0.8
        self.crowd_avoidance_factor = 0.7
        self.weather_safety_threshold = 0.6
        
        # Start mission processing thread
        self.mission_thread = threading.Thread(target=self.process_missions)
        self.mission_thread.daemon = True
        self.mission_thread.start()
        
        rospy.loginfo("Mission planner initialized")
    
    def _load_traffic_patterns(self) -> Dict:
        """Load Indian traffic patterns"""
        return {
            'rush_hours': {
                'morning': (7, 9),
                'evening': (18, 20)
            },
            'crowd_density': {
                'commercial_areas': 0.8,
                'residential_areas': 0.3,
                'market_areas': 0.9,
                'educational_areas': 0.6
            },
            'animal_probability': {
                'morning': 0.4,
                'afternoon': 0.2,
                'evening': 0.3,
                'night': 0.1
            }
        }
    
    def _load_weather_data(self) -> Dict:
        """Load weather impact data"""
        return {
            'monsoon': {'speed_factor': 0.6, 'safety_factor': 0.7},
            'summer': {'speed_factor': 0.8, 'safety_factor': 0.9},
            'winter': {'speed_factor': 0.9, 'safety_factor': 0.95},
            'clear': {'speed_factor': 1.0, 'safety_factor': 1.0}
        }
    
    def create_delivery_mission(self, mission_data: Dict) -> str:
        """Create a new delivery mission"""
        mission_id = f"mission_{int(time.time())}"
        
        # Parse delivery address
        parsed_location = self.landmark_parser.geocode_address(mission_data['delivery_address'])
        
        # Validate location
        if parsed_location.confidence < 0.5:
            rospy.logwarn(f"Low confidence location for mission {mission_id}")
        
        # Create mission
        mission = DeliveryMission(
            mission_id=mission_id,
            customer_name=mission_data['customer_name'],
            customer_phone=mission_data['customer_phone'],
            delivery_address=mission_data['delivery_address'],
            parsed_location=parsed_location,
            order_items=mission_data['order_items'],
            priority=mission_data.get('priority', 2),
            estimated_delivery_time=datetime.now() + timedelta(minutes=30),
            special_instructions=mission_data.get('special_instructions', ''),
            delivery_fee=mission_data.get('delivery_fee', 0.0)
        )
        
        self.active_missions[mission_id] = mission
        rospy.loginfo(f"Created mission {mission_id} for {mission.customer_name}")
        
        return mission_id
    
    def plan_delivery_route(self, mission: DeliveryMission) -> DeliveryRoute:
        """Plan delivery route with Indian-specific considerations"""
        route_id = f"route_{mission.mission_id}"
        
        # Get current weather and time
        current_weather = self._get_current_weather()
        current_time = datetime.now()
        
        # Plan route segments
        segments = self._plan_route_segments(
            self.robot_pose,
            mission.parsed_location.coordinates,
            current_weather,
            current_time
        )
        
        # Calculate route metrics
        total_distance = sum(segment.estimated_time for segment in segments)
        total_time = sum(segment.estimated_time for segment in segments)
        difficulty_score = np.mean([segment.difficulty_score for segment in segments])
        
        # Apply weather adjustment
        weather_adjustment = self.weather_conditions[current_weather]['speed_factor']
        
        # Create route
        route = DeliveryRoute(
            route_id=route_id,
            mission_id=mission.mission_id,
            segments=segments,
            total_distance=total_distance,
            total_time=total_time * weather_adjustment,
            difficulty_score=difficulty_score,
            weather_adjustment=weather_adjustment,
            alternative_routes=[]
        )
        
        # Plan alternative routes
        route.alternative_routes = self._plan_alternative_routes(route)
        
        rospy.loginfo(f"Planned route {route_id}: {len(segments)} segments, {total_time:.1f}s")
        return route
    
    def _plan_route_segments(self, start_pose: Tuple[float, float, float], 
                            end_pose: Tuple[float, float, float],
                            weather: str, time: datetime) -> List[RouteSegment]:
        """Plan route segments with Indian-specific considerations"""
        segments = []
        
        # Calculate direct distance
        distance = np.sqrt((end_pose[0] - start_pose[0])**2 + (end_pose[1] - start_pose[1])**2)
        
        # Determine route complexity based on distance and location
        if distance < 0.5:  # Short distance
            segments.append(RouteSegment(
                start_pose=start_pose,
                end_pose=end_pose,
                segment_type='sidewalk',
                difficulty_score=0.3,
                estimated_time=distance * 2.0,  # 0.5 m/s walking speed
                obstacles=[],
                weather_impact=self.weather_conditions[weather]['safety_factor']
            ))
        else:  # Longer distance - break into segments
            # Segment 1: Initial navigation
            mid_pose = (
                start_pose[0] + (end_pose[0] - start_pose[0]) * 0.5,
                start_pose[1] + (end_pose[1] - start_pose[1]) * 0.5,
                0.0
            )
            
            segments.append(RouteSegment(
                start_pose=start_pose,
                end_pose=mid_pose,
                segment_type='sidewalk',
                difficulty_score=self._calculate_difficulty(start_pose, mid_pose, time),
                estimated_time=distance * 0.5 * 2.0,
                obstacles=self._identify_obstacles(start_pose, mid_pose),
                weather_impact=self.weather_conditions[weather]['safety_factor']
            ))
            
            # Segment 2: Final approach
            segments.append(RouteSegment(
                start_pose=mid_pose,
                end_pose=end_pose,
                segment_type='building',
                difficulty_score=self._calculate_difficulty(mid_pose, end_pose, time),
                estimated_time=distance * 0.5 * 1.5,
                obstacles=self._identify_obstacles(mid_pose, end_pose),
                weather_impact=self.weather_conditions[weather]['safety_factor']
            ))
        
        return segments
    
    def _calculate_difficulty(self, start_pose: Tuple[float, float, float],
                             end_pose: Tuple[float, float, float],
                             time: datetime) -> float:
        """Calculate difficulty score for route segment"""
        difficulty = 0.0
        
        # Time-based difficulty
        hour = time.hour
        if 7 <= hour <= 9 or 18 <= hour <= 20:  # Rush hours
            difficulty += 0.3
        
        # Distance-based difficulty
        distance = np.sqrt((end_pose[0] - start_pose[0])**2 + (end_pose[1] - start_pose[1])**2)
        if distance > 1.0:
            difficulty += 0.2
        
        # Area-based difficulty (simplified)
        # In real implementation, this would use map data
        difficulty += 0.1  # Base difficulty for Indian streets
        
        return min(difficulty, 1.0)
    
    def _identify_obstacles(self, start_pose: Tuple[float, float, float],
                           end_pose: Tuple[float, float, float]) -> List[str]:
        """Identify potential obstacles along route"""
        obstacles = []
        
        # Common Indian street obstacles
        obstacles.extend(['pedestrians', 'street_vendors', 'stray_animals'])
        
        # Add obstacles based on time of day
        current_hour = datetime.now().hour
        if 7 <= current_hour <= 9 or 18 <= current_hour <= 20:
            obstacles.append('traffic_congestion')
        
        return obstacles
    
    def _plan_alternative_routes(self, route: DeliveryRoute) -> List[List[RouteSegment]]:
        """Plan alternative routes for backup"""
        alternatives = []
        
        # Simple alternative: slightly different path
        if len(route.segments) > 1:
            alt_segments = route.segments.copy()
            # Modify second segment slightly
            if len(alt_segments) > 1:
                alt_segments[1].difficulty_score += 0.1
                alternatives.append(alt_segments)
        
        return alternatives
    
    def _get_current_weather(self) -> str:
        """Get current weather condition"""
        # In real implementation, this would query weather API
        return 'clear'  # Default to clear weather
    
    def execute_mission(self, mission_id: str) -> bool:
        """Execute a delivery mission"""
        if mission_id not in self.active_missions:
            rospy.logerr(f"Mission {mission_id} not found")
            return False
        
        mission = self.active_missions[mission_id]
        
        # Check robot capacity
        total_weight = sum(item.get('weight', 0.0) for item in mission.order_items)
        if self.current_load + total_weight > self.cargo_capacity:
            rospy.logwarn(f"Insufficient cargo capacity for mission {mission_id}")
            return False
        
        # Plan route
        route = self.plan_delivery_route(mission)
        
        # Update mission status
        mission.status = 'in_progress'
        mission.assigned_robot = 'robot_1'  # In multi-robot system, this would be dynamic
        self.current_mission = mission
        self.current_route = route
        
        # Start navigation
        self._navigate_to_destination(mission.parsed_location.coordinates)
        
        rospy.loginfo(f"Executing mission {mission_id}")
        return True
    
    def _navigate_to_destination(self, destination: Tuple[float, float]):
        """Navigate to destination"""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = destination[0]
        goal.pose.position.y = destination[1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        rospy.loginfo(f"Navigating to destination: {destination}")
    
    def complete_mission(self, mission_id: str, success: bool = True) -> bool:
        """Complete a delivery mission"""
        if mission_id not in self.active_missions:
            rospy.logerr(f"Mission {mission_id} not found")
            return False
        
        mission = self.active_missions[mission_id]
        mission.status = 'completed' if success else 'failed'
        mission.actual_delivery_time = datetime.now()
        
        # Move to completed missions
        self.completed_missions.append(mission)
        del self.active_missions[mission_id]
        
        # Clear current mission
        if self.current_mission and self.current_mission.mission_id == mission_id:
            self.current_mission = None
            self.current_route = None
        
        rospy.loginfo(f"Completed mission {mission_id}: {'success' if success else 'failed'}")
        return True
    
    def process_missions(self):
        """Process pending missions"""
        while not rospy.is_shutdown():
            try:
                # Check for missions to execute
                if not self.current_mission and self.active_missions:
                    # Find highest priority mission
                    priority_missions = sorted(
                        self.active_missions.values(),
                        key=lambda m: m.priority
                    )
                    
                    if priority_missions:
                        mission = priority_missions[0]
                        self.execute_mission(mission.mission_id)
                
                # Update mission status
                self._publish_mission_status()
                
                time.sleep(1.0)  # Process every second
                
            except Exception as e:
                rospy.logerr(f"Mission processing error: {e}")
                time.sleep(5.0)
    
    def _publish_mission_status(self):
        """Publish current mission status"""
        status = {
            'robot_status': self.robot_status,
            'current_mission': self.current_mission.mission_id if self.current_mission else None,
            'active_missions': len(self.active_missions),
            'completed_missions': len(self.completed_missions),
            'battery_level': self.battery_level,
            'current_load': self.current_load
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)
    
    def odom_callback(self, msg):
        """Handle odometry data"""
        pose = msg.pose.pose
        self.robot_pose = (pose.position.x, pose.position.y, pose.position.z)
    
    def scan_callback(self, msg):
        """Handle laser scan data"""
        # Update robot status based on scan data
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[~np.isnan(ranges) & (ranges < msg.range_max)]
        
        if len(valid_ranges) > 0:
            close_obstacles = np.sum(valid_ranges < 1.0)
            if close_obstacles > 5:
                self.robot_status = 'navigating_carefully'
            else:
                self.robot_status = 'navigating'

# Example usage and testing
def main():
    """Test the mission planner"""
    try:
        planner = MissionPlanner()
        
        # Test mission creation
        test_mission_data = {
            'customer_name': 'Rajesh Kumar',
            'customer_phone': '+91-9876543210',
            'delivery_address': '123, Near Chandni Chowk, Delhi, 110006',
            'order_items': [
                {'name': 'Chai', 'weight': 0.2, 'price': 20},
                {'name': 'Samosa', 'weight': 0.1, 'price': 15}
            ],
            'priority': 1,
            'special_instructions': 'Call before delivery',
            'delivery_fee': 30.0
        }
        
        mission_id = planner.create_delivery_mission(test_mission_data)
        rospy.loginfo(f"Created test mission: {mission_id}")
        
        # Start mission execution
        planner.execute_mission(mission_id)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Mission planner shutdown")
    except Exception as e:
        rospy.logerr(f"Mission planner error: {e}")

if __name__ == "__main__":
    main()
