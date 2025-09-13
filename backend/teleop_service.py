#!/usr/bin/env python3
"""
Remote Teleoperation Service for Indian Delivery Robot
Handles human-in-the-loop control for complex navigation scenarios
"""

import rospy
import json
import threading
import time
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String, Bool
import cv2
import numpy as np
from flask import Flask, request, jsonify, render_template_string
from flask_socketio import SocketIO, emit
import base64
import io
from PIL import Image as PILImage

@dataclass
class TeleopSession:
    """Structure for teleoperation session"""
    session_id: str
    operator_id: str
    start_time: float
    robot_pose: Tuple[float, float, float]
    status: str  # 'active', 'paused', 'completed', 'error'
    difficulty_score: float
    intervention_count: int

@dataclass
class NavigationContext:
    """Structure for navigation context"""
    current_pose: Tuple[float, float, float]
    target_pose: Tuple[float, float, float]
    obstacle_density: float
    pedestrian_count: int
    animal_count: int
    weather_condition: str
    time_of_day: str
    confidence_score: float

class TeleopService:
    """Remote teleoperation service for Indian delivery robot"""
    
    def __init__(self):
        rospy.init_node('teleop_service')
        
        # ROS Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher('/teleop_status', String, queue_size=1)
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.camera_sub = rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)
        
        # Teleoperation state
        self.active_sessions: Dict[str, TeleopSession] = {}
        self.current_session: Optional[TeleopSession] = None
        self.navigation_context = NavigationContext(
            current_pose=(0.0, 0.0, 0.0),
            target_pose=(0.0, 0.0, 0.0),
            obstacle_density=0.0,
            pedestrian_count=0,
            animal_count=0,
            weather_condition='clear',
            time_of_day='day',
            confidence_score=1.0
        )
        
        # Control parameters
        self.max_linear_vel = 0.5
        self.max_angular_vel = 0.5
        self.safety_distance = 1.0
        self.emergency_stop = False
        
        # Web interface
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        self.setup_web_interface()
        
        # Start ROS and web server
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        rospy.loginfo("Teleoperation service initialized")
    
    def setup_web_interface(self):
        """Setup web interface for teleoperation"""
        
        # HTML template for teleoperation interface
        html_template = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>🇮🇳 Indian Delivery Robot Teleoperation</title>
            <meta charset="utf-8">
            <meta name="viewport" content="width=device-width, initial-scale=1">
            <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
            <style>
                body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f5f5f5; }
                .container { max-width: 1200px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
                .header { text-align: center; color: #2c3e50; margin-bottom: 30px; }
                .status { background: #ecf0f1; padding: 15px; border-radius: 5px; margin-bottom: 20px; }
                .controls { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-bottom: 20px; }
                .control-panel { background: #34495e; color: white; padding: 20px; border-radius: 5px; }
                .video-feed { background: #2c3e50; padding: 20px; border-radius: 5px; text-align: center; }
                .button { background: #3498db; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; margin: 5px; }
                .button:hover { background: #2980b9; }
                .button.danger { background: #e74c3c; }
                .button.danger:hover { background: #c0392b; }
                .button.success { background: #27ae60; }
                .button.success:hover { background: #229954; }
                .slider { width: 100%; margin: 10px 0; }
                .info-panel { background: #ecf0f1; padding: 15px; border-radius: 5px; margin-top: 20px; }
                .metric { display: inline-block; margin: 10px; padding: 10px; background: white; border-radius: 5px; }
            </style>
        </head>
        <body>
            <div class="container">
                <div class="header">
                    <h1>🇮🇳 Indian Delivery Robot Teleoperation</h1>
                    <p>Remote control interface for autonomous delivery robot</p>
                </div>
                
                <div class="status" id="status">
                    <h3>Robot Status</h3>
                    <p>Connection: <span id="connection-status">Disconnected</span></p>
                    <p>Session: <span id="session-status">No active session</span></p>
                    <p>Mode: <span id="control-mode">Autonomous</span></p>
                </div>
                
                <div class="controls">
                    <div class="control-panel">
                        <h3>Manual Control</h3>
                        <button class="button" onclick="startTeleop()">Start Teleoperation</button>
                        <button class="button danger" onclick="stopTeleop()">Stop Teleoperation</button>
                        <button class="button danger" onclick="emergencyStop()">Emergency Stop</button>
                        
                        <h4>Movement Controls</h4>
                        <div style="text-align: center;">
                            <button class="button" onmousedown="moveRobot('forward')" onmouseup="stopRobot()">↑ Forward</button><br>
                            <button class="button" onmousedown="moveRobot('left')" onmouseup="stopRobot()">← Left</button>
                            <button class="button" onmousedown="moveRobot('stop')" onmouseup="stopRobot()">⏹ Stop</button>
                            <button class="button" onmousedown="moveRobot('right')" onmouseup="stopRobot()">Right →</button><br>
                            <button class="button" onmousedown="moveRobot('backward')" onmouseup="stopRobot()">↓ Backward</button>
                        </div>
                        
                        <h4>Speed Control</h4>
                        <label>Linear Speed: <span id="linear-speed">0.5</span> m/s</label>
                        <input type="range" class="slider" id="linear-slider" min="0" max="1" step="0.1" value="0.5" onchange="updateSpeed()">
                        
                        <label>Angular Speed: <span id="angular-speed">0.5</span> rad/s</label>
                        <input type="range" class="slider" id="angular-slider" min="0" max="1" step="0.1" value="0.5" onchange="updateSpeed()">
                    </div>
                    
                    <div class="video-feed">
                        <h3>Robot Camera Feed</h3>
                        <img id="camera-feed" src="" alt="Camera feed" style="max-width: 100%; height: auto;">
                        <p id="camera-status">Waiting for camera feed...</p>
                    </div>
                </div>
                
                <div class="info-panel">
                    <h3>Navigation Context</h3>
                    <div class="metric">Obstacle Density: <span id="obstacle-density">0.0</span></div>
                    <div class="metric">Pedestrians: <span id="pedestrian-count">0</span></div>
                    <div class="metric">Animals: <span id="animal-count">0</span></div>
                    <div class="metric">Confidence: <span id="confidence-score">1.0</span></div>
                    <div class="metric">Weather: <span id="weather-condition">Clear</span></div>
                </div>
            </div>
            
            <script>
                const socket = io();
                let isTeleopActive = false;
                let currentSpeed = {linear: 0.5, angular: 0.5};
                
                socket.on('connect', function() {
                    document.getElementById('connection-status').textContent = 'Connected';
                    document.getElementById('connection-status').style.color = 'green';
                });
                
                socket.on('disconnect', function() {
                    document.getElementById('connection-status').textContent = 'Disconnected';
                    document.getElementById('connection-status').style.color = 'red';
                });
                
                socket.on('robot_status', function(data) {
                    document.getElementById('session-status').textContent = data.session_id || 'No active session';
                    document.getElementById('control-mode').textContent = data.mode || 'Autonomous';
                });
                
                socket.on('navigation_context', function(data) {
                    document.getElementById('obstacle-density').textContent = data.obstacle_density.toFixed(2);
                    document.getElementById('pedestrian-count').textContent = data.pedestrian_count;
                    document.getElementById('animal-count').textContent = data.animal_count;
                    document.getElementById('confidence-score').textContent = data.confidence_score.toFixed(2);
                    document.getElementById('weather-condition').textContent = data.weather_condition;
                });
                
                socket.on('camera_feed', function(data) {
                    document.getElementById('camera-feed').src = 'data:image/jpeg;base64,' + data.image;
                    document.getElementById('camera-status').textContent = 'Live feed active';
                });
                
                function startTeleop() {
                    socket.emit('start_teleop', {operator_id: 'operator_1'});
                    isTeleopActive = true;
                }
                
                function stopTeleop() {
                    socket.emit('stop_teleop');
                    isTeleopActive = false;
                }
                
                function emergencyStop() {
                    socket.emit('emergency_stop');
                    isTeleopActive = false;
                }
                
                function moveRobot(direction) {
                    if (!isTeleopActive) return;
                    
                    const cmd = {
                        direction: direction,
                        linear_speed: currentSpeed.linear,
                        angular_speed: currentSpeed.angular
                    };
                    
                    socket.emit('robot_command', cmd);
                }
                
                function stopRobot() {
                    if (!isTeleopActive) return;
                    socket.emit('robot_command', {direction: 'stop'});
                }
                
                function updateSpeed() {
                    currentSpeed.linear = parseFloat(document.getElementById('linear-slider').value);
                    currentSpeed.angular = parseFloat(document.getElementById('angular-slider').value);
                    
                    document.getElementById('linear-speed').textContent = currentSpeed.linear.toFixed(1);
                    document.getElementById('angular-speed').textContent = currentSpeed.angular.toFixed(1);
                }
            </script>
        </body>
        </html>
        """
        
        @self.app.route('/')
        def index():
            return render_template_string(html_template)
        
        @self.socketio.on('start_teleop')
        def handle_start_teleop(data):
            session_id = self.start_teleop_session(data['operator_id'])
            emit('teleop_started', {'session_id': session_id})
        
        @self.socketio.on('stop_teleop')
        def handle_stop_teleop():
            self.stop_teleop_session()
            emit('teleop_stopped')
        
        @self.socketio.on('emergency_stop')
        def handle_emergency_stop():
            self.emergency_stop()
            emit('emergency_stopped')
        
        @self.socketio.on('robot_command')
        def handle_robot_command(data):
            if self.current_session:
                self.execute_robot_command(data)
    
    def start_teleop_session(self, operator_id: str) -> str:
        """Start a new teleoperation session"""
        session_id = f"session_{int(time.time())}"
        
        session = TeleopSession(
            session_id=session_id,
            operator_id=operator_id,
            start_time=time.time(),
            robot_pose=self.navigation_context.current_pose,
            status='active',
            difficulty_score=0.0,
            intervention_count=0
        )
        
        self.active_sessions[session_id] = session
        self.current_session = session
        
        rospy.loginfo(f"Started teleoperation session {session_id} for operator {operator_id}")
        return session_id
    
    def stop_teleop_session(self):
        """Stop current teleoperation session"""
        if self.current_session:
            self.current_session.status = 'completed'
            rospy.loginfo(f"Stopped teleoperation session {self.current_session.session_id}")
            self.current_session = None
    
    def emergency_stop(self):
        """Emergency stop robot"""
        self.emergency_stop = True
        self.stop_robot()
        rospy.logwarn("Emergency stop activated!")
    
    def execute_robot_command(self, command: Dict):
        """Execute robot command from teleoperator"""
        if not self.current_session or self.emergency_stop:
            return
        
        twist = Twist()
        direction = command.get('direction', 'stop')
        linear_speed = command.get('linear_speed', 0.5)
        angular_speed = command.get('angular_speed', 0.5)
        
        # Apply safety limits
        linear_speed = min(linear_speed, self.max_linear_vel)
        angular_speed = min(angular_speed, self.max_angular_vel)
        
        if direction == 'forward':
            twist.linear.x = linear_speed
        elif direction == 'backward':
            twist.linear.x = -linear_speed
        elif direction == 'left':
            twist.angular.z = angular_speed
        elif direction == 'right':
            twist.angular.z = -angular_speed
        elif direction == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
    
    def stop_robot(self):
        """Stop robot movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def odom_callback(self, msg):
        """Handle odometry data"""
        pose = msg.pose.pose
        self.navigation_context.current_pose = (
            pose.position.x,
            pose.position.y,
            pose.position.z
        )
    
    def scan_callback(self, msg):
        """Handle laser scan data"""
        # Calculate obstacle density
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[~np.isnan(ranges) & (ranges < msg.range_max)]
        
        if len(valid_ranges) > 0:
            close_obstacles = np.sum(valid_ranges < self.safety_distance)
            self.navigation_context.obstacle_density = close_obstacles / len(valid_ranges)
        
        # Update confidence score based on obstacle density
        self.navigation_context.confidence_score = max(0.0, 1.0 - self.navigation_context.obstacle_density)
    
    def camera_callback(self, msg):
        """Handle camera data"""
        try:
            # Convert ROS image to OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Encode as JPEG for web streaming
            _, buffer = cv2.imencode('.jpg', cv_image)
            img_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Send to web interface
            self.socketio.emit('camera_feed', {'image': img_base64})
            
        except Exception as e:
            rospy.logwarn(f"Camera processing error: {e}")
    
    def ros_spin(self):
        """ROS spin loop"""
        rospy.spin()
    
    def run_web_server(self, host='0.0.0.0', port=5000):
        """Run web server"""
        rospy.loginfo(f"Starting web server on {host}:{port}")
        self.socketio.run(self.app, host=host, port=port, debug=False)

def main():
    """Main function"""
    try:
        teleop_service = TeleopService()
        
        # Run web server in main thread
        teleop_service.run_web_server()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Teleoperation service shutdown")
    except Exception as e:
        rospy.logerr(f"Teleoperation service error: {e}")

if __name__ == "__main__":
    main()
