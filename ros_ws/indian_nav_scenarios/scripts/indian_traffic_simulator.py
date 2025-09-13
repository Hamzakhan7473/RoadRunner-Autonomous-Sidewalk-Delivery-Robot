#!/usr/bin/env python3

import rospy
import random
import math
from geometry_msgs.msg import Pose, PoseStamped, Twist
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import tf2_ros
import tf2_geometry_msgs

class IndianTrafficSimulator:
    def __init__(self):
        rospy.init_node('indian_traffic_simulator')
        
        # Parameters
        self.pedestrian_density = rospy.get_param('~pedestrian_density', 0.8)
        self.animal_probability = rospy.get_param('~animal_probability', 0.3)
        self.vehicle_density = rospy.get_param('~vehicle_density', 0.6)
        
        # Services
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # Dynamic obstacles
        self.pedestrians = []
        self.animals = []
        self.vehicles = []
        
        # Spawn initial obstacles
        self.spawn_initial_obstacles()
        
        # Start simulation loop
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_obstacles)
        
        rospy.loginfo("Indian Traffic Simulator started")

    def spawn_initial_obstacles(self):
        """Spawn initial dynamic obstacles"""
        
        # Spawn pedestrians
        num_pedestrians = int(self.pedestrian_density * 10)
        for i in range(num_pedestrians):
            self.spawn_pedestrian(f"pedestrian_{i}")
        
        # Spawn animals (stray dogs)
        if random.random() < self.animal_probability:
            num_animals = random.randint(1, 3)
            for i in range(num_animals):
                self.spawn_animal(f"stray_dog_{i}")
        
        # Spawn vehicles (cycles, autorickshaws)
        num_vehicles = int(self.vehicle_density * 5)
        for i in range(num_vehicles):
            vehicle_type = random.choice(['cycle', 'autorickshaw'])
            self.spawn_vehicle(f"{vehicle_type}_{i}", vehicle_type)

    def spawn_pedestrian(self, name):
        """Spawn a pedestrian"""
        x = random.uniform(-15, 15)
        y = random.uniform(-40, 40)
        
        pedestrian_model = f"""
        <sdf version="1.6">
          <model name="{name}">
            <static>false</static>
            <link name="link">
              <pose>{x} {y} 0.5 0 0 0</pose>
              <collision name="collision">
                <geometry>
                  <cylinder>
                    <radius>0.3</radius>
                    <length>1.0</length>
                  </cylinder>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <cylinder>
                    <radius>0.3</radius>
                    <length>1.0</length>
                  </cylinder>
                </geometry>
                <material>
                  <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>Gazebo/Blue</name>
                  </script>
                </material>
              </visual>
              <inertial>
                <mass>70</mass>
                <inertia>
                  <ixx>1.0</ixx>
                  <iyy>1.0</iyy>
                  <izz>1.0</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>
        """
        
        try:
            self.spawn_model(name, pedestrian_model, "", Pose(), "world")
            self.pedestrians.append({
                'name': name,
                'x': x,
                'y': y,
                'vx': random.uniform(-0.5, 0.5),
                'vy': random.uniform(-0.5, 0.5)
            })
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to spawn pedestrian {name}: {e}")

    def spawn_animal(self, name):
        """Spawn a stray animal (dog)"""
        x = random.uniform(-15, 15)
        y = random.uniform(-40, 40)
        
        animal_model = f"""
        <sdf version="1.6">
          <model name="{name}">
            <static>false</static>
            <link name="link">
              <pose>{x} {y} 0.2 0 0 0</pose>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>0.6 0.3 0.4</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>0.6 0.3 0.4</size>
                  </box>
                </geometry>
                <material>
                  <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>Gazebo/Brown</name>
                  </script>
                </material>
              </visual>
              <inertial>
                <mass>15</mass>
                <inertia>
                  <ixx>0.5</ixx>
                  <iyy>0.5</iyy>
                  <izz>0.5</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>
        """
        
        try:
            self.spawn_model(name, animal_model, "", Pose(), "world")
            self.animals.append({
                'name': name,
                'x': x,
                'y': y,
                'vx': random.uniform(-0.3, 0.3),
                'vy': random.uniform(-0.3, 0.3)
            })
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to spawn animal {name}: {e}")

    def spawn_vehicle(self, name, vehicle_type):
        """Spawn a vehicle (cycle or autorickshaw)"""
        x = random.uniform(-15, 15)
        y = random.uniform(-40, 40)
        
        if vehicle_type == 'cycle':
            size = "0.8 0.3 1.0"
            color = "Gazebo/Red"
            mass = 20
        else:  # autorickshaw
            size = "1.5 0.8 1.2"
            color = "Gazebo/Yellow"
            mass = 200
        
        vehicle_model = f"""
        <sdf version="1.6">
          <model name="{name}">
            <static>false</static>
            <link name="link">
              <pose>{x} {y} 0.1 0 0 0</pose>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>{size}</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>{size}</size>
                  </box>
                </geometry>
                <material>
                  <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>{color}</name>
                  </script>
                </material>
              </visual>
              <inertial>
                <mass>{mass}</mass>
                <inertia>
                  <ixx>1.0</ixx>
                  <iyy>1.0</iyy>
                  <izz>1.0</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>
        """
        
        try:
            self.spawn_model(name, vehicle_model, "", Pose(), "world")
            self.vehicles.append({
                'name': name,
                'type': vehicle_type,
                'x': x,
                'y': y,
                'vx': random.uniform(-1.0, 1.0),
                'vy': random.uniform(-1.0, 1.0)
            })
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to spawn vehicle {name}: {e}")

    def update_obstacles(self, event):
        """Update obstacle positions"""
        
        # Update pedestrians
        for pedestrian in self.pedestrians:
            self.update_pedestrian(pedestrian)
        
        # Update animals
        for animal in self.animals:
            self.update_animal(animal)
        
        # Update vehicles
        for vehicle in self.vehicles:
            self.update_vehicle(vehicle)

    def update_pedestrian(self, pedestrian):
        """Update pedestrian position with random movement"""
        dt = 0.1
        
        # Random walk behavior
        if random.random() < 0.1:  # 10% chance to change direction
            pedestrian['vx'] = random.uniform(-0.5, 0.5)
            pedestrian['vy'] = random.uniform(-0.5, 0.5)
        
        # Update position
        pedestrian['x'] += pedestrian['vx'] * dt
        pedestrian['y'] += pedestrian['vy'] * dt
        
        # Boundary checking
        if abs(pedestrian['x']) > 15:
            pedestrian['vx'] *= -1
        if abs(pedestrian['y']) > 40:
            pedestrian['vy'] *= -1
        
        # Update in Gazebo
        self.set_model_pose(pedestrian['name'], pedestrian['x'], pedestrian['y'], 0.5, 0, 0, 0)

    def update_animal(self, animal):
        """Update animal position with erratic movement"""
        dt = 0.1
        
        # Erratic movement (like stray dogs)
        if random.random() < 0.2:  # 20% chance to change direction
            animal['vx'] = random.uniform(-0.3, 0.3)
            animal['vy'] = random.uniform(-0.3, 0.3)
        
        # Update position
        animal['x'] += animal['vx'] * dt
        animal['y'] += animal['vy'] * dt
        
        # Boundary checking
        if abs(animal['x']) > 15:
            animal['vx'] *= -1
        if abs(animal['y']) > 40:
            animal['vy'] *= -1
        
        # Update in Gazebo
        self.set_model_pose(animal['name'], animal['x'], animal['y'], 0.2, 0, 0, 0)

    def update_vehicle(self, vehicle):
        """Update vehicle position"""
        dt = 0.1
        
        # More predictable movement for vehicles
        if random.random() < 0.05:  # 5% chance to change direction
            vehicle['vx'] = random.uniform(-1.0, 1.0)
            vehicle['vy'] = random.uniform(-1.0, 1.0)
        
        # Update position
        vehicle['x'] += vehicle['vx'] * dt
        vehicle['y'] += vehicle['vy'] * dt
        
        # Boundary checking
        if abs(vehicle['x']) > 15:
            vehicle['vx'] *= -1
        if abs(vehicle['y']) > 40:
            vehicle['vy'] *= -1
        
        # Update in Gazebo
        self.set_model_pose(vehicle['name'], vehicle['x'], vehicle['y'], 0.1, 0, 0, 0)

    def set_model_pose(self, model_name, x, y, z, roll, pitch, yaw):
        """Set model pose in Gazebo"""
        try:
            state = ModelState()
            state.model_name = model_name
            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = z
            state.pose.orientation.x = roll
            state.pose.orientation.y = pitch
            state.pose.orientation.z = yaw
            state.pose.orientation.w = 1.0
            
            self.set_model_state(state)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to update {model_name}: {e}")

if __name__ == '__main__':
    try:
        simulator = IndianTrafficSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
