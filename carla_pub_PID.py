#!/usr/bin/env python3
import rospy
import carla
import numpy as np
import math
import random
import collections as queue
from cav_project.msg import ControlInfo
from carla import Client, World, ActorBlueprint, Transform, Location, Rotation
import time

def get_speed(vehicle):
	vel = vehicle.get_velocity()
	return 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

class VehiclePIDController:
	def __init__(self, vehicle, args_lateral, args_longitudinal, max_throttle=0.75, max_brake=0.3, max_steering=0.8):
		self.max_throttle = max_throttle
		self.max_steering = max_steering
		self.max_brake = max_brake
		self.vehicle = vehicle
		self.world = vehicle.get_world()

		# Initialize PID controllers
		self.long_controller = PIDLongCont(self.vehicle, **args_longitudinal)
		self.lat_controller = PIDLatCont(self.vehicle, **args_lateral)

		# Track past steering for smooth control
		self.past_steering = self.vehicle.get_control().steer

	def run_step(self, target_speed, waypoint):
		# Calculate acceleration and steering
		acceleration = self.long_controller.run_step(target_speed)
		current_steering = self.lat_controller.run_step(waypoint)

		# Create vehicle control object
		control = carla.VehicleControl()

		# Handle acceleration and braking
		if acceleration >= 0.0:
		    control.throttle = min(abs(acceleration), self.max_throttle)
		    control.brake = 0.0
		else:
		    control.throttle = 0.0
		    control.brake = min(abs(acceleration), self.max_brake)

		# Smooth out steering changes
		current_steering = np.clip(
		    current_steering, 
		    self.past_steering - 0.1, 
		    self.past_steering + 0.1
		)

		# Limit steering angle
		control.steer = np.clip(current_steering, -self.max_steering, self.max_steering)

		# Additional control settings
		control.hand_brake = False
		control.manual_gear_shift = False

		self.past_steering = control.steer
		return control

class PIDLongCont:
	def __init__(self, vehicle, KP=-0.0005, KI=-0.0005, KD=-0.002, dt=0.1):
		self.vehicle = vehicle
		self.KP, self.KI, self.KD = KP, KI, KD
		self.dt = dt
		self.error_buffer = queue.deque(maxlen=10)
        
	def run_step(self, target_speed):
		current_speed = get_speed(self.vehicle)
		return self.pid_controller(target_speed, current_speed)
    
	def pid_controller(self, target_speed, current_speed):
		error = target_speed - current_speed
		self.error_buffer.append(error)

		# Compute derivative and integral terms
		de = (self.error_buffer[-1] - self.error_buffer[-2])/self.dt if len(self.error_buffer) >= 2 else 0.0
		ie = sum(self.error_buffer)*self.dt if len(self.error_buffer) >= 2 else 0.0

		return np.clip(self.KP*error + self.KI*ie + self.KD*de, -1.0, 1.0)

class PIDLatCont:
	def __init__(self, vehicle, KP=1.0, KI=0.0, KD=0.0, dt=0.3):
		self.vehicle = vehicle
		self.KP, self.KI, self.KD = KP, KI, KD
		self.dt = dt
		self.error_buffer = queue.deque(maxlen=10)
    
	def run_step(self, waypoint):
		return self.pid_controller(waypoint, self.vehicle.get_transform())
    
	def pid_controller(self, waypoint, vehicle_transform):
		# Calculate angle between vehicle direction and waypoint
		v_begin = vehicle_transform.location
		v_end = v_begin + carla.Location(
			x=math.cos(math.radians(vehicle_transform.rotation.yaw)), 
			y=math.sin(math.radians(vehicle_transform.rotation.yaw))
		)

		v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
		w_vec = np.array([
			waypoint.transform.location.x - v_begin.x, 
			waypoint.transform.location.y - v_begin.y, 
			0.0
		])
        
		# Compute steering angle
		dot = math.acos(np.clip(
			np.dot(w_vec, v_vec) / (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), 
			-1.0, 1.0
		))
        
		cross = np.cross(v_vec, w_vec)
		if cross[2] < 0:
			dot *= -1

		self.error_buffer.append(dot)
        
		# Compute derivative and integral terms
		de = (self.error_buffer[-1] - self.error_buffer[-2])/self.dt if len(self.error_buffer) >= 2 else 0.0
		ie = sum(self.error_buffer)*self.dt if len(self.error_buffer) >= 2 else 0.0

		return np.clip(self.KP*dot + self.KI*ie + self.KD*de, -1.0, 1.0)

class CarlaLimoController:
	def __init__(self):
		# Initialize ROS node
		rospy.init_node("carla_limo_controller")
		self.control_info_topic = rospy.Publisher("control_info_", ControlInfo, queue_size=10)
		self.rate = rospy.Rate(10)

		# Connect to Carla
		self.client = carla.Client('localhost', 2000)
		self.client.set_timeout(5.0)
		self.client.load_world('Town04')
		self.world = self.client.get_world()

		# Spawn vehicle
		self.spawn_vehicle()

		# Create PID Controller
		self.pid_controller = VehiclePIDController(
			self.vehicle, 
			args_lateral={'KP':1.0, 'KI':0.1, 'KD':10.0},
			args_longitudinal={'KP':1.0, 'KI':0.0, 'KD':0.0}
		)
        
		# Target parameters
		self.target_speed = 10  # m/s
		self.actor_list = [self.vehicle]

	def spawn_vehicle(self):
		# Vehicle blueprint setup
		vehicle_bp = self.world.get_blueprint_library().find('vehicle.ford.mustang')
		vehicle_bp.set_attribute('role_name', 'ego')

		# Spawn at a random point
		x, y, z = -9.627799, -217.749588, 3.331123 # Replace these with the coordinates you got from the spectator
		x, y, z = -9.801275,-188.7572631,3.0018434
		# Set the desired rotation (example rotation, replace with your own)
		yaw, pitch, roll = 94.368034, -11.225094, 0.000048 # Adjust these as needed
		# Create the spawn point transform
		spawn_point = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw, pitch=pitch, roll=roll))
		self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
		self.spectator = self.world.get_spectator()
		self.spectator.set_transform(self.vehicle.get_transform())
		print('Ego vehicle spawned')
		time.sleep(1)

	def get_control_msg(self):
		# Create ROS message with current vehicle state
		msg = ControlInfo()
		current_control = self.vehicle.get_control()
		current_velocity = self.vehicle.get_velocity()

		msg.steering_angle = current_control.steer
		msg.desired_velocity = math.sqrt(current_velocity.x**2 + current_velocity.y**2) / 15
		msg.control_input = 1  # Placeholder for more complex control logic

		return msg

	def run(self):
		try:
			# with open('waypoints.csv', 'a') as file:
			while not rospy.is_shutdown():
				# Get nearby waypoints
				current_location = self.vehicle.get_location()
				waypoints = self.world.get_map().get_waypoint(current_location)
				# file.write(f"{waypoints.transform.location.x},{waypoints.transform.location.y},{waypoints.transform.location.z}\n")

				# Choose a random nearby waypoint
				waypoint = np.random.choice(waypoints.next(0.3))

				# Generate control signal
				control_signal = self.pid_controller.run_step(self.target_speed, waypoint)

				# Apply control to vehicle
				self.vehicle.apply_control(control_signal)
				print(self.vehicle.get_control().steer)
				print("speed:")
				print(get_speed(self.vehicle))
				# Publish ROS message
				msg = self.get_control_msg()
				self.control_info_topic.publish(msg)
				rospy.loginfo(msg)

				# Sleep to maintain loop rate
				self.rate.sleep()

		except Exception as e:
			print(f"An error occurred: {e}")
        
		finally:
			# Clean up actors
			self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])

if __name__ == '__main__':
	try:
		controller = CarlaLimoController()
		controller.run()
	except rospy.ROSInterruptException:
		pass