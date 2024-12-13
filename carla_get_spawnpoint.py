# import carla

# # Connect to CARLA server
# client = carla.Client('localhost', 2000)
# client.set_timeout(10.0)

# # Load Town07
# client.load_world('Town07')
# world = client.get_world()

# # Get all available spawn points
# spawn_points = world.get_map().get_spawn_points()

# # Print all spawn points to inspect their locations
# for i, spawn_point in enumerate(spawn_points):
#     print(f"Spawn Point {i}: {spawn_point.location}")

# # Select a specific spawn point (e.g., the first one in the list)
# selected_spawn_point = spawn_points[0]  # Change the index to your desired spawn point

# # Spawn a vehicle at the selected spawn point
# blueprint_library = world.get_blueprint_library()
# vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
# vehicle = world.spawn_actor(vehicle_bp, selected_spawn_point)

# # Disable autopilot and destroy vehicle when done
# vehicle.set_autopilot(False)
# vehicle.destroy()

import carla

# Connect to CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Load Town04
client.load_world('Town04')
world = client.get_world()

# Prompt to move the spectator manually
input("Move the spectator to the desired location in the CARLA simulator and press Enter to continue...")

# Get spectator to find coordinates
spectator = world.get_spectator()
spectator_transform = spectator.get_transform()

# Print spectator coordinates
print(f"Spectator location: {spectator_transform.location}")
print(f"Spectator rotation: {spectator_transform.rotation}")

# Set the desired spawn point using the coordinates
spawn_point = carla.Transform(
    carla.Location(x=spectator_transform.location.x, y=spectator_transform.location.y, z=spectator_transform.location.z),
    carla.Rotation(pitch=spectator_transform.rotation.pitch, yaw=spectator_transform.rotation.yaw, roll=spectator_transform.rotation.roll)
)

# Spawn a vehicle at the desired location
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

# Disable autopilot and destroy vehicle when done
vehicle.set_autopilot(False)
vehicle.destroy()
