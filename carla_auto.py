import carla
import math
import time

def distance_between_points(loc1, loc2):
    return math.sqrt((loc1.x - loc2.x)**2 + (loc1.y - loc2.y)**2 + (loc1.z - loc2.z)**2)

# Connect to CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Load Town04
client.load_world('Town04')
world = client.get_world()


# Set the desired spawn coordinates (example coordinates, replace with your own)
x, y, z = 85.673615, -372.373047, 3.686840 # Replace these with the coordinates you got from the spectator

# Set the desired rotation (example rotation, replace with your own)
yaw, pitch, roll = 157.764984, -25.754606, 0.000036 # Adjust these as needed


x, y, z = -9.801275,-188.7572631,3.0018434
# Set the desired rotation (example rotation, replace with your own)
yaw, pitch, roll = 94.368034, -11.225094, 0.000048 # Adjust these as needed
# Create the spawn point transform
spawn_point = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw, pitch=pitch, roll=roll))

# Spawn a vehicle at the desired location
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

# Enable autopilot
vehicle.set_autopilot(True)

# Teleport the spectator to the vehicle
spectator = world.get_spectator()
spectator.set_transform(vehicle.get_transform())

# Open a file in write mode
with open('straight_waypoints.csv', 'w') as file:
    # Write the header
    file.write("x,y,z\n")

    last_location = None
    distance_threshold = 0.3  # 0.3 meters

    # Set start time
    start_time = time.time()

    # Main loop to collect waypoints
    while True:
        transform = vehicle.get_transform()
        location = transform.location

        if last_location is None or distance_between_points(location, last_location) >= distance_threshold:
            file.write(f"{location.x},{location.y},{location.z}\n")
            last_location = location

        # Teleport the spectator to the vehicle periodically
        # spectator.set_transform(vehicle.get_transform())

        # Break condition for the loop - modify as needed
        if time.time() - start_time > 10:  # 300 seconds = 5 minutes
            break

# Disable autopilot and destroy vehicle
vehicle.set_autopilot(False)
vehicle.destroy()
