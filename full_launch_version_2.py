# Beeland Space Agency -- Do not sell this code or make a profit out of it
import krpc
import time
import re
time.sleep(2)
conn = krpc.connect(name='Ascent')
booster = conn.space_center.active_vessel
file_path = "/Users/kingdomofbeeland/Desktop/Kerbal Space Program/English/Ships/Script/data.txt"
def read_trajectory_data(file_path):
    try:
        with open(file_path, "r") as file:
            lines = file.readlines()
            if lines:
                # Get the most recent line
                latest_data = lines[-1].strip()

                # Use regex to extract latitude and longitude from the format
                match = re.search(r"GEOPOSITIONLATLNG\((-?\d+\.\d+),(-?\d+\.\d+)\)", latest_data)
                if match:
                    latitude = float(match.group(1))
                    longitude = float(match.group(2))

                    # Re-write the file with only the most recent data
                    with open(file_path, "w") as file_write:
                        file_write.write(latest_data + "\n")

                    return latitude, longitude
                else:
                    print("No valid data found in the last line.")
                    return None, None
            return None, None
    except FileNotFoundError:
        print(f"File not found at: {file_path}")
        return None, None


mj = conn.mech_jeb
smart_ass = conn.mech_jeb.smart_ass
#mj.target_controller.set_position_target(conn.space_center.target_body, -0.083, -74.533)
smart_ass.autopilot_mode =  mj.SmartASSAutopilotMode.off

mj.target_controller.set_position_target(conn.space_center.active_vessel.orbit.body,28.50889,-81.19361)

#Setting Tower Coordinates:
booster.control.sas = True
booster.control.rcs = False
booster.control.throttle = 1.0
# Stage 0 Attributes


old_tower_old_major = -81.1937
# Tower coordinates (target position)
tower_longitude = -81.19461  # Longitude of launch tower
tower_latitude = 28.509584  # Latitude of launch tower
# close catch lat 28.509844
# closer catch lat 28.509644
# lat test 28.509574
#BEST: 28.509384
#nuevo target 81.1922
landing_longitude = -81.1910
#81.19192
#-81.1912  and 1913  an 1914 and 1915 and 1916  and 1917 Close Catch




# Launch and Turn
booster.control.sas = True
booster.auto_pilot.engage()
#booster.auto_pilot.target_pitch_and_heading(87, 90)
#booster.control.toggle_action_group(4)
time.sleep(0.5)
booster.control.toggle_action_group(3)
time.sleep(2.2)
booster.control.toggle_action_group(2)
time.sleep(0.3)
booster.control.toggle_action_group(1)
time.sleep(0.2)
booster.control.activate_next_stage()
turn_start_altitude = 100
turn_end_altitude = 27500
turn_target_pitch = 27

# Gradual pitch adjustment for the turn
while True:
    booster.auto_pilot.engage()
    booster.control.sas = True
    print(read_trajectory_data(file_path))
    altitude = booster.flight().mean_altitude
    if turn_start_altitude < altitude < turn_end_altitude:
        frac = (altitude - turn_start_altitude) / (turn_end_altitude - turn_start_altitude)
        new_pitch = 90 - frac * (90 - turn_target_pitch)
        booster.auto_pilot.target_pitch_and_heading(new_pitch, 90)
    if altitude > turn_end_altitude:
        break
    time.sleep(1)

# Hotstaging
booster.control.toggle_action_group(1)
time.sleep(0.6)
booster.control.toggle_action_group(2)
time.sleep(0.6)
booster.control.toggle_action_group(3)
time.sleep(3)


# Stage Separation
booster.control.activate_next_stage()

# Change to Ship
newest_vessel_met = 99999999999
ship = 0
for vessel in conn.space_center.vessels:
    print(vessel)
    if vessel.met < newest_vessel_met:
        newest_vessel = vessel

conn.space_center.active_vessel = newest_vessel
ship = conn.space_center.active_vessel
#ship.control.activate_next_stage()
ship.control.rcs = True
smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
smart_ass.force_pitch = True
smart_ass.force_yaw = True
smart_ass.surface_pitch = 15
smart_ass.surface_heading = 90
smart_ass.surface_roll = 0
smart_ass.update(False)
time.sleep(7)

booster = 0
newest_vessel = 0
vessel_met_dict = {}
# Populate the dictionary with MET as the key and vessel as the value
for vessel in conn.space_center.vessels:
    met = vessel.met
    vessel_met_dict[vessel] = met

# Sort the vessels by MET
sorted_vessels = sorted(vessel_met_dict.keys(), key=lambda v: vessel_met_dict[v])

# Print the sorted list of vessels by MET
print("Vessels sorted by MET:")
for vessel in sorted_vessels:
    print(f"Vessel: {vessel.name}, MET: {vessel_met_dict[vessel]}")

# Optionally, set the vessel with the least MET as active
if sorted_vessels:
    least_met_vessel = sorted_vessels[1]
    conn.space_center.active_vessel = least_met_vessel
    print(f"Active vessel set to: {least_met_vessel.name}")

# Change to booster
booster = conn.space_center.active_vessel
# Target coordinates for the launch tower
target_longitude = -81.19361
target_latitude = 28.50889
# Point the booster upwards (5° pitch, heading 270°)
booster.auto_pilot.disengage()
#booster.control.toggle_action_group(4)
time.sleep(0.2)
booster.control.toggle_action_group(3)
booster.control.rcs = True
smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
smart_ass.force_pitch = True
smart_ass.force_yaw = True
smart_ass.surface_pitch = 5
smart_ass.surface_heading = 270
smart_ass.surface_roll_roll = 90
smart_ass.update(False)
booster.control.throttle = 1
time.sleep(6)
print("Switching to guidance systems")

def calculate_heading_to_target(current_lat, current_lon, target_lat, target_lon):
    # Convert degrees to radians
    current_lat_rad = math.radians(current_lat)
    current_lon_rad = math.radians(current_lon)
    target_lat_rad = math.radians(target_lat)
    target_lon_rad = math.radians(target_lon)

    # Calculate the direction vector in spherical coordinates
    delta_lon = target_lon_rad - current_lon_rad
    x = math.cos(target_lat_rad) * math.sin(delta_lon)
    y = (math.cos(current_lat_rad) * math.sin(target_lat_rad) -
         math.sin(current_lat_rad) * math.cos(target_lat_rad) * math.cos(delta_lon))

    # Calculate the initial bearing
    heading_rad = math.atan2(x, y)

    # Convert radians to degrees and normalize to 0-360°
    heading_deg = (math.degrees(heading_rad) + 360) % 360

    return heading_deg

body_radius = 1500000  # Radius of Kerbin (meters)
body_gravity = 9.81  # Gravity on Kerbin (m/s^2)

import math


def haversine(lat1, lon1, lat2, lon2):
    # Convert degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Difference in coordinates
    delta_lat = lat2_rad - lat1_rad
    delta_lon = lon2_rad - lon1_rad

    # Haversine formula
    a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Radius of the Earth in kilometers (Kerbin radius for Kerbin)
    R = 1500  # Radius of Kerbin in kilometers (change for other celestial bodies)

    # Distance in kilometers
    distance = R * c
    return distance


boostback_burn = True
boostback_outer_ring_out = False
boostback_inner_ring_out = False
booster.control.toggle_action_group(9)
booster.control.toggle_action_group(9)
mj.target_controller.set_position_target(conn.space_center.active_vessel.orbit.body,28.50889,-81.19361)
while boostback_burn:
    tLat, tLon = read_trajectory_data(file_path)
    smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
    smart_ass.force_pitch = True
    smart_ass.force_yaw = True
    smart_ass.surface_pitch = 6
    smart_ass.surface_heading = calculate_heading_to_target(tLat,tLon,
                                                            target_latitude,target_longitude)
    smart_ass.surface_roll_roll = 90
    smart_ass.update(False)
    distance_to_target = haversine(tLat, tLon, tower_latitude, tower_longitude)
    print("_________")
    print(f"Impact Point Latitude: {tLat:.6f}, Longitude: {tLon:.6f}")
    print(f"Heading AUtomated: {calculate_heading_to_target(tLat,tLon,target_latitude,target_longitude)}")
    print(f"Distance to target: {distance_to_target:.2f} km")
    print("_________")
    if (distance_to_target < 10) and (boostback_outer_ring_out == False):
        print("Outer Ring Shutdown")
        boostback_outer_ring_out = True
        booster.control.toggle_action_group(3)
        booster.control.throttle = 0.55
    elif (distance_to_target < 3) and (boostback_inner_ring_out == False):
        print("Inner Ring Shutdown")
        boostback_inner_ring_out = True
        booster.control.throttle = 0.10
    elif (distance_to_target < 2) and (boostback_inner_ring_out == True):
        print("Boostback Burn Shutdown")
        boostback_burn = False
        booster.control.throttle = 0.0

    time.sleep(0.1)
print("Coast")
smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
smart_ass.force_pitch = True
smart_ass.force_yaw = True
smart_ass.surface_pitch = 90
smart_ass.surface_heading = 270
smart_ass.surface_roll_roll = 0

wait_for_switch = True

while wait_for_switch:
    if booster.flight().surface_altitude > 80500:
        wait_for_switch = False
    else:
        pass

print("Sufficient Altitude to switch to ship")

booster = 0
ship = 0
newest_vessel = 0
vessel_met_dict = {}
# Populate the dictionary with MET as the key and vessel as the value
for vessel in conn.space_center.vessels:
    met = vessel.met
    vessel_met_dict[vessel] = met

# Sort the vessels by MET
sorted_vessels = sorted(vessel_met_dict.keys(), key=lambda v: vessel_met_dict[v])

# Print the sorted list of vessels by MET
print("Vessels sorted by MET:")
for vessel in sorted_vessels:
    print(f"Vessel: {vessel.name}, MET: {vessel_met_dict[vessel]}")

# Optionally, set the vessel with the least MET as active
if sorted_vessels:
    least_met_vessel = sorted_vessels[0]
    conn.space_center.active_vessel = least_met_vessel
    print(f"Active vessel set to: {least_met_vessel.name}")

# Change to booster
ship = conn.space_center.active_vessel

print("Switched to Ship")
print("Orbital Guidance System activated")
nominal_orbital_insertion = False

ship.control.throttle = 1.0
# Semi-Major Axis -> Largest area on eliptical orbit
# Semi-Minor Axis -> Smallest area on eliptical orbit
# Eccentricity -> 1=ElonGated 0=Circle
circular_mode = False
while nominal_orbital_insertion == False:
    if circular_mode == False:
        print("Apoapsis Mode")
        if ship.orbit.time_to_apoapsis > 60:
            smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
            smart_ass.force_pitch = True
            smart_ass.force_yaw = True
            smart_ass.surface_pitch = -30
            smart_ass.surface_heading = 90
            smart_ass.surface_roll_roll = 0
            smart_ass.update(False)
        elif ship.orbit.time_to_apoapsis > 40:
            smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
            smart_ass.force_pitch = True
            smart_ass.force_yaw = True
            smart_ass.surface_pitch = -30
            smart_ass.surface_heading = 90
            smart_ass.surface_roll_roll = 0
            smart_ass.update(False)
        elif ship.orbit.time_to_apoapsis > 20:
            smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
            smart_ass.force_pitch = True
            smart_ass.force_yaw = True
            smart_ass.surface_pitch = -25
            smart_ass.surface_heading = 90
            smart_ass.surface_roll_roll = 0
            smart_ass.update(False)
        elif ship.orbit.time_to_apoapsis > 10:
            smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
            smart_ass.force_pitch = True
            smart_ass.force_yaw = True
            smart_ass.surface_pitch = -15
            smart_ass.surface_heading = 90
            smart_ass.surface_roll_roll = 0
            smart_ass.update(False)
        elif ship.orbit.time_to_apoapsis > 3:
            smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
            smart_ass.force_pitch = True
            smart_ass.force_yaw = True
            smart_ass.surface_pitch = -5
            smart_ass.surface_heading = 90
            smart_ass.surface_roll_roll = 0
            smart_ass.update(False)
        if ship.orbit.time_to_apoapsis < 3:
            smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
            smart_ass.force_pitch = True
            smart_ass.force_yaw = True
            smart_ass.surface_pitch = -1
            smart_ass.surface_heading = 90
            smart_ass.surface_roll_roll = 0
            smart_ass.update(False)
            circular_mode = True
        if ship.orbit.time_to_apoapsis > 500:
            smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
            smart_ass.force_pitch = True
            smart_ass.force_yaw = True
            smart_ass.surface_pitch = 15
            smart_ass.surface_heading = 90
            smart_ass.surface_roll_roll = 0
            smart_ass.update(False)
            circular_mode = True
    else:
        if ship.orbit.time_to_apoapsis > 500:
            smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
            smart_ass.force_pitch = True
            smart_ass.force_yaw = True
            smart_ass.surface_pitch = 10
            smart_ass.surface_heading = 90
            smart_ass.surface_roll_roll = 0
            smart_ass.update(False)
        elif ship.orbit.time_to_apoapsis > 3:
            circular_mode = False
    if (ship.orbit.periapsis_altitude > 84000) and (ship.orbit.apoapsis_altitude > 84000):
        ship.control.throttle = 0
        nominal_orbital_insertion = True
        ship.control.throttle = 0
        print("Nominal Orbital Insertion")


print("Switching to Booster")
ship.control.throttle = 0.0
ship.control.throttle = 0.0
time.sleep(10)

booster = 0
ship = 0
newest_vessel = 0
vessel_met_dict = {}
# Populate the dictionary with MET as the key and vessel as the value
for vessel in conn.space_center.vessels:
    met = vessel.met
    vessel_met_dict[vessel] = met

# Sort the vessels by MET
sorted_vessels = sorted(vessel_met_dict.keys(), key=lambda v: vessel_met_dict[v])

# Print the sorted list of vessels by MET
print("Vessels sorted by MET:")
for vessel in sorted_vessels:
    print(f"Vessel: {vessel.name}, MET: {vessel_met_dict[vessel]}")

# Optionally, set the vessel with the least MET as active
if sorted_vessels:
    least_met_vessel = sorted_vessels[1]
    conn.space_center.active_vessel = least_met_vessel
    print(f"Active vessel set to: {least_met_vessel.name}")

# Change to booster
booster = conn.space_center.active_vessel

mj.target_controller.set_position_target(conn.space_center.active_vessel.orbit.body,28.50889,-81.19361)
pre_aerodynamic_control = True
while pre_aerodynamic_control:
    smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
    smart_ass.force_pitch = True
    smart_ass.force_yaw = True
    smart_ass.surface_pitch = 90
    smart_ass.surface_heading = 90
    smart_ass.surface_roll_roll = 0
    smart_ass.update(False)

    if booster.flight().surface_altitude < 45000:
        pre_aerodynamic_control = False
print("Switching to Aerodynamic Control")
smart_ass.surface_roll = 90
smart_ass.update(True)
# RESET TELEMETRY

booster.control.toggle_action_group(9)
booster.control.toggle_action_group(9)
def haversine(lat1, lon1, lat2, lon2):
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    delta_lat = lat2_rad - lat1_rad
    delta_lon = lon2_rad - lon1_rad

    a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    R = 1500  # Radius of Kerbin in kilometers

    return R * c  # Distance in kilometers

# Function to calculate dynamic pitch based on impact point distance
def calculate_dynamic_pitch(impact_distance):
    if booster.flight().surface_altitude > 22000:
        return 80
    else:
        if impact_distance < 0.001:
            return 90
        elif impact_distance < 0.005:
            return 90
        elif impact_distance < 0.01:
            return 88
        elif impact_distance < 0.02:
            return 87
        elif impact_distance < 0.03:
            return 85
        elif impact_distance < 0.07:
            return 75
        elif impact_distance < 0.13:
            return 70
        elif impact_distance < 0.23:
            return 65
        elif impact_distance < 0.3:
            return 60
        elif impact_distance < 0.6:
            return 50
        elif impact_distance < 1:
            return 45
        elif impact_distance < 2:
            return 30
        elif impact_distance >= 2:
            return 20

def calculate_dynamic_pitch_tower_close(impact_distance):
    if booster.flight().surface_altitude > 22000:
        return 80
    else:
        if impact_distance < 0.001:
            return 90
        #90
        elif impact_distance < 0.005:
            return 90
        #90
        elif impact_distance < 0.01:
            return 89
        #89
        elif impact_distance < 0.02:
            return 87
        #87
        elif impact_distance < 0.03:
            return 86
        #86
        elif impact_distance < 0.07:
            return 86
        #86
        elif impact_distance < 0.13:
            return 85
        elif impact_distance < 0.23:
            return 85
        elif impact_distance < 0.3:
            return 70
        elif impact_distance < 0.6:
            return 50
        elif impact_distance < 1:
            return 45
        elif impact_distance < 2:
            return 30
        elif impact_distance >= 2:
            return 20

def calculate_dyamic_pitch_catch(impact_distance):
    if booster.flight().surface_altitude > 100:
        if impact_distance < 0.0005:
            return 90
        elif impact_distance < 0.001:
            return 88
        elif impact_distance < 0.002:
            return 87
        elif impact_distance < 0.003:
            return 86
        elif impact_distance < 0.004:
            return 85
        elif impact_distance < 0.005:
            return 84
        elif impact_distance < 0.007:
            return 83
        elif impact_distance < 0.008:
            return 82
        elif impact_distance < 0.009:
            return 81
        elif impact_distance < 0.01:
            return 80
        elif impact_distance > 0.01:
            return 78
    else:
        if impact_distance < 0.0001:
            return 90
        elif impact_distance < 0.0002:
            return 87
        elif impact_distance < 0.0005:
            return 85
        elif impact_distance < 0.001:
            return 83
        elif impact_distance < 0.0015:
            return 81
        elif impact_distance >= 0.0015:
            return 80


# Function to calculate the heading towards the reverse direction from the tower
def calculate_suicide_burn_altitude(speed, thrust, mass):
    gravity = 9.81
    a_max = thrust / mass
    if a_max <= gravity:
        pass
    a = a_max - gravity
    h = (speed**2) / (2*a)
    return h


def calculate_throttle(speed, thrust_max, mass, height_target):
    a_req = (speed ** 2) / (2*height_target)
    total_a = a_req + 9.81
    throttle_percentage = (total_a * mass / thrust_max)
    if throttle_percentage > 1.0:
        throttle_percentage = 1.0
    return  throttle_percentage



def calculate_heading_to_target(current_lat, current_lon, target_lat, target_lon):
    x_diff = current_lon - target_lon
    y_diff = current_lat - target_lat
    radians_theta = math.atan2(y_diff, x_diff)
    theta = math.degrees(radians_theta)
    possible_angle = 270 - theta + 180
    if possible_angle < 0:
        angle = possible_angle + 360
    else:
        angle = possible_angle
    return angle




 # Reverse the heading to go away from the tower
landing_state = True
# Main function to perform aerodescent
def aero_descent():
    booster.control.toggle_action_group(9)
    booster.control.toggle_action_group(9)
    # Deactivate outer engines (Action Groups 1 and 2)
    booster.control.toggle_action_group(1)  # Outer engines off
    booster.control.toggle_action_group(2)  # Outer engines off
    print("Outer Engines off")
    # Full throttle for aerodescent
    #booster.control.throttle = 1.0
    landing_state = True
    # Perform aerodescent towards the target (tower)
    while landing_state:

        smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.surface
        smart_ass.interface_mode = mj.SmartASSInterfaceMode.surface

        tLat, tLon = read_trajectory_data(file_path)

        if tLat is None or tLon is None:
            print("Failed to read trajectory data.")
            break

        # Calculate the distance from the impact point to the tower
        distance_to_target = haversine(tLat, tLon, tower_latitude, tower_longitude)
        booster_to_tower = haversine(tLat,tLon,28.509885,-81.19371)

        if (booster.flight().mach * booster.flight().speed_of_sound) < 40:
            pitch = calculate_dynamic_pitch_tower_close(booster_to_tower)
        elif (booster.flight().mach * booster.flight().speed_of_sound) < 80:
            pitch = calculate_dynamic_pitch_tower_close(booster_to_tower)
        elif (booster.flight().mach * booster.flight().speed_of_sound) < 280:
            pitch = calculate_dynamic_pitch_tower_close(booster_to_tower)
        else:
            pitch = calculate_dynamic_pitch(distance_to_target)

        # Calculate the heading towards the reverse direction of the tower
        heading = calculate_heading_to_target(tLat, tLon,
                                              tower_latitude, tower_longitude)
        if (booster.flight().mach * booster.flight().speed_of_sound) < 320:
            heading = calculate_heading_to_target(tLat, tLon,
                                                  tower_latitude, landing_longitude)
        else:
            heading = calculate_heading_to_target(tLat, tLon,
                                                  tower_latitude, tower_longitude)

        vertical_speed = math.sin(pitch * (math.pi/180)) * booster.flight().mach * booster.flight().speed_of_sound
        throttle_calculated = calculate_throttle((booster.flight().mach * booster.flight().speed_of_sound),booster.max_thrust,booster.mass,80)
        throttle_three_engines_calculated = calculate_throttle((booster.flight().mach * booster.flight().speed_of_sound),booster.max_thrust,booster.mass,70)
        # Set the SmartASS autopilot to control the heading and pitch dynamically

        if booster.flight().surface_altitude < 77.5:
            time.sleep(1)
            booster.control.throttle = 0.0
            smart_ass.autopilot_mode = mj.SmartASSAutopilotMode.off
            landing_state = False
        elif (booster.flight().mach * booster.flight().speed_of_sound) < 16:
            booster.control.set_action_group(3, True)
            smart_ass.surface_roll = 90
            smart_ass.force_roll = True
            smart_ass.update(False)
            booster.control.throttle = throttle_three_engines_calculated
            # 1.0 THROTTLE
            # test 12
        elif (booster.flight().mach * booster.flight().speed_of_sound) < 60:
            booster.control.throttle = throttle_calculated
            # 0.5 THROTTLE
            smart_ass.surface_roll = 90
            smart_ass.force_roll = True
            smart_ass.update(False)
            #alt 10550
            # close catch 80
            # test 70
            # test 65
        elif booster.flight().surface_altitude < 8620 and (booster.flight().mach * booster.flight().speed_of_sound) > 60:
            #9850 close catch
            #9825 close catch -> better?
            #9810 test NO
            booster.control.throttle = throttle_calculated
            #1.0 THROTTLE
            smart_ass.surface_roll = 90
            smart_ass.force_roll = True
            smart_ass.update(False)

        smart_ass.force_roll = True
        smart_ass.force_pitch = True
        smart_ass.force_yaw = True
        if (booster.flight().mach * booster.flight().speed_of_sound) < 80:
            smart_ass.surface_heading = heading + 180
            #smart_ass.surface_heading = heading
        elif (booster.flight().mach * booster.flight().speed_of_sound) < 150:
            smart_ass.surface_heading = heading + 180
            #smart_ass.surface_heading = heading
        elif (booster.flight().mach * booster.flight().speed_of_sound) < 300:
            smart_ass.surface_heading = heading + 180
            #smart_ass.surface_heading = heading
        else:
            smart_ass.surface_heading = heading

        if (booster.flight().mach * booster.flight().speed_of_sound) < 300:
            smart_ass.surface_pitch = pitch
        else:
            smart_ass.surface_pitch = pitch - 2
        smart_ass.surface_roll = 90
        smart_ass.update(True)

        print(f"Distance to target: {distance_to_target:.2f} km")
        print(f"Pitch: {pitch}, Heading: {heading}")
        print(f"ALT: {booster.flight().surface_altitude}")
        print(f"Suicide Alt: {calculate_suicide_burn_altitude((booster.flight().mach * booster.flight().speed_of_sound),booster.max_thrust,booster.mass)}")
        print(f"Throttle: {calculate_throttle((booster.flight().mach * booster.flight().speed_of_sound),booster.max_thrust,booster.mass,10)}")


aero_descent()

print("----------  END OF FILE   --------")