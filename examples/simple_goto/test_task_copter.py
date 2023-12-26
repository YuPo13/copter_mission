from __future__ import print_function
import time, math
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect('udpin:127.0.0.1:14551')


def distance_calculation(home_lattitude, home_longitude, destination_lattitude, destination_longitude):
    """
    This function returns the distance between two geographiclocations using
    the haversine formula.

    Inputs:
        1.  home_lattitude          -   Home or Current Location's  Latitude
        2.  home_longitude          -   Home or Current Location's  Longitude
        3.  destination_lattitude   -   Destination Location's  Latitude
        4.  destination_longitude   -   Destination Location's  Longitude

    """
    # Radius of earth in metres
    radius = 6371e3

    rlat1, rlon1 = home_lattitude * (math.pi / 180), home_longitude * (math.pi / 180)
    rlat2, rlon2 = destination_lattitude * (math.pi / 180), destination_longitude * (math.pi / 180)
    dlat = (destination_lattitude - home_lattitude) * (math.pi / 180)
    dlon = (destination_longitude - home_longitude) * (math.pi / 180)

    # Haversine formula to find distance
    a = (math.sin(dlat/2) * math.sin(dlat/2)) + (math.cos(rlat1) * math.cos(rlat2) * (math.sin(dlon/2) * math.sin(dlon/2)))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    # Distance (in meters)
    distance = radius * c

    return distance
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def goto(new_lat, new_lon, goto_function=vehicle.simple_goto):
    current_location = vehicle.location.global_relative_frame
    target_location = LocationGlobalRelative(new_lat, new_lon, current_location.alt)
    target_distance = get_distance_metres(current_location, target_location)
    goto_function(target_location)

    while vehicle.mode.name == "GUIDED": #Stop action if we are no longer in guided mode.
        remaining_distance = get_distance_metres(vehicle.location.global_frame, target_location)
        print("Distance to target:{}".format(remaining_distance))
        if remaining_distance <= target_distance * 0.01: #Just below target, in case of undershoot.
            print ('Reached target')
            break
        time.sleep(2)


def arm_and_takeoff(a_target_altitude):
    """
    a_target_altitude - the target altitude for the copter
    This method arms vehicle and fly to a_target_altitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(a_target_altitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= a_target_altitude:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(100)
time.sleep(1)


# sleep so we can see the change in map
time.sleep(5)

print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
print('Current lan before goto: {}'.format(vehicle.location.global_relative_frame.lat))
point2 = LocationGlobalRelative(50.443326, 30.448078, 100)

goto(point2.lat, point2.lon)

time.sleep(5)

msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        305,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        0, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
vehicle.send_mavlink(msg)

print("Returning to Launch")
vehicle.parameters['RTL_ALT'] = 0
vehicle.mode = VehicleMode("RTL")
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
