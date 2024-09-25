from pymavlink import mavutil
import asyncio
import threading
import time

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

#udpin:localhost:14550

# Wait for the first heartbeat
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Function to receive MAVLink messages
#def receive_messages():
 #   while True:
  #      msg = the_connection.recv_match(blocking=True)
   #     if msg:
    #        print("Received message:", msg)

# Start a thread for receiving messages
#receive_thread = threading.Thread(target=receive_messages, daemon=True)
#receive_thread.start()

# Function to send periodic heartbeats
def send_heartbeats():
    while True:
        the_connection.mav.heartbeat_send(0, 0, 0, 0, 0)  # Sending a heartbeat
        time.sleep(1)  # Send a heartbeat every second

# Start a thread for sending heartbeats
heartbeat_thread = threading.Thread(target=send_heartbeats, daemon=True)
heartbeat_thread.start()

#FOR GUIDED MODE
async def set_mode(mode):
    mode_id = the_connection.mode_mapping()[mode]
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    ack = False
    while not ack:
        ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        ack = ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE
        await asyncio.sleep(1)

async def arm_drone():
    print("Arming drone...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    await asyncio.sleep(1)
    await change_speed()

async def change_speed():
    print("Changing speed...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, 0, 1, 0, 0, 0, 0, 0
    )
    await asyncio.sleep(1)
    await takeoff()

async def takeoff():
    print("Taking off...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 1
    )
    await asyncio.sleep(10)
    await move_to_local1()

async def move_to_local1():
    print("Moving to LOCAL1...")
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b100111111000), 2, 0, -1, 
        0, 0, 0, 0, 0, 0, 0, 0
    ))
    await asyncio.sleep(20)
    await move_to_local2()

async def move_to_local2():
    print("Moving to LOCAL2...")
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b100111111000), 2, 2, -1,
        0, 0, 0, 0, 0, 0, 0, 0
    ))
    await asyncio.sleep(30)
    await move_to_local3()

async def move_to_local3():
    print("Moving to LOCAL3...")
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b100111111000), 0, 2, -1,
        0, 0, 0, 0, 0, 0, 0, 0
    ))
    await asyncio.sleep(40)
    await move_to_local0()

async def move_to_local0():
    print("Moving to LOCAL0...")
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b100111111000), 0, 0, -1,
        0, 0, 0, 0, 0, 0, 0, 0
    ))
    await asyncio.sleep(50)
    await land_drone()

async def land_drone():
    print("Landing drone...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, -1
    )

async def main():
    await arm_drone()

# Run the asyncio event loop in the main thread
asyncio.run(main())

# Keep the script running
while True:
    time.sleep(1)  # Prevent the main thread from exiting
