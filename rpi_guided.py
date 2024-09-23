from pymavlink import mavutil
import asyncio
import threading
import time

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for the first heartbeat
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Function to send periodic heartbeats
def send_heartbeats():
    while True:
        the_connection.mav.heartbeat_send(0, 0, 0, 0, 0)  # Sending a heartbeat
        time.sleep(1)  # Send a heartbeat every second

# Start a thread for sending heartbeats
heartbeat_thread = threading.Thread(target=send_heartbeats, daemon=True)
heartbeat_thread.start()

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
#         await asyncio.sleep(1)



async def main():
    await set_mode('GUIDED')
    await asyncio.sleep(2)
#    await arm_drone()

# Run the asyncio event loop in the main thread
asyncio.run(main())

# Keep the script running
while True:
    time.sleep(1)  # Prevent the main thread from exiting
