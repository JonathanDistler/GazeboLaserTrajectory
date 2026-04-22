import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
#controls the drone to a specific heading, granted there is some instability in the simulation so it isn't completely static 

TARGET_N = 1732.05080757 #changes to dy being 1732.05080757 instead of 1936
TARGET_E = 0.0
TARGET_D = -1000.0 #changes height to be 1000 instead of 500

async def run():

    drone = System()
    await drone.connect(system_address="udp://127.0.0.1:14580")

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    print("Arming...")
    await drone.action.arm()

    # -----------------------------
    # TAKEOFF FIRST
    # -----------------------------
    print("Takeoff...")
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # -----------------------------
    # CRITICAL FIX:
    # SEND INITIAL SETPOINT RIGHT BEFORE OFFBOARD - gazebo doesn't do well with huge command positions to begin with 
    # -----------------------------
    print("Sending initial setpoint...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -10.0, 0.0)
    )

    await asyncio.sleep(0.1)

    # -----------------------------
    # START OFFBOARD
    # -----------------------------
    print("Starting OFFBOARD...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Failed to start offboard: {e}")
        await drone.action.disarm()
        return

    # -----------------------------
    # NOW STREAM CONTINUOUSLY
    # -----------------------------
    print("Streaming target...")

    while True:
        await drone.offboard.set_position_ned(
            PositionNedYaw(TARGET_N, TARGET_E, TARGET_D, 0.0)
        )
        await asyncio.sleep(0.05)

if __name__ == "__main__":
    asyncio.run(run())