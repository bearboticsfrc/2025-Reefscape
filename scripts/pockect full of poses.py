from networktables import NetworkTables
from time import sleep
from itertools import cycle
from os import system
import platform

TEAM_NUMBER = 40, 68

DISCONNECTED_WAIT = 1

CURRENT_POSE_PATH = "/DriveState/Pose"
TARGET_POSE_PATH = "/Robot/m_robotContainer/Auto Start Pose"

WAIT_ICON_CYCLE = cycle(("|", "/", "-", "\\"))

def clear_screen() -> None:
    system("cls" if platform.system() == "Windows" else "clear")

def get_target_pose() -> tuple[float, float, float]:
    """Get the target pose from NT  using TARGET_POSE_PATH as the entry path..
    
    :return: tuple[float, float, float] The Pose represented as a tuple of (x, y, theta)"""
    
    return NetworkTables.getEntry(TARGET_POSE_PATH).value

def get_current_pose() -> tuple[float, float, float]:
    """Get the current pose from NT using CURRENT_POSE_PATH as the entry path.
    
    :return: tuple[float, float, float] The Pose represented as a tuple of (x, y, theta)"""
    
    return NetworkTables.getEntry(CURRENT_POSE_PATH).value

def main() -> None:
    clear_screen()

    NetworkTables.initialize(server=f"10.{TEAM_NUMBER[0]}.{TEAM_NUMBER[1]}.2")

    while not NetworkTables.isConnected():
        print(f"[{next(WAIT_ICON_CYCLE)}] NT disconnected! Is the robot on?", end="\r")
        sleep(0.25)
    else:
        clear_screen()

    while True:
        sleep(0.25)

        current_pose = get_current_pose()
        target_pose = get_target_pose()

        if target_pose is None or current_pose is None:
            print(f"[{next(WAIT_ICON_CYCLE)}] Target Pose / Current Pose is null!", end="\r")
            continue

        print(current_pose, target_pose)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        NetworkTables.shutdown()