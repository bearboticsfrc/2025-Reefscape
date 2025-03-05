from networktables import NetworkTables
from time import sleep
from os import system

TEAM_NUMBER = 40, 68

def main() -> None:
    NetworkTables.initialize(server=f"10.{TEAM_NUMBER[0]}.{TEAM_NUMBER[1]}.2")

    while True:
        sleep(0.5)

        current_pose = NetworkTables.getTable("DriveState").getNumberArray("Pose", None)
        target_pose = NetworkTables.getTable("Robot").getSubTable("m_robotContainer").getNumberArray("Auto Start Pose", None)

        if target_pose is None or current_pose is None:
            continue

        print(current_pose, target_pose)

if __name__ == "__main__":
    main()