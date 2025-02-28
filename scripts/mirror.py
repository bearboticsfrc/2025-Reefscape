import json

def mirror_rotation(rotation: float) -> float:
    return -rotation

def mirror_translation(x: float, y: float) -> tuple[float, float]:
    return x, 8.0519016 - y

def main(file: str) -> None:
    with open(file) as fp:
        path = json.load(fp)

    end_rotation = path["goalEndState"]["rotation"]
    ideal_rotation = path["idealStartingState"]["rotation"]

    for i, waypoint in enumerate(path["waypoints"]):
        point = waypoint["anchor"]

        x, y = mirror_translation(point["x"], point["y"])

        path["waypoints"][i]["anchor"]["x"] = x
        path["waypoints"][i]["anchor"]["y"] = y

        previous_control = waypoint["prevControl"]

        if previous_control:
            point = previous_control

            x, y = mirror_translation(point["x"], point["y"])

            path["waypoints"][i]["prevControl"]["x"] = x
            path["waypoints"][i]["prevControl"]["y"] = y

        next_control = waypoint["nextControl"]

        if next_control:
            point = next_control

            x, y = mirror_translation(point["x"], point["y"])

            path["waypoints"][i]["nextControl"]["x"] = x
            path["waypoints"][i]["nextControl"]["y"] = y

    for i, rotation_target in enumerate(path["rotationTargets"]):
        rotation = rotation_target["rotationDegrees"]

        path["rotationTargets"][i]["rotationDegrees"] = mirror_rotation(rotation)

    path["goalEndState"]["rotation"] = mirror_rotation(end_rotation)
    path["idealStartingState"]["rotation"] = mirror_rotation(ideal_rotation)

    with open(file[:-5] + ".mirrored.path", "w") as fp:
        json.dump(path, fp)

main(r"C:\Users\kelle\Documents\Programming\Java\Bearbotics\2025-Reefscape\scripts\deploy\pathplanner\paths\CToPStation.path")