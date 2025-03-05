import argparse
from pathlib import Path
import time
import json

def mirror_rotation(rotation: float) -> float:
    return -rotation

def mirror_translation(x: float, y: float) -> tuple[float, float]:
    return x, 8.0519016 - y

def mirror_path(file: Path) -> None:
    print(f"  Mirroring: {file.name!r}")
    with file.open() as fp:
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

    new_path = "M-"+ str(file.resolve())[:-5] + ".path"

    with open(new_path, "w") as fp:
        json.dump(path, fp)

    return Path(new_path)

def mirror_auto(file: Path) -> None:
    print(f"\nMirroring: {file.name!r}:")

    with file.open() as fp:
        auto = json.load(fp)

    for i, path in enumerate(auto["command"]["data"]["commands"]):
        if path["type"] != "path":
            continue

        path_path = path["data"]["pathName"] + ".path"

        path = (file.parent.parent / "paths" / path_path)

        if not path.exists():
            raise RuntimeError(f"Couldn't find actual path to {path_path}!")
        
        new_path = mirror_path(path)
        auto["command"]["data"]["commands"][i]["data"]["pathName"] = new_path.name[:-5]

    new_auto = "M-" + str(file.resolve())[:-5] + ".auto"

    with open(new_auto, "w") as fp:
        json.dump(auto, fp)

def main() -> None:
    parser = argparse.ArgumentParser(description="Mirror all Pathplanner paths in an auto.")

    parser.add_argument("path", help="Path to the Pathplanner auto.", type=Path)
    args = parser.parse_args()

    start = time.perf_counter()
    count = 0

    if args.path.is_dir():
        for file in args.path.rglob("*"):
            if not file.is_file() or file.suffix != ".auto" or file.name.startswith("M-"):
                continue

            count += 1
            mirror_auto(file)
    else:
        count += 1
        mirror_auto(args.path)

    if count:
        print(f"\nFinished mirroring {count} {'auto' if count == 1 else 'autos'} in {time.perf_counter() - start:.02f}s.")
    else:
        print("No autos to mirror!")

if __name__ == "__main__":
    main()