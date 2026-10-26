#!/usr/bin/env python3
"""Regenerate converted C# files from the Box2D C sources and apply their manual patches.

usage: port.py <box2d root> [name ...]

Each entry maps a C source file to a C# file and an optional patch script in patches/.
Without names every entry is regenerated.
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
LIB = os.path.join(REPO, "src", "Box2DNet")

# name: (C sources, output relative to src/Box2DNet)
FILES = {
    "manifold": (["src/manifold.c"], "Collision/Manifold.cs"),
    "dynamic_tree": (["src/dynamic_tree.c"], "Collision/DynamicTree.cs"),
    "mover": (["src/mover.c"], "Collision/Mover.cs"),
    "types": (["include/box2d/types.h"], "Types.cs"),
    "body_h": (["src/body.h"], "Dynamics/BodyTypes.cs"),
    "shape_h": (["src/shape.h"], "Dynamics/ShapeTypes.cs"),
    "contact_h": (["src/contact.h"], "Dynamics/ContactTypes.cs"),
    "joint_h": (["src/joint.h"], "Joints/JointTypes.cs"),
    "island_h": (["src/island.h"], "Dynamics/IslandTypes.cs"),
    "solver_set_h": (["src/solver_set.h"], "Dynamics/SolverSetTypes.cs"),
    "constraint_graph_h": (["src/constraint_graph.h"], "Dynamics/ConstraintGraphTypes.cs"),
    "broad_phase_h": (["src/broad_phase.h"], "Dynamics/BroadPhaseTypes.cs"),
    "sensor_h": (["src/sensor.h"], "Dynamics/SensorTypes.cs"),
    "physics_world_h": (["src/physics_world.h"], "Dynamics/WorldTypes.cs"),
    "solver_h": (["src/solver.h"], "Dynamics/SolverTypes.cs"),
    "contact_solver_h": (["src/contact_solver.h"], "Dynamics/ContactSolverTypes.cs"),
    "types_c": (["src/types.c"], "Dynamics/Defaults.cs"),
    "body": (["src/body.c"], "Dynamics/Body.cs"),
    "shape": (["src/shape.c"], "Dynamics/Shape.cs"),
    "contact": (["src/contact.c"], "Dynamics/Contact.cs"),
    "sensor": (["src/sensor.c"], "Dynamics/Sensor.cs"),
    "island": (["src/island.c"], "Dynamics/Island.cs"),
    "solver_set": (["src/solver_set.c"], "Dynamics/SolverSet.cs"),
    "constraint_graph": (["src/constraint_graph.c"], "Dynamics/ConstraintGraph.cs"),
    "broad_phase": (["src/broad_phase.c"], "Dynamics/BroadPhase.cs"),
    "physics_world": (["src/physics_world.c"], "Dynamics/World.cs"),
    "solver": (["src/solver.c"], "Dynamics/Solver.cs"),
    "contact_solver": (["src/contact_solver.c"], "Dynamics/ContactSolver.cs"),
    "joint": (["src/joint.c"], "Joints/Joint.cs"),
    "distance_joint": (["src/distance_joint.c"], "Joints/DistanceJoint.cs"),
    "motor_joint": (["src/motor_joint.c"], "Joints/MotorJoint.cs"),
    "mover_joint": (["src/mover_joint.c"], "Joints/MoverJoint.cs"),
    "pogo_joint": (["src/pogo_joint.c"], "Joints/PogoJoint.cs"),
    "prismatic_joint": (["src/prismatic_joint.c"], "Joints/PrismaticJoint.cs"),
    "revolute_joint": (["src/revolute_joint.c"], "Joints/RevoluteJoint.cs"),
    "weld_joint": (["src/weld_joint.c"], "Joints/WeldJoint.cs"),
    "wheel_joint": (["src/wheel_joint.c"], "Joints/WheelJoint.cs"),
}


def port(root, name):
    sources, out = FILES[name]
    out_path = os.path.join(LIB, out)
    cmd = [sys.executable, os.path.join(HERE, "c2cs.py"), root] + sources
    text = subprocess.run(cmd, check=True, capture_output=True, text=True, encoding="utf-8").stdout
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    patch = os.path.join(HERE, "patches", name + ".py")
    if os.path.exists(patch):
        subprocess.run([sys.executable, patch, out_path], check=True)
    print("ported", name, "->", out)


def main():
    root = sys.argv[1]
    names = sys.argv[2:] or list(FILES)
    for name in names:
        port(root, name)


if __name__ == "__main__":
    main()
