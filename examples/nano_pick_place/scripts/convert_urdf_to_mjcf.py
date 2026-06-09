#!/usr/bin/env python3
"""
Convert ADORA1 Nano URDF to MuJoCo MJCF
=========================================

Reads the ground-truth URDF, uses MuJoCo's URDF compiler to produce
correct mesh placements, then post-processes the XML to add actuators,
ground plane, target object, and keyframe.

Usage:
    cd examples/nano_pick_place
    python scripts/convert_urdf_to_mjcf.py

Output:
    models/nano_full.xml
"""

import glob
import os
import re
import struct
import xml.etree.ElementTree as ET

import mujoco

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
ASSETS_DIR = os.path.join(PROJECT_DIR, "models", "nano_assets")
OUTPUT_PATH = os.path.join(PROJECT_DIR, "models", "nano_full.xml")
URDF_SOURCE = os.path.expanduser(
    "~/Public/github_nano_3D/nano/nano.urdf"
)

MUJOCO_MAX_FACES = 200000

# Arm revolute joint names (from URDF) — the only actuated joints
ARM_JOINT_NAMES = [
    "STS3215_03a-v1_Revolute-45",       # base rotation
    "STS3215_03a-v1-1_Revolute-49",     # shoulder
    "STS3215_03a-v1-2_Revolute-51",     # elbow
    "STS3215_03a-v1-3_Revolute-53",     # wrist pitch
    "STS3215_03a_Wrist_Roll-v1_Revolute-55",  # wrist roll
    "STS3215_03a-v1-4_Revolute-57",     # gripper
]


def decimate_oversized_meshes():
    """Decimate STL meshes that exceed MuJoCo's face limit (200K)."""
    import trimesh

    for stl_path in glob.glob(os.path.join(ASSETS_DIR, "*.stl")):
        with open(stl_path, "rb") as f:
            f.read(80)  # skip header
            nfaces = struct.unpack("<I", f.read(4))[0]

        if nfaces <= MUJOCO_MAX_FACES:
            continue

        name = os.path.basename(stl_path)
        target_faces = MUJOCO_MAX_FACES - 1000  # margin
        ratio = target_faces / nfaces
        print(f"  Decimating {name}: {nfaces} -> ~{target_faces} faces (ratio={ratio:.3f})")

        mesh = trimesh.load(stl_path)
        decimated = mesh.simplify_quadric_decimation(face_count=target_faces)
        decimated.export(stl_path)
        print(f"    Done: {len(decimated.faces)} faces")


def strip_mesh_prefix(urdf_text: str) -> str:
    """Remove 'meshes/' directory prefix from mesh filenames.

    MuJoCo's URDF parser resolves mesh paths relative to the URDF file
    location, so we place the URDF alongside the STLs and strip the
    subdirectory prefix.
    """
    return re.sub(r'filename="meshes/', 'filename="', urdf_text)


def load_and_compile_urdf() -> str:
    """Load URDF via MuJoCo compiler, return compiled MJCF XML string."""
    with open(URDF_SOURCE, "r") as f:
        urdf_text = f.read()

    urdf_text = strip_mesh_prefix(urdf_text)

    # Write temp URDF into assets dir so mesh paths resolve
    tmp_urdf = os.path.join(ASSETS_DIR, "_nano_temp.urdf")
    try:
        with open(tmp_urdf, "w") as f:
            f.write(urdf_text)

        model = mujoco.MjModel.from_xml_path(tmp_urdf)
        print(f"URDF compiled: nq={model.nq}, nv={model.nv}, "
              f"nbody={model.nbody}, njnt={model.njnt}, ngeom={model.ngeom}")

        # Save compiled MJCF to temp file
        tmp_mjcf = os.path.join(ASSETS_DIR, "_nano_temp.xml")
        mujoco.mj_saveLastXML(tmp_mjcf, model)

        with open(tmp_mjcf, "r") as f:
            mjcf_text = f.read()

        return mjcf_text
    finally:
        for tmp in [tmp_urdf, os.path.join(ASSETS_DIR, "_nano_temp.xml")]:
            if os.path.exists(tmp):
                os.remove(tmp)


def post_process(mjcf_text: str) -> str:
    """Add actuators, ground plane, target object, and keyframe."""
    root = ET.fromstring(mjcf_text)
    root.set("model", "nano_full")

    # --- Compiler: set meshdir ---
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("meshdir", "nano_assets")

    # --- Raise robot so wheels touch ground ---
    # Wheel centers are at z=0.018, wheel radius ~0.051, so bottom at z=-0.033.
    # Wrap all worldbody children in a body raised by 0.035m (small margin).
    worldbody = root.find("worldbody")
    z_offset = 0.035  # raise so wheel bottoms just touch ground
    robot_body = ET.SubElement(worldbody, "body")
    robot_body.set("name", "robot_base")
    robot_body.set("pos", f"0 0 {z_offset}")
    # Move all existing worldbody children into the robot_base body
    children_to_move = list(worldbody)
    for child in children_to_move:
        if child is not robot_body:
            worldbody.remove(child)
            robot_body.append(child)

    # --- Print joint info for debugging ---
    print("\n=== Joint index mapping ===")
    joint_qpos_map = {}
    qpos_idx = 0

    # Collect all joints by traversing the XML
    all_joints = root.findall(".//joint")
    for j in all_joints:
        jname = j.get("name", "unnamed")
        jtype = j.get("type", "hinge")
        jrange = j.get("range", "")
        print(f"  Joint: {jname}  type={jtype}  range={jrange}")

    # --- Add actuators ---
    actuator_elem = root.find("actuator")
    if actuator_elem is not None:
        root.remove(actuator_elem)
    actuator_elem = ET.SubElement(root, "actuator")

    for i, jname in enumerate(ARM_JOINT_NAMES):
        joint_elem = root.find(f".//joint[@name='{jname}']")
        if joint_elem is None:
            print(f"  WARNING: joint '{jname}' not found in compiled MJCF!")
            continue

        jrange = joint_elem.get("range", "-2.618 2.618")
        kp = "30" if i == 5 else "50"  # lower gain for gripper
        act = ET.SubElement(actuator_elem, "position")
        act.set("name", f"act_joint{i+1}")
        act.set("joint", jname)
        act.set("kp", kp)
        act.set("ctrlrange", jrange)
        print(f"  Actuator act_joint{i+1} -> {jname}  kp={kp}  range={jrange}")

    # --- Add ground plane ---
    ground = ET.SubElement(worldbody, "geom")
    ground.set("name", "ground")
    ground.set("type", "plane")
    ground.set("size", "1 1 0.01")
    ground.set("rgba", "0.9 0.9 0.9 1")
    ground.set("pos", "0 0 0")

    # --- Add target ball (far out in front-left of arm) ---
    # Arm reaches [-0.04, 0.25] with q=[-0.069, 1.158, -1.553, 1.5, 0, 0]
    target_body = ET.SubElement(worldbody, "body")
    target_body.set("name", "target_object")
    target_body.set("pos", "-0.04 0.25 0.06")
    fj = ET.SubElement(target_body, "freejoint")
    target_geom = ET.SubElement(target_body, "geom")
    target_geom.set("type", "sphere")
    target_geom.set("size", "0.024")
    target_geom.set("rgba", "1 0 0 1")
    target_geom.set("mass", "0.005")

    # --- Add green place plate (far out in front-right, ~14cm from ball) ---
    # Arm reaches [0.10, 0.25] with q=[0.621, 2.0, -1.868, -0.136, 0, 0]
    place_plate = ET.SubElement(worldbody, "geom")
    place_plate.set("name", "place_plate")
    place_plate.set("type", "cylinder")
    place_plate.set("size", "0.04 0.003")
    place_plate.set("pos", "0.10 0.25 0.04")
    place_plate.set("rgba", "0.2 0.8 0.2 1")
    place_plate.set("mass", "0.1")
    place_plate.set("contype", "1")
    place_plate.set("conaffinity", "1")

    # --- Add light ---
    light = ET.SubElement(worldbody, "light")
    light.set("pos", "0 0 1.5")
    light.set("dir", "0 0 -1")
    light.set("diffuse", "0.8 0.8 0.8")

    # --- Keyframe (all zeros for arm + freejoint for target) ---
    keyframe_elem = root.find("keyframe")
    if keyframe_elem is not None:
        root.remove(keyframe_elem)

    # We'll add keyframe after we know the qpos size
    # For now, skip — MuJoCo will use defaults

    return ET.tostring(root, encoding="unicode")


def print_qpos_mapping(output_path: str):
    """Load the final model and print qpos index mapping."""
    model = mujoco.MjModel.from_xml_path(output_path)
    print(f"\n=== Final model stats ===")
    print(f"  nq={model.nq}  nv={model.nv}  nu={model.nu}")
    print(f"  njnt={model.njnt}  nbody={model.nbody}  ngeom={model.ngeom}")

    print(f"\n=== qpos index mapping ===")
    arm_indices = []
    for i in range(model.njnt):
        jname = model.joint(i).name
        jtype = int(model.jnt_type[i])
        qpos_start = int(model.jnt_qposadr[i])
        type_names = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}
        tname = type_names.get(jtype, f"type{jtype}")
        nq = 7 if jtype == 0 else (4 if jtype == 1 else 1)
        print(f"  joint[{i}] {jname:50s}  type={tname:6s}  qpos[{qpos_start}:{qpos_start+nq}]")

        if jname in ARM_JOINT_NAMES:
            arm_indices.append(qpos_start)

    print(f"\n=== Arm joint qpos indices: {arm_indices} ===")
    if arm_indices:
        start = arm_indices[0]
        end = arm_indices[-1] + 1
        print(f"  Arm slice: qpos[{start}:{end}]")
        if end - start == len(ARM_JOINT_NAMES):
            print(f"  Contiguous! Use joints[{start}:{end}]")
        else:
            print(f"  WARNING: Non-contiguous arm joints!")

    return arm_indices


def main():
    print(f"Source URDF: {URDF_SOURCE}")
    print(f"Assets dir:  {ASSETS_DIR}")
    print(f"Output:      {OUTPUT_PATH}")

    # Step 1: Decimate oversized meshes
    print("\n--- Step 1: Decimating oversized meshes ---")
    decimate_oversized_meshes()

    # Step 2: Compile URDF via MuJoCo
    print("\n--- Step 2: Compiling URDF via MuJoCo ---")
    mjcf_text = load_and_compile_urdf()

    # Step 3: Post-process
    print("\n--- Step 3: Post-processing MJCF ---")
    final_xml = post_process(mjcf_text)

    # Step 3: Write output
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    with open(OUTPUT_PATH, "w") as f:
        f.write(final_xml)
    print(f"\nWritten: {OUTPUT_PATH}")

    # Step 4: Print qpos mapping
    print("\n--- Step 3: Verifying final model ---")
    arm_indices = print_qpos_mapping(OUTPUT_PATH)

    return arm_indices


if __name__ == "__main__":
    main()
