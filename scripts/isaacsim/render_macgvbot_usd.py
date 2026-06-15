#!/usr/bin/env python3
"""Render or simulate the repository's macgvbot.usd scene from Isaac Sim.

Run this file with Isaac Sim's Python interpreter, not the system Python.
It opens a USD stage containing the MacGyvBot robot and drawer, creates a
capture camera, drives the robot joint targets through a drawer/tool motion,
saves RGB frames, and optionally converts them to an mp4.

[물리 법칙 반영 및 쉘 스크립트 호환 패치 완료]
시스템 파이썬 파싱 에러를 방지하기 위해 pxr 임포트를 함수 내부로 격리했습니다.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_USD = REPO_ROOT / "macgvbot.usd"
DEFAULT_OUTPUT = REPO_ROOT / "outputs" / "isaacsim" / "macgvbot_usd_capture"
DEFAULT_ROBOT_DESCRIPTION = Path("/home/ssu/Downloads/m0609_description.yaml")
DEFAULT_ROBOT_URDF = Path(
    "/home/ssu/ros2_ws/src/doosan-robot2/dsr_description2/urdf/m0609_isaac_sim.urdf"
)

ROBOT_JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]

HOME_JOINT_DEGREES = [0.0, 0.0, 90.0, 0.0, 90.0, 90.0]
OBSERVATION_JOINT_DEGREES = [0.0, -40.0, 55.0, 0.0, 120.0, 90.0]

# 서랍 층별(0: 1층 pliers, 1: 2층 screwdriver, 2: 3층 wrench) 경계선 가변 상수 맵
JOINT_BOUNDARIES = {
    "HANDLE": {
        0: [24.86, 20.51, 94.78, -17.16, 40.44, 126.99],  # 1층 (Pliers)
        1: [24.68, 17.01, 85.48, -12.82, 53.29, 123.09],  # 2층 (Screwdriver)
        2: [24.84, 15.75, 74.14, -12.17, 65.79, 119.95],  # 3층 (Wrench)
    },
    "PULL": {
        0: [24.86, 4.51, 116.78, -17.16, 30.44, 126.99],   # 1층 (Pliers)
        1: [33.80, 8.50, 100.20, -16.20, 54.10, 125.00],  # 2층 (Screwdriver)
        2: [24.84, -0.25, 96.14, -12.17, 55.79, 119.95],   # 3층 (Wrench)
    }
}

TOOL_DRAWER_IDS = {
    "wrench": 2,
    "screwdriver": 1,
    "pliers": 0,
}

DRAWER_PRIM_BY_ID = {
    0: "/drawer/drawer_floor_01",
    1: "/drawer/drawer_floor_02",
    2: "/drawer/drawer_floor_03",
}

DRAWER_OPEN_OFFSET_XYZ_M = [-0.260, 0.0, 0.0]
# The handle is on the drawer's +Y face. Approach from in front of that face,
# independently of the -X direction used to pull the drawer open.
DRAWER_HANDLE_FRONT_DIRECTION_XYZ = [0.0, 1.0, 0.0]
DRAWER_OBSERVE_OFFSET_XYZ_M = [0.10, 0.0, 0.15]
DRAWER_HANDLE_PREAPPROACH_X_OFFSET_M = -0.02
DRAWER_OBSERVATION_J6_DEG = 90.0

DEFAULT_DRAWER_AXIS, DEFAULT_DRAWER_DISTANCE = next(
    (axis, abs(distance))
    for axis, distance in zip(("x", "y", "z"), DRAWER_OPEN_OFFSET_XYZ_M)
    if abs(distance) > 1e-9
)
DEFAULT_JOINT_SIGNS = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
GRIPPER_OPEN_M = 0.036
GRIPPER_CLOSED_M = 0.004
DRAWER_HANDLE_PREAPPROACH_JOINT_BLEND = 0.82
BASE_DRAWER_TRANSLATES: dict[str, tuple[float, float, float]] = {}

# link_6 origin -> center between the inner finger pads, expressed in link_6.
GRIPPER_TCP_OFFSET_LINK6_M = (0.0, -0.00570734, 0.13899417)
MAX_ANCHORED_IK_DELTA_DEGREES = [18.0, 18.0, 25.0, 30.0, 30.0, 35.0]
GRIPPER_CLOSED_DEGREES = 30.0
GRASP_MIN_CLOSURE_RATIO = 0.12
GRASP_MAX_CENTER_ERROR_M = 0.065
GRASP_REQUIRED_READY_FRAMES = 3

# physics 모드: close 시점에 그리퍼-서랍을 강체로 묶는 grasp constraint 경로 및 성공 판정 기준
GRASP_CONSTRAINT_PATH = "/World/MacGyvBotGraspConstraint"
OPEN_SUCCESS_RATIO = 0.5      # pull 후 목표거리의 50% 이상 이동하면 open 성공
CLOSE_SUCCESS_EPS_M = 0.03    # push 후 원위치 ±3cm 이내면 close 성공
GRASP_CREATE_T = 0.42         # robot_pose_for_frame t 스케줄: pull 직전 HANDLE close 시점
GRASP_FALLBACK_T = 0.50       # 이 시점까지 손끝이 손잡이에 닿지 않아도 강제 결합(stall 방지)
GRASP_RELEASE_T = 0.88        # push 후 HANDLE 복귀하여 그리퍼 여는 시점


@dataclass(frozen=True)
class DrawerIkPlan:
    pregrasp: list[float]
    grasp: list[float]
    pull_waypoints: list[list[float]]
    handle_world: tuple[float, float, float]


@dataclass
class GraspGate:
    released: bool = False
    wait_frames: int = 0
    ready_frames: int = 0
    reported_wait: bool = False
    release_frame: int | None = None


@dataclass
class DrawerMotionState:
    pull_gate: GraspGate = field(default_factory=GraspGate)
    close_gate: GraspGate = field(default_factory=GraspGate)


@dataclass
class PhysicsDrawerState:
    drawer_body_path: str
    joint_path: str
    anchor_path: str
    axis_index: int                       # 0=x, 1=y, 2=z (월드 변위 측정 축)
    rest_position: tuple[float, float, float]
    constraint_path: str | None = None
    grasp_created: bool = False
    grasp_released: bool = False
    max_open_delta: float = 0.0           # 닫힌 위치 대비 최대 이동량(m)
    final_delta: float = 0.0              # 마지막 프레임 기준 이동량(m)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render MacGyvBot macgvbot.usd frames/video in Isaac Sim.")
    parser.add_argument("--usd", type=Path, default=DEFAULT_USD, help="USD stage to open.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT, help="Directory for frames.")
    parser.add_argument("--width", type=int, default=1280, help="Render width.")
    parser.add_argument("--height", type=int, default=720, help="Render height.")
    parser.add_argument("--frames", type=int, default=180, help="Number of frames.")
    parser.add_argument("--fps", type=int, default=30, help="Output FPS.")
    parser.add_argument("--list-stage", action="store_true", help="Print likely robot prims.")
    parser.add_argument("--find-prim", default="", help="Find matching prim paths.")
    parser.add_argument("--list-articulation-debug", action="store_true", help="Print physics schemas.")
    parser.add_argument("--dry-inspect", action="store_true", help="Inspect and exit.")
    parser.add_argument("--plan-only", action="store_true", help="Compute and print the grasp plan, then exit.")
    parser.add_argument("--simulate-twin", action=argparse.BooleanOptionalAction, default=True, help="Animate robot joints.")
    parser.add_argument("--drawer-control", choices=("animated", "physics"), default="animated", help="Drawer motion source: 'animated' keeps the legacy path, 'physics' moves the drawer only via a grasp constraint.")
    parser.add_argument("--grasp-anchor", default="/m0609/onrobot_rg2ft/gripper_body", help="Gripper rigid-body link bound to the drawer in physics mode. Must be rigidly fixed to the arm (the palm) — fingertips sit on a passive mimic linkage that absorbs the pull instead of moving the drawer.")
    parser.add_argument("--drawer-open-fraction", type=float, default=0.95, help="physics mode: fraction of the HANDLE->PULL motion to travel (controls how far the drawer opens). Larger = opens wider.")
    parser.add_argument("--gripper-closed-degrees", type=float, default=50.0, help="Gripper finger_joint target angle when closing (range ~0..67.6). Larger = grips tighter (smaller finger gap).")
    # grasp 포즈 미세조정용 관절 오프셋(도). 기본 비활성(0) — 원래 하드코딩 HANDLE 포즈 사용.
    parser.add_argument("--grasp-pose-offset", default="0,0,0,0,0,0", help="physics mode: per-joint degree offset added to the HANDLE/PULL grasp pose. Default disabled; set e.g. '2,0,0,0,0,0' to nudge.")
    parser.add_argument("--tool", default="screwdriver", choices=tuple(sorted(TOOL_DRAWER_IDS)), help="Tool drawer mapping.")
    parser.add_argument("--drawer-id", type=int, choices=(0, 1, 2), default=None, help="Override drawer identity.")
    parser.add_argument("--print-sim-targets", action="store_true", help="Print auto-detected parameters.")
    parser.add_argument("--physics-steps-per-frame", type=int, default=16, help="Physics updates per frame.")
    parser.add_argument("--author-joint-drives", action=argparse.BooleanOptionalAction, default=False, help="Write USD drives.")
    parser.add_argument("--drawer-preapproach-joint-blend", type=float, default=DRAWER_HANDLE_PREAPPROACH_JOINT_BLEND, help="Preapproach blend fraction.")
    parser.add_argument("--auto-ik", action=argparse.BooleanOptionalAction, default=True, help="Correct the calibrated grasp pose while preserving its joint branch.")
    parser.add_argument("--robot-description", type=Path, default=DEFAULT_ROBOT_DESCRIPTION, help="Lula robot descriptor YAML.")
    parser.add_argument("--robot-urdf", type=Path, default=DEFAULT_ROBOT_URDF, help="M0609 URDF used by Lula.")
    parser.add_argument("--pregrasp-distance", type=float, default=0.06, help="Distance in front of the handle in meters.")
    parser.add_argument("--pull-waypoints", type=int, default=14, help="Number of samples along the calibrated drawer pull.")
    parser.add_argument("--grasp-center-tolerance", type=float, default=GRASP_MAX_CENTER_ERROR_M, help="Maximum handle-to-finger-midpoint error before pulling.")
    parser.add_argument("--show-waypoint-marker", action=argparse.BooleanOptionalAction, default=False, help="Show marker at target.")
    parser.add_argument("--joint-signs", default=",".join(str(int(value)) for value in DEFAULT_JOINT_SIGNS), help="Joint signs.")
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True, help="Run without UI.")
    parser.add_argument("--renderer", default="RaytracedLighting", choices=("RaytracedLighting", "RealTimePathTracing", "PathTracing", "MinimalRendering"), help="Renderer mode.")
    parser.add_argument("--camera-mode", default="fixed", choices=("orbit", "fixed"), help="Camera movement style.")
    parser.add_argument("--camera-target", type=float, nargs=3, default=(0.02, 0.12, 0.35), metavar=("X", "Y", "Z"), help="Camera lookat center.")
    parser.add_argument("--camera-radius", type=float, default=3.2, help="Orbit distance.")
    parser.add_argument("--camera-height", type=float, default=1.8, help="Camera altitude.")
    parser.add_argument("--camera-start-deg", type=float, default=-35.0, help="Start angle.")
    parser.add_argument("--camera-end-deg", type=float, default=55.0, help="End angle.")
    parser.add_argument("--fixed-camera-position", type=float, nargs=3, default=(-1.20, -1.35, 0.90), metavar=("X", "Y", "Z"), help="Fixed camera coords.")
    parser.add_argument("--drawer-prim", default="", help="Target drawer prim path.")
    parser.add_argument("--articulation-root", default="/m0609/base_link", help="Robot physics articulation root.")
    parser.add_argument("--drawer-axis", default=DEFAULT_DRAWER_AXIS, choices=("x", "y", "z"), help="Drawer travel axis.")
    parser.add_argument("--drawer-distance", type=float, default=DEFAULT_DRAWER_DISTANCE, help="Drawer stroke length.")
    parser.add_argument("--drawer-mass", type=float, default=4.0, help="Drawer mass in kg.")
    parser.add_argument("--drawer-damping", type=float, default=5.0, help="Passive drawer joint damping.")
    parser.add_argument("--grip-friction", type=float, default=1.2, help="Finger/handle friction coefficient.")
    parser.add_argument("--rt-subframes", type=int, default=8, help="Raytracing subframes.")
    parser.add_argument("--video", action=argparse.BooleanOptionalAction, default=True, help="Compile video.")
    parser.add_argument("--video-name", default="macgvbot_usd_capture.mp4", help="Video file name.")
    parser.add_argument("--keep-frames", action=argparse.BooleanOptionalAction, default=True, help="Keep RGB frames.")
    return parser.parse_args()


def parse_joint_signs(value: str) -> list[float]:
    try:
        signs = [float(item.strip()) for item in str(value).split(",")]
    except ValueError as exc:
        raise ValueError("--joint-signs must contain comma-separated numbers") from exc
    if len(signs) != len(ROBOT_JOINT_NAMES):
        raise ValueError(f"--joint-signs must have {len(ROBOT_JOINT_NAMES)} values")
    return [1.0 if sign >= 0.0 else -1.0 for sign in signs]


def apply_joint_signs(joint_degrees: list[float], signs: list[float]) -> list[float]:
    return [float(value) * float(sign) for value, sign in zip(joint_degrees, signs)]


def parse_pose_offset(value: str) -> list[float]:
    try:
        offset = [float(item.strip()) for item in str(value).split(",")]
    except ValueError as exc:
        raise ValueError("--grasp-pose-offset must contain comma-separated numbers") from exc
    if len(offset) != len(ROBOT_JOINT_NAMES):
        raise ValueError(f"--grasp-pose-offset must have {len(ROBOT_JOINT_NAMES)} values")
    return offset


def require_existing_file(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"USD stage not found: {resolved}")
    return resolved


def clean_output_dir(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    resolved.mkdir(parents=True, exist_ok=True)
    for old_frame in resolved.rglob("rgb*.png"):
        old_frame.unlink()
    return resolved


def wait_for_stage_load(simulation_app, max_updates: int = 240) -> None:
    for _ in range(max_updates):
        simulation_app.update()


def list_stage_prims() -> None:
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    interesting_words = ("m0609", "joint", "drawer", "handle", "gripper", "finger")
    print("Stage prims likely relevant to MacGyvBot simulation:")
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if any(word in path.lower() for word in interesting_words):
            print(f"  {path}  type={prim.GetTypeName()}")


def find_stage_prims(pattern: str) -> None:
    import omni.usd
    needle = str(pattern or "").strip().lower()
    if not needle: return
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if needle in path.lower():
            print(f"  {path}  type={prim.GetTypeName()}")


def list_articulation_debug() -> None:
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not path.startswith("/m0609"): continue
        applied = [str(s) for s in prim.GetAppliedSchemas()]
        if "PhysicsArticulationRootAPI" in applied or "PhysicsRigidBodyAPI" in applied:
            print(f"  {path}  schemas={','.join(applied)}")


def set_camera_pose(camera_path: str, position: tuple[float, float, float], target) -> None:
    from pxr import Gf, UsdGeom
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(camera_path)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*position), Gf.Vec3d(*target), Gf.Vec3d(0.0, 0.0, 1.0))
    xform.AddTransformOp().Set(view.GetInverse())


def ensure_camera(camera_path: str, focal_length: float = 28.0) -> str:
    from pxr import Sdf, UsdGeom
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    camera = UsdGeom.Camera.Define(stage, Sdf.Path(camera_path))
    camera.CreateFocalLengthAttr(focal_length)
    return camera_path


def ensure_lights() -> None:
    from pxr import Sdf, UsdLux
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/MacGyvBotCaptureDomeLight"))
    dome.CreateIntensityAttr(650.0)


def ensure_physics_scene() -> None:
    from pxr import Sdf, UsdPhysics
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        if UsdPhysics.Scene(prim): return
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
    scene.CreateGravityMagnitudeAttr(9.81)


def resolve_drawer_prim(drawer_prim: str, drawer_id: int) -> str:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    requested = str(drawer_prim or "").strip()
    if requested:
        prim = stage.GetPrimAtPath(requested)
        if not prim or not prim.IsValid():
            raise ValueError(f"Drawer prim does not exist: {requested}")
        return requested
    return DRAWER_PRIM_BY_ID[drawer_id]


def configure_drawer_physics(
    drawer_prim_path: str,
    axis: str,
    distance_m: float,
    mass_kg: float,
    damping: float,
) -> str:
    from pxr import UsdGeom, UsdPhysics
    import omni.usd

    if distance_m <= 0.0:
        raise ValueError("--drawer-distance must be positive; the USD joint axis already defines the opening direction")
    if mass_kg <= 0.0:
        raise ValueError("--drawer-mass must be positive")

    stage = omni.usd.get_context().get_stage()
    drawer_prim = stage.GetPrimAtPath(drawer_prim_path)
    if not drawer_prim or not drawer_prim.IsValid():
        raise ValueError(f"Drawer prim does not exist: {drawer_prim_path}")
    if not drawer_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError(f"Drawer prim is not a rigid body: {drawer_prim_path}")

    joint_path = f"{drawer_prim_path}/PrismaticJoint"
    joint_prim = stage.GetPrimAtPath(joint_path)
    joint = UsdPhysics.PrismaticJoint(joint_prim)
    if not joint:
        raise ValueError(f"Prismatic joint does not exist: {joint_path}")

    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    travel_stage_units = float(distance_m) / float(meters_per_unit)
    joint.CreateAxisAttr().Set(axis.upper())
    joint.CreateLowerLimitAttr().Set(0.0)
    joint.CreateUpperLimitAttr().Set(travel_stage_units)

    mass = UsdPhysics.MassAPI.Apply(drawer_prim)
    mass.CreateMassAttr().Set(float(mass_kg))

    drive = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
    if not drive:
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "linear")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(0.0)
    drive.CreateDampingAttr().Set(max(float(damping), 0.0))
    drive.CreateTargetVelocityAttr().Set(0.0)

    axis = joint.GetAxisAttr().Get()
    print(
        f"[DRAWER] Passive physics enabled: prim={drawer_prim_path}, "
        f"joint={joint_path}, axis={axis}, limits=0..{travel_stage_units:.4f}, mass={mass_kg}"
    )
    return joint_path


def configure_grip_friction(drawer_prim_path: str, friction: float) -> int:
    from pxr import Sdf, Usd, UsdPhysics, UsdShade
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    material = UsdShade.Material.Define(stage, Sdf.Path("/World/MacGyvBotGripPhysicsMaterial"))
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr().Set(max(float(friction), 0.0))
    physics_material.CreateDynamicFrictionAttr().Set(max(float(friction), 0.0))
    physics_material.CreateRestitutionAttr().Set(0.0)

    targets = []
    handle_prim = stage.GetPrimAtPath(f"{drawer_prim_path}/Cube")
    if handle_prim and handle_prim.IsValid() and handle_prim.HasAPI(UsdPhysics.CollisionAPI):
        targets.append(handle_prim)

    gripper_root = stage.GetPrimAtPath("/m0609/onrobot_rg2ft")
    if gripper_root and gripper_root.IsValid():
        for prim in Usd.PrimRange(gripper_root):
            path = prim.GetPath().pathString.lower()
            is_collision_scope = path.endswith("/collisions")
            is_finger_link = any(word in path for word in ("finger", "knuckle"))
            if is_collision_scope and is_finger_link:
                targets.append(prim)

    for prim in targets:
        binding = (
            UsdShade.MaterialBindingAPI(prim)
            if prim.HasAPI(UsdShade.MaterialBindingAPI)
            else UsdShade.MaterialBindingAPI.Apply(prim)
        )
        binding.Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )

    print(f"[GRIP] Physics material applied to {len(targets)} colliders; friction={friction}")
    return len(targets)


# ==============================================================================
# 🧰 [PHYSICS MODE] constraint 기반 물리 서랍 — drawer transform 직접 조작 금지
# ==============================================================================
DRAWER_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def setup_physics_drawer(
    drawer_prim_path: str,
    axis: str,
    distance_m: float,
    mass_kg: float,
    damping: float,
    anchor_path: str,
) -> "PhysicsDrawerState":
    """Make the selected drawer a physics-driven body (idempotent).

    The shipped USD already exposes the drawer as a dynamic rigid body with a
    prismatic joint to the static ``/drawer/Frame``; this verifies that setup,
    re-applies any missing Physics API, and refreshes the joint limits so the
    drawer can only travel along its axis. No transform is ever written.
    """
    from pxr import Sdf, UsdPhysics
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    drawer_prim = stage.GetPrimAtPath(drawer_prim_path)
    if not drawer_prim or not drawer_prim.IsValid():
        raise ValueError(f"Drawer prim does not exist: {drawer_prim_path}")

    # 1) drawer body = dynamic rigid body
    if not drawer_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(drawer_prim)
        print(f"[PHYSICS] Applied RigidBodyAPI to {drawer_prim_path}")

    # 2) drawer box + handle cube must collide
    collider_count = 0
    for child in drawer_prim.GetChildren():
        if child.GetTypeName() in ("Mesh", "Cube"):
            if not child.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(child)
            collider_count += 1

    # 3) prismatic joint to the (static) cabinet frame — reuse or create
    joint_path = f"{drawer_prim_path}/PrismaticJoint"
    joint_prim = stage.GetPrimAtPath(joint_path)
    if not joint_prim or not joint_prim.IsValid():
        joint = UsdPhysics.PrismaticJoint.Define(stage, Sdf.Path(joint_path))
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/drawer/Frame")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(drawer_prim_path)])
        print(f"[PHYSICS] Created prismatic joint: {joint_path}")

    # 4) axis / limits / mass / passive damping (reuses existing helper)
    joint_path = configure_drawer_physics(
        drawer_prim_path, axis, distance_m, mass_kg, damping
    )

    rest_position, _ = world_pose(drawer_prim_path)
    axis_index = DRAWER_AXIS_INDEX[str(axis).lower()]
    state = PhysicsDrawerState(
        drawer_body_path=drawer_prim_path,
        joint_path=joint_path,
        anchor_path=anchor_path,
        axis_index=axis_index,
        rest_position=rest_position,
    )
    print(
        f"[PHYSICS] drawer setup done: prim={drawer_prim_path}, joint={joint_path}, "
        f"anchor={anchor_path}, colliders={collider_count}, "
        f"rest={[round(v, 4) for v in rest_position]}, axis={axis}"
    )
    return state


def drawer_axis_delta(state: "PhysicsDrawerState") -> float:
    """Signed displacement of the drawer body along its travel axis since rest."""
    position, _ = world_pose(state.drawer_body_path)
    return float(position[state.axis_index] - state.rest_position[state.axis_index])


def lock_idle_drawers(selected_drawer_path: str) -> list[str]:
    """Make every non-target drawer kinematic so the arm can't knock it open.

    Only the selected drawer should move; the others stay dynamic in the USD and
    would slide open if the gripper grazes them (e.g. while returning home). A
    kinematic rigid body keeps its collider but is immovable by contacts.
    """
    from pxr import UsdPhysics
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    locked: list[str] = []
    for path in DRAWER_PRIM_BY_ID.values():
        if path == selected_drawer_path:
            continue
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        rb = (
            UsdPhysics.RigidBodyAPI(prim)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI)
            else UsdPhysics.RigidBodyAPI.Apply(prim)
        )
        rb.CreateKinematicEnabledAttr().Set(True)
        locked.append(path)
    print(f"[PHYSICS] Locked idle drawers (kinematic): {locked}")
    return locked


def configure_gripper_drive(
    gripper_paths: list[str],
    stiffness: float = 2000.0,
    damping: float = 150.0,
    max_force: float = 1000.0,
) -> bool:
    """Bake the gripper's angular drive gains BEFORE timeline.play().

    The shipped finger_joint drive has damping=0, so a position target never
    settles. Editing drive gains after the articulation is initialized does not
    reliably reach the live PhysX articulation, so we author non-zero damping
    here, at setup time, before play. The per-frame target is still pushed via
    dynamic_control in set_gripper_force_closure.
    """
    from pxr import UsdPhysics
    import omni.usd

    if not gripper_paths:
        print("[GRIPPER] No driven joint to configure before play.")
        return False
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(gripper_paths[0])
    if not prim or not prim.IsValid():
        return False
    drive = UsdPhysics.DriveAPI.Get(prim, "angular") or UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateStiffnessAttr().Set(float(stiffness))
    drive.CreateDampingAttr().Set(float(damping))
    drive.CreateMaxForceAttr().Set(float(max_force))
    print(
        f"[GRIPPER] Pre-play drive baked: {gripper_paths[0]} "
        f"stiffness={stiffness}, damping={damping}, maxForce={max_force}"
    )
    return True


def create_grasp_constraint(
    gripper_link_path: str,
    drawer_body_path: str,
    constraint_path: str = GRASP_CONSTRAINT_PATH,
) -> str:
    """Lock the drawer body to the gripper link at their current relative pose.

    A FixedJoint authored with local frames derived from the live world poses
    keeps the current relative transform, so enabling it does not snap the
    drawer. The drawer then follows the arm purely through physics. If the
    prismatic+fixed closed loop ever fights the solver, swap this for a
    D6 joint that frees only the travel axis.
    """
    from pxr import Gf, Sdf, UsdPhysics
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    existing = stage.GetPrimAtPath(constraint_path)
    if existing and existing.IsValid():
        stage.RemovePrim(constraint_path)

    anchor_pos, anchor_quat = world_pose(gripper_link_path)
    drawer_pos, drawer_quat = world_pose(drawer_body_path)

    # body0(gripper)와 body1(drawer)의 월드 변환을 행렬로 만든 뒤, body1 프레임을
    # body0 로컬로 표현한 단일 행렬(rel)에서 위치·회전을 함께 추출한다. 이렇게 하면
    # 회전 합성 순서 모호성이 사라져 "disjointed body transforms" 스냅이 발생하지 않는다.
    def _world_matrix(pos, quat):
        rot = Gf.Rotation(Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3])))
        return Gf.Matrix4d().SetRotate(rot).SetTranslateOnly(Gf.Vec3d(*pos))

    t0 = _world_matrix(anchor_pos, anchor_quat)
    t1 = _world_matrix(drawer_pos, drawer_quat)
    rel = t1 * t0.GetInverse()  # rel * t0 == t1  → body1 frame expressed in body0 local
    rel_translation = rel.ExtractTranslation()
    rel_rotation = rel.ExtractRotationQuat()

    joint = UsdPhysics.FixedJoint.Define(stage, Sdf.Path(constraint_path))
    joint.CreateBody0Rel().SetTargets([Sdf.Path(gripper_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(drawer_body_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rel_translation))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(rel_rotation))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateBreakForceAttr().Set(1e10)
    joint.CreateBreakTorqueAttr().Set(1e10)
    joint.CreateJointEnabledAttr().Set(True)
    return constraint_path


def remove_grasp_constraint(constraint_path: str) -> None:
    from pxr import UsdPhysics
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(constraint_path)
    if not prim or not prim.IsValid():
        return
    joint = UsdPhysics.FixedJoint(prim)
    if joint:
        joint.CreateJointEnabledAttr().Set(False)
    stage.RemovePrim(constraint_path)


def should_create_grasp(frame_index: int, frames: int) -> bool:
    if frames <= 1:
        return False
    return frame_index >= int(round(GRASP_CREATE_T * (frames - 1)))


def should_release_grasp(frame_index: int, frames: int) -> bool:
    if frames <= 1:
        return False
    return frame_index >= int(round(GRASP_RELEASE_T * (frames - 1)))


def detect_robot_joint_prims() -> dict[str, str]:
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    joint_paths: dict[str, str] = {}
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in ROBOT_JOINT_NAMES:
            joint_paths[name] = prim.GetPath().pathString
    return joint_paths


def detect_gripper_prims() -> list[str]:
    from pxr import UsdPhysics
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    paths: list[str] = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if "onrobot_rg2ft/joints" not in path.lower():
            continue
        if UsdPhysics.DriveAPI.Get(prim, "angular"):
            paths.append(path)
    return paths


def set_robot_joint_targets(joint_paths: dict[str, str], joint_degrees: list[float]) -> None:
    from pxr import UsdPhysics
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    for name, deg in zip(ROBOT_JOINT_NAMES, joint_degrees):
        path = joint_paths.get(name)
        if not path: continue
        prim = stage.GetPrimAtPath(path)
        drive = UsdPhysics.DriveAPI.Get(prim, "angular") or UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTargetPositionAttr().Set(float(deg))
        drive.CreateStiffnessAttr().Set(500000.0)
        drive.CreateDampingAttr().Set(5000.0)


def ensure_robot_articulation_root(joint_paths: dict[str, str], preferred: str) -> str:
    from pxr import UsdPhysics
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(preferred)
    if prim and prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return preferred
    for candidate in stage.Traverse():
        if candidate.HasAPI(UsdPhysics.ArticulationRootAPI):
            return candidate.GetPath().pathString
    return "/m0609/root_joint"


def dynamic_control_candidate_paths(root: str) -> list[str]:
    return list(dict.fromkeys(("/m0609/base_link", root, "/m0609/root_joint", "/m0609")))


def set_dynamic_control_targets(joint_degrees: list[float], root_path: str) -> bool:
    try:
        from omni.isaac.dynamic_control import _dynamic_control
        dc = _dynamic_control.acquire_dynamic_control_interface()
        articulation = 0
        for p in dynamic_control_candidate_paths(root_path):
            articulation = dc.get_articulation(p)
            if articulation != 0: break
        if articulation == 0: return False
        for name, deg in zip(ROBOT_JOINT_NAMES, joint_degrees):
            dof = dc.find_articulation_dof(articulation, name)
            if dof: dc.set_dof_position_target(dof, math.radians(float(deg)))
        return True
    except Exception:
        return False


# ==============================================================================
# 🦾 [MODIFIED] 물리 기반 악력 제어 (손잡이가 잡힐 때까지 오므리기)
# ==============================================================================
def gripper_target_degrees(joint_prim, close_action: bool, closed_degrees: float = GRIPPER_CLOSED_DEGREES) -> float:
    if not close_action:
        return 0.0
    return float(closed_degrees)


def set_gripper_force_closure(
    gripper_paths: list[str],
    close_action: bool,
    root_path: str = "",
    closed_degrees: float = GRIPPER_CLOSED_DEGREES,
) -> bool:
    from pxr import UsdPhysics
    import omni.usd

    if not gripper_paths:
        print("[GRIPPER] No driven gripper joint was detected.")
        return False

    stage = omni.usd.get_context().get_stage()
    path = gripper_paths[0]
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        print(f"[GRIPPER] Joint prim does not exist: {path}")
        return False

    target_degrees = gripper_target_degrees(prim, close_action, closed_degrees)
    target_radians = math.radians(target_degrees)
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if not drive:
        print(f"[GRIPPER] Angular drive is missing: {path}")
        return False

    drive.CreateStiffnessAttr().Set(5000.0)
    drive.CreateDampingAttr().Set(100.0)
    drive.CreateMaxForceAttr().Set(200.0)
    drive.CreateTargetPositionAttr().Set(target_degrees)

    try:
        from omni.isaac.dynamic_control import _dynamic_control

        dc = _dynamic_control.acquire_dynamic_control_interface()
        articulation = 0
        for candidate in dynamic_control_candidate_paths(root_path):
            articulation = dc.get_articulation(candidate)
            if articulation != 0:
                break
        if articulation != 0:
            dof_name = prim.GetName()
            dof = dc.find_articulation_dof(articulation, dof_name)
            if dof:
                dc.set_dof_position_target(dof, target_radians)
                return True
    except Exception as exc:
        print(f"[GRIPPER] Dynamic Control fallback: {exc}")

    return True


def gripper_closure_ratio(
    gripper_paths: list[str],
    root_path: str,
    closed_degrees: float = GRIPPER_CLOSED_DEGREES,
) -> float | None:
    from pxr import UsdPhysics
    import omni.usd

    if not gripper_paths:
        return None
    prim = omni.usd.get_context().get_stage().GetPrimAtPath(gripper_paths[0])
    if not prim or not prim.IsValid():
        return None
    target_radians = abs(math.radians(gripper_target_degrees(prim, True, closed_degrees)))
    if target_radians <= 1e-6:
        return None

    try:
        from omni.isaac.dynamic_control import _dynamic_control

        dc = _dynamic_control.acquire_dynamic_control_interface()
        for candidate in dynamic_control_candidate_paths(root_path):
            articulation = dc.get_articulation(candidate)
            if articulation == 0:
                continue
            dof = dc.find_articulation_dof(articulation, prim.GetName())
            if dof:
                state = dc.get_dof_state(dof, _dynamic_control.STATE_POS)
                return min(abs(float(state.pos)) / target_radians, 1.0)
    except Exception:
        return None
    return None


def grasp_center_error(drawer_prim_path: str) -> float | None:
    finger_paths = (
        "/m0609/onrobot_rg2ft/left_inner_finger",
        "/m0609/onrobot_rg2ft/right_inner_finger",
    )
    try:
        left, _ = world_pose(finger_paths[0])
        right, _ = world_pose(finger_paths[1])
        handle, _ = world_pose(f"{drawer_prim_path}/Cube")
    except ValueError:
        return None

    midpoint = tuple((a + b) * 0.5 for a, b in zip(left, right))
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(midpoint, handle)))


def ease_in_out(amount: float) -> float:
    amount = min(max(float(amount), 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * amount)


def interpolate_pose(start: list[float], end: list[float], amount: float) -> list[float]:
    t = ease_in_out(amount)
    return [float(a) + (float(b) - float(a)) * t for a, b in zip(start, end)]


def world_pose(prim_path: str):
    from pxr import UsdGeom
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise ValueError(f"Prim does not exist: {prim_path}")
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
    translation = matrix.ExtractTranslation()
    rotation = matrix.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    return (
        (float(translation[0]), float(translation[1]), float(translation[2])),
        (float(rotation.GetReal()), float(imaginary[0]), float(imaginary[1]), float(imaginary[2])),
    )


def build_drawer_ik_plan(
    args: argparse.Namespace,
    drawer_prim_path: str,
    drawer_id: int,
) -> DrawerIkPlan:
    import numpy as np
    from isaacsim.core.utils.numpy.rotations import rot_matrices_to_quats
    from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver

    description = require_existing_file(args.robot_description)
    urdf = require_existing_file(args.robot_urdf)
    solver = LulaKinematicsSolver(str(description), str(urdf))

    base_position, base_orientation = world_pose("/m0609/base_link")
    solver.set_robot_base_pose(np.asarray(base_position), np.asarray(base_orientation))

    handle_position, _ = world_pose(f"{drawer_prim_path}/Cube")
    handle = np.asarray(handle_position, dtype=np.float64)

    handle_seed_degrees = np.asarray(JOINT_BOUNDARIES["HANDLE"][drawer_id], dtype=np.float64)
    orientation_seed = np.radians(handle_seed_degrees)
    _, link6_rotation = solver.compute_forward_kinematics("link_6", orientation_seed)
    link6_quaternion = rot_matrices_to_quats(link6_rotation)
    tcp_offset_world = link6_rotation @ np.asarray(GRIPPER_TCP_OFFSET_LINK6_M, dtype=np.float64)

    pull_vector = np.asarray(DRAWER_OPEN_OFFSET_XYZ_M, dtype=np.float64)
    pull_length = np.linalg.norm(pull_vector)
    if pull_length <= 1e-9:
        raise ValueError("DRAWER_OPEN_OFFSET_XYZ_M must define a non-zero pull direction")
    pull_direction = pull_vector / pull_length

    approach_vector = np.asarray(DRAWER_HANDLE_FRONT_DIRECTION_XYZ, dtype=np.float64)
    approach_length = np.linalg.norm(approach_vector)
    if approach_length <= 1e-9:
        raise ValueError("DRAWER_HANDLE_FRONT_DIRECTION_XYZ must define a non-zero direction")
    if args.pregrasp_distance <= 0.0:
        raise ValueError("--pregrasp-distance must be positive")
    approach_direction = approach_vector / approach_length

    def solve_tcp(tcp_world, warm_start):
        link6_target = np.asarray(tcp_world, dtype=np.float64) - tcp_offset_world
        joints, success = solver.compute_inverse_kinematics(
            "link_6",
            link6_target,
            link6_quaternion,
            warm_start=np.asarray(warm_start, dtype=np.float64),
            position_tolerance=0.002,
            orientation_tolerance=0.03,
        )
        if not success:
            raise RuntimeError(f"Lula IK failed for TCP target {np.asarray(tcp_world).tolist()}")
        return np.degrees(joints).tolist()

    pregrasp_tcp = handle + approach_direction * float(args.pregrasp_distance)
    pregrasp = solve_tcp(pregrasp_tcp, orientation_seed)
    grasp = solve_tcp(handle, np.radians(pregrasp))

    grasp_delta = np.asarray(grasp) - handle_seed_degrees
    max_delta = np.asarray(MAX_ANCHORED_IK_DELTA_DEGREES)
    if np.any(np.abs(grasp_delta) > max_delta):
        raise RuntimeError(
            "anchored IK left the calibrated joint branch: "
            f"delta={[round(value, 2) for value in grasp_delta.tolist()]}"
        )

    pull_waypoints = [grasp]
    warm_start = np.radians(grasp)
    waypoint_count = max(int(args.pull_waypoints), 2)
    for index in range(1, waypoint_count):
        amount = index / (waypoint_count - 1)
        tcp_target = handle + pull_direction * float(args.drawer_distance) * amount
        joints = solve_tcp(tcp_target, warm_start)
        step_delta = np.asarray(joints) - np.degrees(warm_start)
        if np.any(np.abs(step_delta) > 35.0):
            raise RuntimeError(
                "Cartesian pull changed IK branch: "
                f"step_delta={[round(value, 2) for value in step_delta.tolist()]}"
            )
        pull_waypoints.append(joints)
        warm_start = np.radians(joints)

    print(f"[IK] Handle world position: {handle.tolist()}")
    print(f"[IK] Pregrasp TCP position: {pregrasp_tcp.tolist()}")
    print(f"[IK] Approach direction: {approach_direction.tolist()}")
    print(f"[IK] Pull direction: {pull_direction.tolist()}")
    print(f"[IK] Calibrated handle seed: {handle_seed_degrees.tolist()}")
    print(f"[IK] Grasp correction: {[round(value, 2) for value in grasp_delta.tolist()]}")
    print(f"[IK] Pregrasp joints: {[round(value, 2) for value in pregrasp]}")
    print(f"[IK] Grasp joints: {[round(value, 2) for value in grasp]}")
    print(f"[IK] Pull end joints: {[round(value, 2) for value in pull_waypoints[-1]]}")
    return DrawerIkPlan(
        pregrasp=pregrasp,
        grasp=grasp,
        pull_waypoints=pull_waypoints,
        handle_world=tuple(float(value) for value in handle),
    )


def sample_joint_waypoints(waypoints: list[list[float]], amount: float) -> list[float]:
    if len(waypoints) == 1:
        return list(waypoints[0])
    scaled = min(max(float(amount), 0.0), 1.0) * (len(waypoints) - 1)
    index = min(int(scaled), len(waypoints) - 2)
    return interpolate_pose(waypoints[index], waypoints[index + 1], scaled - index)


def robot_pose_for_frame(
    frame_index: int,
    frames: int,
    drawer_id: int,
    blend: float,
    ik_plan: DrawerIkPlan | None = None,
    pull_scale: float = 1.0,
    skip_observation: bool = False,
    pose_offset: list[float] | None = None,
) -> tuple[list[float], float, bool]:
    if frames <= 1: return HOME_JOINT_DEGREES, 0.0, False
    t = frame_index / (frames - 1)

    p_start = ik_plan.grasp if ik_plan else JOINT_BOUNDARIES["HANDLE"][drawer_id]
    p_end = ik_plan.pull_waypoints[-1] if ik_plan else JOINT_BOUNDARIES["PULL"][drawer_id]
    # physics 모드: 서랍 과도 인출 방지를 위해 HANDLE->PULL 이동량을 비율로 축소(선형)
    if pull_scale != 1.0:
        scale = min(max(float(pull_scale), 0.0), 1.0)
        p_end = [a + (b - a) * scale for a, b in zip(p_start, p_end)]
    # grasp pose nudge: 손끝을 손잡이에 더 맞추기 위한 관절 오프셋(grasp/pull 동일 적용)
    if pose_offset:
        p_start = [a + o for a, o in zip(p_start, pose_offset)]
        p_end = [a + o for a, o in zip(p_end, pose_offset)]
    pre_pose = (
        ik_plan.pregrasp
        if ik_plan
        else interpolate_pose(OBSERVATION_JOINT_DEGREES, p_start, blend)
    )

    # skip_observation: 손잡이로 가기 전 관찰(탐지/인스펙션) 포즈 경유를 생략하고
    # HOME에서 곧바로 pre_pose로 접근한다(0.00~0.27 구간을 직접 보간).
    if skip_observation:
        if t < 0.27:
            return interpolate_pose(HOME_JOINT_DEGREES, pre_pose, t / 0.27), 0.0, False
        if t < 0.34:
            return interpolate_pose(pre_pose, p_start, (t - 0.27) / 0.07), 0.0, False
        # 0.34 이후는 아래 공통 스케줄(grasp/pull/push)로 진행
    elif t < 0.10:
        return interpolate_pose(HOME_JOINT_DEGREES, OBSERVATION_JOINT_DEGREES, t / 0.10), 0.0, False
    elif t < 0.20:
        return OBSERVATION_JOINT_DEGREES, 0.0, False

    if t < 0.27:
        return interpolate_pose(OBSERVATION_JOINT_DEGREES, pre_pose, (t - 0.20) / 0.07), 0.0, False
    if t < 0.34:
        return interpolate_pose(pre_pose, p_start, (t - 0.27) / 0.07), 0.0, False
    if t < 0.42:
        return p_start, 0.0, True
    if t < 0.56:
        amount = (t - 0.42) / 0.14
        pose = (
            sample_joint_waypoints(ik_plan.pull_waypoints, amount)
            if ik_plan
            else interpolate_pose(p_start, p_end, amount)
        )
        return pose, ease_in_out(amount), True
    if t < 0.62:
        return p_end, 1.0, True
    if t < 0.66:
        return p_end, 1.0, False
    if t < 0.70:
        return p_end, 1.0, True
    if t < 0.84:
        amount = (t - 0.70) / 0.14
        pose = (
            sample_joint_waypoints(list(reversed(ik_plan.pull_waypoints)), amount)
            if ik_plan
            else interpolate_pose(p_end, p_start, amount)
        )
        return pose, 1.0 - ease_in_out(amount), True
    if t < 0.88:
        return p_start, 0.0, True
    if t < 0.91:
        return p_start, 0.0, False
    if t < 0.96:
        return interpolate_pose(p_start, pre_pose, (t - 0.91) / 0.05), 0.0, False
    return interpolate_pose(pre_pose, HOME_JOINT_DEGREES, (t - 0.96) / 0.04), 0.0, False


def update_grasp_gate(
    gate: GraspGate,
    label: str,
    frame_index: int,
    gripper_paths: list[str],
    root_path: str,
    drawer_prim_path: str,
    center_tolerance: float,
) -> bool:
    closure = gripper_closure_ratio(gripper_paths, root_path)
    center_error = grasp_center_error(drawer_prim_path)
    closure_ready = closure is not None and closure >= GRASP_MIN_CLOSURE_RATIO
    centered = center_error is not None and center_error <= center_tolerance
    gate.wait_frames += 1
    gate.ready_frames = gate.ready_frames + 1 if closure_ready and centered else 0

    if gate.ready_frames >= GRASP_REQUIRED_READY_FRAMES:
        gate.released = True
        gate.release_frame = frame_index
        print(
            f"[GRASP] {label} gate released: closure={closure:.2f}, "
            f"center_error={center_error:.4f}m, wait_frames={gate.wait_frames}"
        )
        return True

    if not gate.reported_wait or gate.wait_frames % 30 == 0:
        closure_text = "unknown" if closure is None else f"{closure:.2f}"
        center_text = "unknown" if center_error is None else f"{center_error:.4f}m"
        print(
            f"[GRASP] Holding {label}; closure={closure_text}, "
            f"center_error={center_text}, ready_frames={gate.ready_frames}"
        )
        gate.reported_wait = True
    return False


def apply_physics_drawer_frame(
    args: argparse.Namespace,
    frame_index: int,
    joint_paths: dict[str, str],
    gripper_paths: list[str],
    drawer_id: int,
    signs: list[float],
    root_path: str,
    physics_state: "PhysicsDrawerState",
) -> None:
    """physics 모드: 하드코딩 HANDLE->PULL waypoint + close 시점 constraint.

    grasp gate / auto-IK는 사용하지 않고, drawer는 grasp constraint를 통한
    물리 결과로만 움직인다(transform 직접 조작 없음).
    """
    joint_degrees, _, grasp_trigger = robot_pose_for_frame(
        frame_index,
        args.frames,
        drawer_id,
        args.drawer_preapproach_joint_blend,
        None,  # ik_plan=None → JOINT_BOUNDARIES 하드코딩 사용
        pull_scale=args.drawer_open_fraction,
        skip_observation=True,  # 손잡이로 가기 전 관찰(탐지) 포즈 경유 생략
        pose_offset=parse_pose_offset(args.grasp_pose_offset),
    )
    signed_joints = apply_joint_signs(joint_degrees, signs)

    if args.simulate_twin:
        if not set_dynamic_control_targets(signed_joints, root_path):
            set_robot_joint_targets(joint_paths, signed_joints)
        set_gripper_force_closure(
            gripper_paths, grasp_trigger, root_path, args.gripper_closed_degrees
        )

    # close 시점: 손끝이 실제로 손잡이에 닿았을 때만 결합(접촉 게이트). 단, 정해진
    # fallback 시점까지 닿지 않으면 강제로 결합해 동작이 멈추지 않게 한다(stall 방지).
    if not physics_state.grasp_created and should_create_grasp(frame_index, args.frames):
        center_error = grasp_center_error(physics_state.drawer_body_path)
        in_contact = center_error is not None and center_error <= float(args.grasp_center_tolerance)
        fallback = frame_index >= int(round(GRASP_FALLBACK_T * (args.frames - 1)))
        if in_contact or fallback:
            physics_state.constraint_path = create_grasp_constraint(
                physics_state.anchor_path, physics_state.drawer_body_path
            )
            physics_state.grasp_created = True
            quality = "contact" if in_contact else "FALLBACK(no contact)"
            err_text = "unknown" if center_error is None else f"{center_error:.4f}m"
            print(
                f"[PHYSICS] grasp constraint created @frame {frame_index} [{quality}]: "
                f"anchor={physics_state.anchor_path}, drawer={physics_state.drawer_body_path}, "
                f"constraint={physics_state.constraint_path}, "
                f"finger->handle={err_text}, drawer_disp={drawer_axis_delta(physics_state):.4f}m"
            )

    # open 시점: grasp constraint 제거
    if (
        should_release_grasp(frame_index, args.frames)
        and physics_state.grasp_created
        and not physics_state.grasp_released
    ):
        if physics_state.constraint_path:
            remove_grasp_constraint(physics_state.constraint_path)
        physics_state.grasp_released = True
        print(
            f"[PHYSICS] grasp constraint removed @frame {frame_index}: "
            f"drawer_disp={drawer_axis_delta(physics_state):.4f}m"
        )

    # drawer 변위 추적(요구 #6) — 직접 이동 없이 물리 결과만 측정
    delta = drawer_axis_delta(physics_state)
    physics_state.final_delta = delta
    physics_state.max_open_delta = max(physics_state.max_open_delta, abs(delta))
    if frame_index % 15 == 0:
        # 그리퍼가 실제로 닫히는지 + 손끝이 손잡이에 얼마나 가까운지 진단
        closure = gripper_closure_ratio(gripper_paths, root_path, args.gripper_closed_degrees)
        center_error = grasp_center_error(physics_state.drawer_body_path)
        closure_text = "n/a" if closure is None else f"{closure:.2f}"
        center_text = "n/a" if center_error is None else f"{center_error:.4f}m"
        print(
            f"[PHYSICS] frame {frame_index}: drawer disp={delta:.4f}m, "
            f"grip closure={closure_text}, finger->handle={center_text}"
        )


def apply_digital_twin_frame(
    args: argparse.Namespace,
    frame_index: int,
    joint_paths: dict[str, str],
    gripper_paths: list[str],
    drawer_id: int,
    signs: list[float],
    root_path: str,
    ik_plan: DrawerIkPlan | None,
    motion_state: DrawerMotionState,
    drawer_prim_path: str,
    physics_state: "PhysicsDrawerState | None" = None,
) -> None:
    if args.drawer_control == "physics":
        if physics_state is None:
            raise ValueError("physics drawer control requires an initialized PhysicsDrawerState")
        apply_physics_drawer_frame(
            args,
            frame_index,
            joint_paths,
            gripper_paths,
            drawer_id,
            signs,
            root_path,
            physics_state,
        )
        return

    frame_span = max(args.frames - 1, 1)
    nominal_pull_grasp_frame = int(round(0.34 * frame_span))
    nominal_pull_frame = int(round(0.42 * frame_span))
    nominal_close_grasp_frame = int(round(0.66 * frame_span))
    nominal_close_frame = int(round(0.70 * frame_span))

    pull_delay = 0
    if motion_state.pull_gate.release_frame is not None:
        pull_delay = max(motion_state.pull_gate.release_frame - nominal_pull_frame, 0)

    motion_frame_index = frame_index
    if not motion_state.pull_gate.released and frame_index >= nominal_pull_frame:
        motion_frame_index = nominal_pull_frame
    else:
        motion_frame_index = max(frame_index - pull_delay, 0)

    adjusted_close_frame = nominal_close_frame + pull_delay
    adjusted_close_grasp_frame = nominal_close_grasp_frame + pull_delay
    close_delay = 0
    if motion_state.close_gate.release_frame is not None:
        close_delay = max(motion_state.close_gate.release_frame - adjusted_close_frame, 0)
        motion_frame_index = max(motion_frame_index - close_delay, 0)
    elif motion_state.pull_gate.released and frame_index >= adjusted_close_frame:
        motion_frame_index = nominal_close_frame

    joint_degrees, _, grasp_trigger = robot_pose_for_frame(
        motion_frame_index,
        args.frames,
        drawer_id,
        args.drawer_preapproach_joint_blend,
        ik_plan,
    )

    if (
        ik_plan is not None
        and frame_index >= nominal_pull_grasp_frame
        and not motion_state.pull_gate.released
    ):
        joint_degrees = ik_plan.grasp
        grasp_trigger = True
        update_grasp_gate(
            motion_state.pull_gate,
            "pull grasp",
            frame_index,
            gripper_paths,
            root_path,
            drawer_prim_path,
            float(args.grasp_center_tolerance),
        )
    elif (
        ik_plan is not None
        and motion_state.pull_gate.released
        and frame_index >= adjusted_close_grasp_frame
        and not motion_state.close_gate.released
    ):
        joint_degrees = ik_plan.pull_waypoints[-1]
        grasp_trigger = True
        update_grasp_gate(
            motion_state.close_gate,
            "close grasp",
            frame_index,
            gripper_paths,
            root_path,
            drawer_prim_path,
            float(args.grasp_center_tolerance),
        )

    signed_joints = apply_joint_signs(joint_degrees, signs)

    if args.simulate_twin:
        if not set_dynamic_control_targets(signed_joints, root_path):
            set_robot_joint_targets(joint_paths, signed_joints)

        set_gripper_force_closure(gripper_paths, grasp_trigger, root_path)


def make_video(output_dir: Path, video_path: Path, fps: int) -> None:
    frames = sorted(output_dir.rglob("rgb*.png"))
    if not frames: return
    concat_file = output_dir / "ffmpeg_frames.txt"
    with concat_file.open("w", encoding="utf-8") as h:
        for f in frames:
            h.write(f"file '{f.as_posix()}'\n")
            h.write(f"duration {1.0/fps:.8f}\n")
        h.write(f"file '{frames[-1].as_posix()}'\n")
    subprocess.run(["ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", str(concat_file), "-r", str(fps), "-pix_fmt", "yuv420p", "-vcodec", "libx264", str(video_path)], check=True)


def main() -> int:
    args = parse_args()
    joint_signs = parse_joint_signs(args.joint_signs)
    usd_path = require_existing_file(args.usd)
    drawer_id = args.drawer_id if args.drawer_id is not None else TOOL_DRAWER_IDS[args.tool]
    
    try:
        from isaacsim.simulation_app import SimulationApp
    except ImportError:
        from isaacsim import SimulationApp
        
    simulation_app = SimulationApp({"headless": args.headless, "width": args.width, "height": args.height, "renderer": args.renderer})
    try:
        import carb
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        
        settings = carb.settings.get_settings()
        settings.set("/omni/replicator/captureOnPlay", False)
        settings.set("/app/runLoops/main/rateLimitEnabled", False)
        settings.set("/physics/fixedTimeStep", 1.0 / 60.0)
        settings.set("/physics/useFixedTimeStep", True)
        settings.set("/physics/maxSecondarySteps", 1)
        settings.set("/exts/omni.replicator.core/enableWriteToFabric", True)
        settings.set("/rtx/post/dlss/execMode", 2)
        settings.set("/app/viewport/grid/enabled", False)
        
        context = omni.usd.get_context()
        if not context.open_stage(str(usd_path)): return 1
        wait_for_stage_load(simulation_app)
        
        ensure_lights()
        ensure_physics_scene()
        camera_path = ensure_camera("/World/MacGyvBotCaptureCamera")
        set_camera_pose(camera_path, camera_position_for_frame(args, 0), args.camera_target)

        selected_drawer_prim = resolve_drawer_prim(args.drawer_prim, drawer_id)
        physics_state = None
        if args.drawer_control == "physics":
            physics_state = setup_physics_drawer(
                selected_drawer_prim,
                args.drawer_axis,
                args.drawer_distance,
                args.drawer_mass,
                args.drawer_damping,
                args.grasp_anchor,
            )
            # 선택 안 된 서랍은 kinematic으로 고정(홈 복귀 중 그리퍼가 건드려도 안 열림)
            lock_idle_drawers(selected_drawer_prim)
            # 그리퍼 드라이브 게인을 play 전에 baking (런타임 편집은 반영 안 됨)
            configure_gripper_drive(detect_gripper_prims())
        else:
            configure_drawer_physics(
                selected_drawer_prim,
                args.drawer_axis,
                args.drawer_distance,
                args.drawer_mass,
                args.drawer_damping,
            )
        configure_grip_friction(selected_drawer_prim, args.grip_friction)
        stale_grasp_joint = context.get_stage().GetPrimAtPath("/World/TemporaryGraspJoint")
        if stale_grasp_joint and stale_grasp_joint.IsValid():
            context.get_stage().RemovePrim("/World/TemporaryGraspJoint")
        
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(60): simulation_app.update()
        
        if args.list_stage: list_stage_prims()
        if args.find_prim: find_stage_prims(args.find_prim)
        if args.list_articulation_debug: list_articulation_debug()
        
        if args.dry_inspect or args.list_stage or args.find_prim or args.list_articulation_debug:
            print("\n🔍 [INSPECT COMPLETE] 안전하게 종료합니다.")
            timeline.pause()
            return 0
            
        joint_paths = detect_robot_joint_prims()
        root_path = ensure_robot_articulation_root(joint_paths, args.articulation_root)

        ik_plan = None
        if args.auto_ik and args.drawer_control != "physics":
            try:
                ik_plan = build_drawer_ik_plan(args, selected_drawer_prim, drawer_id)
            except Exception as exc:
                print(f"[IK] Automatic planning failed; using legacy joint targets: {exc}")

        if args.drawer_control == "physics":
            mode = "physics grasp constraint"
        else:
            mode = "anchored grasp correction" if ik_plan else "legacy joint targets"
        print(f"\n[SIM] Drawer grasp mode: {mode}")
        if args.plan_only:
            timeline.pause()
            return 0

        gripper_paths = detect_gripper_prims()
        if not gripper_paths:
            print("[GRIPPER] No angular-drive joint found under onrobot_rg2ft/joints.")
        else:
            print(f"[GRIPPER] Driven joint: {gripper_paths[0]}")
        motion_state = DrawerMotionState()

        output_dir = clean_output_dir(args.output_dir)
        render_prod = rep.create.render_product(camera_path, (args.width, args.height))
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=str(output_dir), rgb=True)
        writer.attach([render_prod])

        # 레프리케이터 오케스트레이터가 정상 동작할 수 있도록 초기화 상태 보정
        rep.orchestrator.set_capture_on_play(False)

        for frame_index in range(args.frames):
            set_camera_pose(camera_path, camera_position_for_frame(args, frame_index), args.camera_target)
            apply_digital_twin_frame(
                args,
                frame_index,
                joint_paths,
                gripper_paths,
                drawer_id,
                joint_signs,
                root_path,
                ik_plan,
                motion_state,
                selected_drawer_prim,
                physics_state,
            )

            # 물리 연산 스텝 쪼개기
            for _ in range(max(args.physics_steps_per_frame, 1)):
                simulation_app.update()

            # 💡 [RENDER PATCH] RenderVarToHost 병목 및 튕김 방지를 위한 동기화 스텝
            rep.orchestrator.step(rt_subframes=args.rt_subframes)

            # 렌더링 버퍼가 완전히 비워질 때까지 앱 엔진 동기화 업데이트
            simulation_app.update()
            
        timeline.pause()
        rep.orchestrator.wait_until_complete()

        if physics_state is not None:
            # 기대 개방량 = pull 비율 * 전체 stroke. 실제로는 손끝 이동량만큼 열린다.
            expected_open = abs(args.drawer_distance) * min(max(args.drawer_open_fraction, 0.0), 1.0)
            open_ok = physics_state.max_open_delta >= OPEN_SUCCESS_RATIO * expected_open
            close_ok = abs(physics_state.final_delta) <= CLOSE_SUCCESS_EPS_M
            print(
                f"[PHYSICS] RESULT open={open_ok} "
                f"(max {physics_state.max_open_delta:.3f} / expected {expected_open:.3f} m), "
                f"close={close_ok} (final {physics_state.final_delta:.3f} m)"
            )

        if args.video: make_video(output_dir, output_dir / args.video_name, args.fps)
        return 0
    finally:
        simulation_app.close()


def camera_position_for_frame(args: argparse.Namespace, frame_index: int) -> tuple[float, float, float]:
    if args.camera_mode == "fixed" or args.frames <= 1: return tuple(args.fixed_camera_position)
    amt = frame_index / (args.frames - 1)
    ang = math.radians(args.camera_start_deg + (args.camera_end_deg - args.camera_start_deg) * amt)
    return (args.camera_target[0] + args.camera_radius * math.cos(ang), args.camera_target[1] + args.camera_radius * math.sin(ang), args.camera_height)


if __name__ == "__main__":
    try: raise SystemExit(main())
    except Exception as e:
        print(f"render_macgvbot_usd.py failed: {e}", file=sys.stderr)
        raise
