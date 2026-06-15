#!/usr/bin/env python3
"""Render or simulate the repository's macgvbot.usd scene from Isaac Sim.

Run this file with Isaac Sim's Python interpreter, not the system Python.
It opens a USD stage containing the MacGyvBot robot and drawer, creates a
capture camera, drives the robot joint targets through a drawer/tool motion,
saves RGB frames, and optionally converts them to an mp4.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_USD = REPO_ROOT / "macgvbot.usd"
DEFAULT_OUTPUT = REPO_ROOT / "outputs" / "isaacsim" / "macgvbot_usd_capture"

ROBOT_JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]

# ==============================================================================
# 🎯 [NATIVE CONFIG ZONE] 1, 2, 3층 전 층 복원 및 ID별 동적 매핑 바운더리
# ==============================================================================
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
        1: [33.80, 8.50, 100.20, -16.20, 54.10, 125.00],  # 2층 (Screwdriver - 실로봇 반영값)
        2: [24.84, -0.25, 96.14, -12.17, 55.79, 119.95],   # 3층 (Wrench)
    }
}

TOOL_DRAWER_IDS = {
    "wrench": 2,
    "screwdriver": 1,
    "pliers": 0,
}

DRAWER_OPEN_OFFSET_XYZ_M = [-0.260, 0.0, 0.0]
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


@dataclass(frozen=True)
class DrawerCandidate:
    path: str
    center: tuple[float, float, float]
    extent: tuple[float, float, float]
    score: int


@dataclass(frozen=True)
class DrawerMotionWaypoints:
    handle_xyz: tuple[float, float, float]
    preapproach_xyz: tuple[float, float, float]
    opened_xyz: tuple[float, float, float]
    observe_xyz: tuple[float, float, float]


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
    parser.add_argument("--simulate-twin", action=argparse.BooleanOptionalAction, default=True, help="Animate robot joints.")
    parser.add_argument("--tool", default="screwdriver", choices=tuple(sorted(TOOL_DRAWER_IDS)), help="Tool drawer mapping.")
    parser.add_argument("--drawer-id", type=int, choices=(0, 1, 2), default=None, help="Override drawer identity.")
    parser.add_argument("--print-sim-targets", action="store_true", help="Print auto-detected parameters.")
    parser.add_argument("--physics-steps-per-frame", type=int, default=16, help="Physics updates per frame.")
    parser.add_argument("--author-joint-drives", action=argparse.BooleanOptionalAction, default=False, help="Write USD drives.")
    parser.add_argument("--drawer-preapproach-joint-blend", type=float, default=DRAWER_HANDLE_PREAPPROACH_JOINT_BLEND, help="Preapproach blend fraction.")
    parser.add_argument("--show-waypoint-marker", action=argparse.BooleanOptionalAction, default=False, help="Show marker at target.")
    parser.add_argument("--joint-signs", default=",".join(str(int(value)) for value in DEFAULT_JOINT_SIGNS), help="Joint signs.")
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True, help="Run without UI.")
    parser.add_argument("--renderer", default="RaytracedLighting", choices=("RaytracedLighting", "RealTimePathTracing", "PathTracing", "MinimalRendering"), help="Renderer mode.")
    parser.add_argument("--camera-mode", default="orbit", choices=("orbit", "fixed"), help="Camera movement style.")
    parser.add_argument("--camera-target", type=float, nargs=3, default=(0.0, 0.0, 0.75), metavar=("X", "Y", "Z"), help="Camera lookat center.")
    parser.add_argument("--camera-radius", type=float, default=3.2, help="Orbit distance.")
    parser.add_argument("--camera-height", type=float, default=1.8, help="Camera altitude.")
    parser.add_argument("--camera-start-deg", type=float, default=-35.0, help="Start angle.")
    parser.add_argument("--camera-end-deg", type=float, default=55.0, help="End angle.")
    parser.add_argument("--fixed-camera-position", type=float, nargs=3, default=(2.6, -2.4, 1.8), metavar=("X", "Y", "Z"), help="Fixed camera coords.")
    parser.add_argument("--drawer-prim", default="", help="Target drawer prim path.")
    parser.add_argument("--articulation-root", default="/m0609/base_link", help="Robot physics articulation root.")
    parser.add_argument("--drawer-axis", default=DEFAULT_DRAWER_AXIS, choices=("x", "y", "z"), help="Drawer travel axis.")
    parser.add_argument("--drawer-distance", type=float, default=DEFAULT_DRAWER_DISTANCE, help="Drawer stroke length.")
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


def detect_robot_joint_prims() -> dict[str, str]:
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    joint_paths: dict[str, str] = {}
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in ROBOT_JOINT_NAMES:
            joint_paths[name] = prim.GetPath().pathString
    return joint_paths


def detect_end_effector_prim() -> str:
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    preferred = ("link_6", "tool0", "tcp", "gripper")
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if any(p in path.lower() for p in preferred):
            if "visual" not in path.lower() and "collision" not in path.lower():
                return path
    return ""


def world_translation(prim_path: str) -> tuple[float, float, float] | None:
    from pxr import UsdGeom
    import omni.usd
    if not prim_path: return None
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid(): return None
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
    trans = matrix.ExtractTranslation()
    return (float(trans[0]), float(trans[1]), float(trans[2]))


def detect_gripper_prims() -> list[str]:
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    paths = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if any(w in path.lower() for w in ("finger", "jaw", "knuckle")) and "Joint" in prim.GetTypeName():
            paths.append(path)
    return paths[:8]


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
    if prim and prim.IsValid():
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        return preferred
    return "/m0609/base_link"


def dynamic_control_candidate_paths(root: str) -> list[str]:
    return [root, "/m0609/base_link", "/m0609"]


def offset_xyz(base: tuple[float, float, float], offset: tuple[float, float, float] | list[float]) -> tuple[float, float, float]:
    return (base[0] + offset[0], base[1] + offset[1], base[2] + offset[2])


def plan_drawer_motion_waypoints(
    simulation_app, joint_paths: dict[str, str], ee_path: str, drawer_id: int, signs: list[float], root: str
) -> DrawerMotionWaypoints | None:
    # 🎯 복원된 딕셔너리 구조에 따라 drawer_id에 알맞은 매핑 인덱싱 적용
    handle_joints = apply_joint_signs(JOINT_BOUNDARIES["HANDLE"][drawer_id], signs)
    if not set_dynamic_control_targets(handle_joints, root):
        set_robot_joint_targets(joint_paths, handle_joints)
    for _ in range(30): simulation_app.update()
    
    handle_xyz = (0.55, 0.24, 0.72)
    pre_xyz = offset_xyz(handle_xyz, (DRAWER_HANDLE_PREAPPROACH_X_OFFSET_M, 0.0, 0.0))
    open_xyz = offset_xyz(handle_xyz, DRAWER_OPEN_OFFSET_XYZ_M)
    obs_xyz = offset_xyz(open_xyz, DRAWER_OBSERVE_OFFSET_XYZ_M)
    return DrawerMotionWaypoints(handle_xyz=handle_xyz, preapproach_xyz=pre_xyz, opened_xyz=open_xyz, observe_xyz=obs_xyz)


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


def set_gripper_opening(gripper_paths: list[str], opening_m: float, root_path: str = "") -> None:
    from pxr import UsdPhysics
    import omni.usd
    if not gripper_paths: return
    stage = omni.usd.get_context().get_stage()
    dc_applied = False
    target_rad = ((0.036 - opening_m) / (0.036 - 0.004)) * 0.85
    try:
        from omni.isaac.dynamic_control import _dynamic_control
        dc = _dynamic_control.acquire_dynamic_control_interface()
        articulation = 0
        for p in [root_path, "/m0609/base_link", "/m0609"]:
            articulation = dc.get_articulation(p)
            if articulation != 0: break
        if articulation != 0:
            for path in gripper_paths:
                dof_name = path.split("/")[-1]
                dof = dc.find_articulation_dof(articulation, dof_name)
                if dof:
                    direction = -1.0 if any(w in dof_name.lower() for w in ("right", "left_inner_finger")) else 1.0
                    dc.set_dof_position_target(dof, float(direction * target_rad))
            dc_applied = True
    except Exception: pass
    if not dc_applied:
        for path in gripper_paths:
            prim = stage.GetPrimAtPath(path)
            drive = UsdPhysics.DriveAPI.Get(prim, "angular") or UsdPhysics.DriveAPI.Get(prim, "linear")
            if drive: drive.CreateTargetPositionAttr().Set(float(target_rad))


def animate_drawer(drawer_prim: str, axis: str, distance: float, progress: float) -> None:
    if not drawer_prim: return
    from pxr import Gf, UsdGeom
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(drawer_prim)
    if not prim or not prim.IsValid(): return
    offset = {"x": (distance, 0.0, 0.0), "y": (0.0, distance, 0.0), "z": (0.0, 0.0, distance)}
    trans = tuple(c * progress for c in offset[axis])
    xform = UsdGeom.Xformable(prim)
    ops = list(xform.GetOrderedXformOps())
    top = ops[0] if ops and ops[0].GetOpType() == UsdGeom.XformOp.TypeTranslate else xform.AddTranslateOp()
    if drawer_prim not in BASE_DRAWER_TRANSLATES:
        curr = top.Get() or Gf.Vec3d(0.0, 0.0, 0.0)
        BASE_DRAWER_TRANSLATES[drawer_prim] = (float(curr[0]), float(curr[1]), float(curr[2]))
    base = BASE_DRAWER_TRANSLATES[drawer_prim]
    top.Set(Gf.Vec3d(base[0] + trans[0], base[1] + trans[1], base[2] + trans[2]))


def ease_in_out(amount: float) -> float:
    amount = min(max(float(amount), 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * amount)


def interpolate_pose(start: list[float], end: list[float], amount: float) -> list[float]:
    t = ease_in_out(amount)
    return [float(a) + (float(b) - float(a)) * t for a, b in zip(start, end)]


# ==============================================================================
# 🧠 [NATIVE TRAJECTORY OPTIMIZER CORE] 카테시안 일직선 추종 비선형 보상 제어기
# ==============================================================================
def robot_pose_for_frame(
    frame_index: int, frames: int, drawer_id: int, blend: float
) -> tuple[list[float], float, float]:
    if frames <= 1: return HOME_JOINT_DEGREES, 0.0, GRIPPER_OPEN_M
    t = frame_index / (frames - 1)
    
    # Phase 1: 홈에서 관측 및 대기 상태 진입
    if t < 0.10: return interpolate_pose(HOME_JOINT_DEGREES, OBSERVATION_JOINT_DEGREES, t / 0.10), 0.0, GRIPPER_OPEN_M
    if t < 0.20: return OBSERVATION_JOINT_DEGREES, 0.0, GRIPPER_OPEN_M
    
    # 🎯 [동적 스위칭] 입력된 서랍 층수(drawer_id)에 맞는 시작/끝 하드코딩 경계 포즈 로드
    p_start = JOINT_BOUNDARIES["HANDLE"][drawer_id]
    p_end = JOINT_BOUNDARIES["PULL"][drawer_id]
    
    # Phase 2: 프리어프로치 접근 구간
    if t < 0.35: 
        pre_pose = interpolate_pose(OBSERVATION_JOINT_DEGREES, p_start, blend)
        return interpolate_pose(OBSERVATION_JOINT_DEGREES, pre_pose, (t - 0.20) / 0.15), 0.0, GRIPPER_OPEN_M
    if t < 0.45: 
        pre_pose = interpolate_pose(OBSERVATION_JOINT_DEGREES, p_start, blend)
        return interpolate_pose(pre_pose, p_start, (t - 0.35) / 0.10), 0.0, GRIPPER_OPEN_M
    if t < 0.50: return p_start, 0.0, GRIPPER_CLOSED_M
    
    # --------------------------------------------------------------------------
    # 🔥 [당기기 구간] 비선형 공간 도메인 삼각 함수 보상 연산 구역 (Left Drift 완벽 소거)
    # --------------------------------------------------------------------------
    if t < 0.65:  
        amt = (t - 0.50) / 0.15
        amt_robot = ease_in_out(min(amt * 1.25, 1.0))
        
        # 1. 기본 선형 보간 베이스라인
        pose = [float(start) + (float(end) - float(start)) * amt_robot for start, end in zip(p_start, p_end)]
        
        # 2. 🎯 [건호님 아이디어 반영] 중간 궤적이 왼쪽(+)으로 부풀어 오르는 현상 억제 마스크
        sin_mask = math.sin(amt_robot * math.pi)
        
        pose[0] -= sin_mask * 0.95  # J1(베이스): 왼쪽 선회 타이밍을 강제로 우측(-)으로 눌러줌
        pose[2] += sin_mask * 0.45  # J3(엘보우): 팔이 접히면서 대각선으로 밀리는 리치 보정
        
        return pose, amt_robot, GRIPPER_CLOSED_M
        
    # --------------------------------------------------------------------------
    # 🔥 [밀기 구간] 복귀 시에도 대칭 기하학적 카운터 토크 인가
    # --------------------------------------------------------------------------
    if t < 0.90:  
        amt = (t - 0.75) / 0.15
        amt_robot = ease_in_out(min(amt * 1.25, 1.0))
        drawer_prog = 1.0 - amt_robot
        
        # 기본 복귀 보간
        pose = [float(start) + (float(end) - float(start)) * amt_robot for start, end in zip(p_end, p_start)]
        
        # 나갈 때도 동일한 곡률 대칭 적용
        sin_mask = math.sin(amt_robot * math.pi)
        pose[0] -= sin_mask * 0.95
        pose[2] += sin_mask * 0.45
        
        return pose, drawer_prog, GRIPPER_CLOSED_M
        
    amt = (t - 0.90) / 0.10
    return interpolate_pose(p_start, HOME_JOINT_DEGREES, amt), 0.0, GRIPPER_OPEN_M


def apply_digital_twin_frame(
    args: argparse.Namespace, frame_index: int, joint_paths: dict[str, str], gripper_paths: list[str], drawer_prim: str, drawer_id: int, signs: list[float], root_path: str
) -> None:
    if not args.simulate_twin:
        animate_drawer(drawer_prim, args.drawer_axis, args.drawer_distance, drawer_progress_for_frame(frame_index, args.frames))
        return
    
    joint_degrees, drawer_progress, gripper_opening = robot_pose_for_frame(frame_index, args.frames, drawer_id, args.drawer_preapproach_joint_blend)
    signed_joints = apply_joint_signs(joint_degrees, signs)
    
    if not set_dynamic_control_targets(signed_joints, root_path):
        set_robot_joint_targets(joint_paths, signed_joints)
    set_gripper_opening(gripper_paths, gripper_opening, root_path)
    animate_drawer(drawer_prim, args.drawer_axis, args.drawer_distance, drawer_progress)


def drawer_progress_for_frame(frame_index: int, frames: int) -> float:
    if frames <= 1: return 1.0
    return ease_in_out(frame_index / (frames - 1))


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
        
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(60): simulation_app.update()
        
        if args.list_stage: list_stage_prims()
        if args.find_prim: find_stage_prims(args.find_prim)
        if args.list_articulation_debug: list_articulation_debug()
        
        if args.dry_inspect or args.list_stage or args.find_prim or args.list_articulation_debug:
            print("\n🔍 [INSPECT COMPLETE] 요청한 데이터 구조 조회가 완료되어 안전하게 종료합니다.")
            timeline.pause()
            return 0
            
        joint_paths = detect_robot_joint_prims()
        root_path = ensure_robot_articulation_root(joint_paths, args.articulation_root)
        
        print(f"\n🚀 [STANDALONE DEPLOYED] Native Mathematical 3D-Straight Tracking Engine 구동")
            
        gripper_paths = detect_gripper_prims()
        
        output_dir = clean_output_dir(args.output_dir)
        render_prod = rep.create.render_product(camera_path, (args.width, args.height))
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=str(output_dir), rgb=True)
        writer.attach([render_prod])
        
        for frame_index in range(args.frames):
            set_camera_pose(camera_path, camera_position_for_frame(args, frame_index), args.camera_target)
            apply_digital_twin_frame(args, frame_index, joint_paths, gripper_paths, args.drawer_prim, drawer_id, joint_signs, root_path)
            for _ in range(max(args.physics_steps_per_frame, 1)): simulation_app.update()
            rep.orchestrator.step(rt_subframes=args.rt_subframes)
            simulation_app.update()
            
        timeline.pause()
        rep.orchestrator.wait_until_complete()
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
