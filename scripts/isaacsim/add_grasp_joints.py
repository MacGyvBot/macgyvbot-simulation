#!/usr/bin/env python3
"""Add disabled FixedJoint prims to macgvbot.usd for each drawer.

Run with packman Python 3.11:
  PY311=/home/ssu/.cache/packman/chk/python/3.11.13+nv1-linux-x86_64
  USD_DIR=/home/ssu/.cache/packman/chk/usd.py311.manylinux_2_35_x86_64.stock.release/0.24.05.kit.7-gl.16400+05f48f24
  LD_LIBRARY_PATH=$USD_DIR/lib:$PY311/lib PYTHONPATH=$USD_DIR/lib/python \
    $PY311/bin/python3.11 scripts/isaacsim/add_grasp_joints.py
"""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
USD_PATH = REPO_ROOT / "macgvbot.usd"

GRIPPER_BODY = "/m0609/onrobot_rg2ft/gripper_body"
DRAWER_BODIES = {
    0: "/drawer/drawer_floor_01",
    1: "/drawer/drawer_floor_02",
    2: "/drawer/drawer_floor_03",
}


def main() -> None:
    from pxr import Gf, Sdf, Usd, UsdPhysics

    print(f"Opening: {USD_PATH}")
    stage = Usd.Stage.Open(str(USD_PATH))

    for drawer_id, drawer_path in DRAWER_BODIES.items():
        joint_path = f"/World/GraspJoint_floor_0{drawer_id + 1}"
        existing = stage.GetPrimAtPath(joint_path)
        if existing and existing.IsValid():
            print(f"  Already exists, overwriting: {joint_path}")
            stage.RemovePrim(joint_path)

        joint = UsdPhysics.FixedJoint.Define(stage, Sdf.Path(joint_path))
        joint.CreateBody0Rel().SetTargets([Sdf.Path(GRIPPER_BODY)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(drawer_path)])
        joint.CreateJointEnabledAttr().Set(False)
        joint.CreateBreakForceAttr().Set(1e10)
        joint.CreateBreakTorqueAttr().Set(1e10)
        # Author identity local frames — will be overwritten at runtime before enabling
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        print(f"  Added (disabled): {joint_path}")
        print(f"    body0 → {GRIPPER_BODY}")
        print(f"    body1 → {drawer_path}")

    stage.GetRootLayer().Save()
    print(f"Saved: {USD_PATH}")


if __name__ == "__main__":
    main()
