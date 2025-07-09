import omni.usd
import omni.kit.commands

from pxr import Usd, UsdGeom, Sdf

# Global path
BASE_PATH = "/Users/Messe/Documents/KUKA_Training/Workshop/Isaac_Workshop/"
# USD-Dateipfade
TARGET_USD_PATH = "/Users/Public/Documents/Workshop/Scene_assembly/scene_assembly.usd"

ROBOT_USD = BASE_PATH + "Day2/01_SceneAssembly/iisy15.usd"
GRIPPER_USD = BASE_PATH + "Day2/01_SceneAssembly/Robotiq_2f_140.usd"

# 1. Hilfsfunktion: metersPerUnit für lokale USDs setzen
def set_meters_per_unit(usd_path, value=1.0):
    try:
        stage = Usd.Stage.Open(usd_path)
        if stage is not None:
            stage.SetMetadata("metersPerUnit", value)
            stage.GetRootLayer().Save()
            print(f"Setze metersPerUnit={value} in {usd_path}")
        else:
            print(f"Konnte {usd_path} nicht öffnen!")
    except Exception as e:
        print(f"Fehler beim Setzen von metersPerUnit in {usd_path}: {e}")

# 2. Szene und Platzhalter erzeugen
stage = Usd.Stage.CreateNew(TARGET_USD_PATH)
UsdGeom.Xform.Define(stage, "/World/GroundPlane")
ground_plane = UsdGeom.Mesh.Define(stage, "/World/GroundPlane/Plane")
# Create a simple quad as ground plane (10x10 meters)
half_size = 5.0
points = [
    (-half_size, 0.0, -half_size),
    (half_size, 0.0, -half_size),
    (half_size, 0.0, half_size),
    (-half_size, 0.0, half_size)
]
faceVertexIndices = [0, 1, 2, 3]
faceVertexCounts = [4]
ground_plane.CreatePointsAttr(points)
ground_plane.CreateFaceVertexIndicesAttr(faceVertexIndices)
ground_plane.CreateFaceVertexCountsAttr(faceVertexCounts)
ground_plane.CreateNormalsAttr([(0.0, 1.0, 0.0)] * 4)
ground_plane.CreateSubdivisionSchemeAttr("none")
UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Xform.Define(stage, "/World/Robot_assembly")
UsdGeom.Xform.Define(stage, "/World/Robot_assembly/Robot")
UsdGeom.Xform.Define(stage, "/World/Robot_assembly/Gripper")
UsdGeom.Xform.Define(stage, "/World/Robot_assembly/Camera")
UsdGeom.Xform.Define(stage, "/World/Robot_assembly/Base")
stage.SetMetadata("metersPerUnit", 1.0)
stage.GetRootLayer().Save()
print("Szene mit Platzhaltern erstellt!")

# 3.2 Prim-pfade
ROBOT_PRIM_PATH = "/World/Robot_assembly/Robot/iisy_15"
GRIPPER_PRIM_PATH = "/World/Robot_assembly/Gripper/Robotiq_2F_140"

set_meters_per_unit(ROBOT_USD, 1.0)

# 4. Szene laden und Payloads einfügen
ctx = omni.usd.get_context()
ctx.open_stage(TARGET_USD_PATH)
stage = ctx.get_stage()

if not stage.GetPrimAtPath(ROBOT_PRIM_PATH).IsValid():
    stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
stage.GetPrimAtPath(ROBOT_PRIM_PATH).GetPayloads().ClearPayloads()
omni.kit.commands.execute(
    "AddPayload",
    stage=stage,
    prim_path=ROBOT_PRIM_PATH,
    payload=Sdf.Payload(assetPath=ROBOT_USD)
)

if not stage.GetPrimAtPath(GRIPPER_PRIM_PATH).IsValid():
    stage.DefinePrim(GRIPPER_PRIM_PATH, "Xform")
stage.GetPrimAtPath(GRIPPER_PRIM_PATH).GetPayloads().ClearPayloads()
omni.kit.commands.execute(
    "AddPayload",
    stage=stage,
    prim_path=GRIPPER_PRIM_PATH,
    payload=Sdf.Payload(assetPath=GRIPPER_USD)
)

stage.GetRootLayer().Save()
print("Robot, Gripper, Camera und Action Graphs als Payloads exakt eingefügt!")


# 5.xformOp:scale:unitsResolve-Operationen entfernen -> Skalierung war sonst x100
def remove_scale_unitsresolve_ops(prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    new_ops = [op for op in ops if "scale:unitsResolve" not in op.GetName()]
    if len(new_ops) != len(ops):
        xform.SetXformOpOrder([op.GetName() for op in new_ops])
        if prim.HasAttribute("xformOp:scale:unitsResolve"):
            prim.RemoveAttribute("xformOp:scale:unitsResolve")
        print(f"Entfernt: xformOp:scale:unitsResolve von {prim_path}")

remove_scale_unitsresolve_ops("/World/Robot_assembly/Robot")
remove_scale_unitsresolve_ops(ROBOT_PRIM_PATH)
remove_scale_unitsresolve_ops("/World/Robot_assembly/Gripper")

stage.GetRootLayer().Save()
print("Alle xformOp:scale:unitsResolve-Operationen entfernt und Szene geladen!")

# 6. ASSEMBLY: Robot and Gripper assembly from (Isaac Sim RobotAssembler)
    #in the second script to be executed.

