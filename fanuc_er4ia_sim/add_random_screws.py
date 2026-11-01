"""Add reproducible M6x30 screws after cell.build(stage), before world.reset().

Uses the organizer's STL for visuals and a shaft + countersunk-head collider.
Existing scene objects stay as-is. This script can also export a screws-only USD
layer: run with the same Isaac Python, from the repository containing scene_config.
"""
from __future__ import annotations
import argparse
import math
from pathlib import Path
import random
import struct

import numpy as np
import sort_config as S

ASSET = Path(__file__).parent / "assets" / "ISO10642-M6x30-countersunk-screw.stl"


def layout(config, count=6, seed=42, drops=None):
    rng = random.Random(seed)
    drops = S.DROP_XY if drops is None else drops
    poses = []
    for _ in range(count):
        for attempt in range(10000):
            x, y = rng.uniform(0.24, 0.47), rng.uniform(-0.25, 0.25)
            point = np.array([x, y])
            if np.linalg.norm(point) > S.MAX_RADIUS - 0.025:
                continue
            if any(np.linalg.norm(point-p) < S.DROP_RADIUS+0.025 for p in drops.values()):
                continue
            occupied = list(config.SORT_CUBES) + list(config.SORT_CAPS) + poses
            if any(math.dist((x,y), p[:2]) < 0.055 for p in occupied):
                continue
            blocked = False
            for vx, vy, yaw in config.SORT_VIALS:
                axis = np.array([math.cos(math.radians(yaw)), math.sin(math.radians(yaw))])
                t = np.clip((point-[vx,vy])@axis, -config.VIAL_LENGTH/2, config.VIAL_LENGTH/2)
                if np.linalg.norm(point - (np.array([vx,vy])+t*axis)) < config.VIAL_RADIUS+0.035:
                    blocked = True
                    break
            if not blocked:
                poses.append((x, y, rng.uniform(-180,180)))
                break
        else:
            raise ValueError(f"Only {len(poses)} separated screws fit; request fewer or change the seed")
    return poses


def stl_points(path=ASSET):
    data = Path(path).read_bytes()
    n = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84+n*50:
        raise ValueError("Expected the supplied binary STL")
    vertices = np.array([struct.unpack_from("<9f", data, 84+i*50+12) for i in range(n)]).reshape(-1,3)
    # Supplied STL: Z=-30..0 mm, head at Z=0. Long axis becomes local +X.
    return np.column_stack((vertices[:,2]+15, vertices[:,0], vertices[:,1])) / 1000


def add_random_screws(stage, config, count=6, seed=42, drops=None):
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade
    points = stl_points()
    material = UsdShade.Material.Define(stage, "/World/ScrewContactMaterial")
    friction = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    friction.CreateStaticFrictionAttr(0.6)
    friction.CreateDynamicFrictionAttr(0.5)
    friction.CreateRestitutionAttr(0.0)
    paths = []
    for i, (x, y, yaw) in enumerate(layout(config, count, seed, drops)):
        path = f"/World/SortScrew{i}"
        if stage.GetPrimAtPath(path):
            raise ValueError(f"{path} already exists")
        root = UsdGeom.Xform.Define(stage, path)
        root.AddTranslateOp().Set(Gf.Vec3d(x,y,config.TABLE_TOP_Z+config.SURFACE_DZ+0.007))
        root.AddRotateZOp().Set(yaw)
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(0.006)  # estimated 6 g
        visual = UsdGeom.Mesh.Define(stage, path+"/Visual")
        visual.CreatePointsAttr(points.tolist())
        visual.CreateFaceVertexCountsAttr([3]*(len(points)//3))
        visual.CreateFaceVertexIndicesAttr(list(range(len(points))))
        visual.CreateSubdivisionSchemeAttr("none")
        visual.CreateDisplayColorAttr([(0.20,0.21,0.22)])
        # Separate colliders avoid a whole-screw convex hull filling in around its shaft.
        shaft = UsdGeom.Cylinder.Define(stage, path+"/ShaftCollision")
        shaft.CreateAxisAttr("X")
        shaft.CreateRadiusAttr(0.003)
        shaft.CreateHeightAttr(0.027)
        shaft.AddTranslateOp().Set(Gf.Vec3d(-0.0015,0,0))
        head = UsdGeom.Mesh.Define(stage, path+"/HeadCollision")
        n = 24
        ring = [(xx,rr*math.cos(2*math.pi*j/n),rr*math.sin(2*math.pi*j/n))
                for xx,rr in [(0.012,0.003),(0.015,0.006)] for j in range(n)]
        faces = [list(range(n-1,-1,-1)),list(range(n,2*n))]
        faces += [[j,(j+1)%n,(j+1)%n+n,j+n] for j in range(n)]
        head.CreatePointsAttr(ring)
        head.CreateFaceVertexCountsAttr([len(f) for f in faces])
        head.CreateFaceVertexIndicesAttr([k for f in faces for k in f])
        head.CreateSubdivisionSchemeAttr("none")
        UsdPhysics.MeshCollisionAPI.Apply(head.GetPrim()).CreateApproximationAttr("convexHull")
        for collider in (shaft,head):
            UsdGeom.Imageable(collider.GetPrim()).CreateVisibilityAttr("invisible")
            UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
            UsdShade.MaterialBindingAPI.Apply(collider.GetPrim()).Bind(material,
                bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
        paths.append(path)
    return paths


def add_misc_demo(stage, config, screw_count=6, seed=42, drops=None):
    """One purple 16 x 7 x 8 mm block, outside all supported shape/colour classes."""
    from pxr import Gf, UsdGeom, UsdPhysics
    x, y, yaw = layout(config, screw_count + 1, seed, drops)[-1]
    block = UsdGeom.Cube.Define(stage, "/World/SortMisc0")
    block.CreateSizeAttr(1.0)
    block.AddTranslateOp().Set(Gf.Vec3d(x, y, config.TABLE_TOP_Z + config.SURFACE_DZ + .0045))
    block.AddRotateZOp().Set(yaw)
    block.AddScaleOp().Set(Gf.Vec3f(.016, .007, .008))
    block.CreateDisplayColorAttr([(.65, .10, .65)])
    UsdPhysics.RigidBodyAPI.Apply(block.GetPrim())
    UsdPhysics.CollisionAPI.Apply(block.GetPrim())
    UsdPhysics.MassAPI.Apply(block.GetPrim()).CreateMassAttr(.004)
    return str(block.GetPath())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count",type=int,default=6)
    parser.add_argument("--seed",type=int,default=42)
    parser.add_argument("--output",default="random_screws.usda")
    args = parser.parse_args()
    from pxr import Usd, UsdGeom
    import scene_config as C
    stage = Usd.Stage.CreateNew(args.output)
    UsdGeom.SetStageMetersPerUnit(stage,1.0)
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    paths = add_random_screws(stage,C,args.count,args.seed)
    stage.GetRootLayer().Save()
    print(f"Wrote {len(paths)} screws to {args.output}")
