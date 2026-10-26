"""Five colored, physically enclosed tabletop sorting areas (no raised floor)."""
import numpy as np
import scene_config as C
BINS={
 'cube_yellow':((.16,-.36),(.096,.16),(1.,.75,.03)),
 'cube_red':((.26,-.36),(.096,.16),(.9,.06,.05)),
 'cube_blue':((.36,-.36),(.096,.16),(.05,.25,.9)),
 'vial':((.23,.39),(.20,.18),(.02,.7,.7)),
 'cap':((.41,.33),(.13,.14),(.65,.15,.8)),
}
SLOTS={
 'cube_yellow':[(.16,-.32),(.16,-.40)],
 'cube_red':[(.26,-.32),(.26,-.40)],
 'cube_blue':[(.36,-.32),(.36,-.40)],
 'vial':[(.23,.35),(.23,.43)],
 'cap':[(.41,.30),(.41,.36)],
}
WALL_HEIGHT=.008
WALL_THICK=.003

# End-region side stops leave the centre and offset cap rail unobstructed.
# Inner faces at +/-11 mm; radius 8.5 mm gives 2.5 mm lateral travel.
# No raised floor: the cylinder remains supported by the original table.
def vial_stops():
    for i,(x,y) in enumerate(SLOTS['vial']):
        for end in (-1,1):
            for side in (-1,1):
                yield ('vial', f'Cradle{i}_{end+1}_{side+1}',
                       np.array([x+end*.060, y+side*.013, C.SURFACE_DZ+.005]),
                       np.array([.008,.004,.010]), (.04,.40,.45))

def walls():
    yield from vial_stops()
    for group,((x,y),(w,d),color) in BINS.items():
        for name,size,xy in [
            ('Left',(WALL_THICK,d+2*WALL_THICK,WALL_HEIGHT),(x-w/2-WALL_THICK/2,y)),
            ('Right',(WALL_THICK,d+2*WALL_THICK,WALL_HEIGHT),(x+w/2+WALL_THICK/2,y)),
            ('Bottom',(w,WALL_THICK,WALL_HEIGHT),(x,y-d/2-WALL_THICK/2)),
            ('Top',(w,WALL_THICK,WALL_HEIGHT),(x,y+d/2+WALL_THICK/2))]:
            yield group,name,np.array([*xy,C.SURFACE_DZ+WALL_HEIGHT/2]),np.array(size),color

def build(stage):
    import cell
    from pxr import UsdGeom,Gf
    for group,((x,y),(w,d),color) in BINS.items():
        # Colored floor is visual only. Original support height stays unchanged.
        cell.box(stage,f'/World/SortingAreas/{group}/Color', (w,d,.0001),
            (x,y,C.TABLE_TOP_Z+C.SURFACE_DZ+.00006),color,collision=False)
    for group,name,center,size,color in walls():
        position=center+[0,0,C.TABLE_TOP_Z]
        prim=cell.box(stage,f'/World/SortingAreas/{group}/{name}',tuple(size),tuple(position),color)
        cell.bind_physics_material(prim);cell.small_contact_offsets(prim)
    # Slots are visible outlines; give all 5 vertices an explicit width.
    for group,slots in SLOTS.items():
        for i,(x,y) in enumerate(slots):
            hx,hy=(.073,.012) if group=='vial' else ((.019,.019) if group=='cap' else (.009,.009))
            z=C.TABLE_TOP_Z+C.SURFACE_DZ+.0002
            curve=UsdGeom.BasisCurves.Define(stage,f'/World/SortingAreas/{group}/Slot{i}')
            curve.CreateTypeAttr('linear');curve.CreateCurveVertexCountsAttr([5])
            curve.CreatePointsAttr([Gf.Vec3f(a,b,z) for a,b in [(x-hx,y-hy),(x+hx,y-hy),(x+hx,y+hy),(x-hx,y+hy),(x-hx,y-hy)]])
            curve.CreateWidthsAttr([.0006]*5);curve.CreateDisplayColorAttr([Gf.Vec3f(1,1,1)])
