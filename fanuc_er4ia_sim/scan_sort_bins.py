"""Five colored, physically enclosed tabletop sorting areas (no raised floor)."""
import numpy as np
import scene_config as C
BINS={
 'cube_yellow':((.16,-.36),(.096,.16),(1.,.75,.03)),
 'cube_red':((.26,-.36),(.096,.16),(.9,.06,.05)),
 'cube_blue':((.36,-.36),(.096,.16),(.05,.25,.9)),
 'vial':((.23,.39),(.20,.18),(.02,.7,.7)),
 'cap':((.41,.33),(.13,.14),(.65,.15,.8)),
 # M6 screws and the 16 mm cube: the free reachable strip south of the cube areas.
 'screw':((.135,-.50),(.10,.05),(.35,.35,.38)),
 'cube_large':((.06,-.50),(.035,.035),(.2,.5,.9)),
}
SLOTS={
 'cube_yellow':[(.16,-.32),(.16,-.40)],
 'cube_red':[(.26,-.32),(.26,-.40)],
 'cube_blue':[(.36,-.32),(.36,-.40)],
 'vial':[(.23,.35),(.23,.43)],
 'cap':[(.41,.30),(.41,.36)],
 'screw':[(.11,-.50),(.16,-.50)],
 'cube_large':[(.06,-.50)],
}
if C.FINGER_SET == 'dual_v2':
    # Real 203 mm vials plus the inactive primary rail need a wider destination.
    # Keep the cap area separate; this is a v2 simulator fixture only.
    BINS['vial'] = ((.20,.39),(.27,.18),(.02,.7,.7))
    SLOTS['vial'] = [(.20,.35),(.20,.43)]
# Areas marked on the floor only (no walls): small parts that cannot roll away.
# The long wide-zone rail sits at fingertip height and cannot reach between
# walls of an area this small. Real bins replace these areas in step 3.
NO_WALLS={'screw','cube_large'}
WALL_HEIGHT=.008
WALL_THICK=.003

# End-region side stops leave the centre and offset cap rail unobstructed.
# Inner faces at +/-11 mm; radius 8.5 mm gives 2.5 mm lateral travel.
# No raised floor: the cylinder remains supported by the original table.
def vial_stops():
    # v2: the 50 mm wide pad covers 34.5-84.5 mm from the vial centre on one
    # side, so the stops sit near the middle (16-24 mm), clear of it.
    end_offset = .020 if C.FINGER_SET == 'dual_v2' else .060
    side_offset = C.VIAL_RADIUS + .0045
    for i,(x,y) in enumerate(SLOTS['vial']):
        for end in (-1,1):
            for side in (-1,1):
                yield ('vial', f'Cradle{i}_{end+1}_{side+1}',
                       np.array([x+end*end_offset, y+side*side_offset, C.SURFACE_DZ+.005]),
                       np.array([.008,.004,.010]), (.04,.40,.45))

def walls():
    yield from vial_stops()
    for group,((x,y),(w,d),color) in BINS.items():
        if group in NO_WALLS: continue
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
            hx,hy={'vial':(C.VIAL_LENGTH/2+.003,C.VIAL_RADIUS+.0035),'cap':(.019,.019),
                   'screw':(C.SCREW_LENGTH/2+.003,C.SCREW_HEAD_D/2+.002)}.get(group,(.011,.011))
            z=C.TABLE_TOP_Z+C.SURFACE_DZ+.0002
            curve=UsdGeom.BasisCurves.Define(stage,f'/World/SortingAreas/{group}/Slot{i}')
            curve.CreateTypeAttr('linear');curve.CreateCurveVertexCountsAttr([5])
            curve.CreatePointsAttr([Gf.Vec3f(a,b,z) for a,b in [(x-hx,y-hy),(x+hx,y-hy),(x+hx,y+hy),(x-hx,y+hy),(x-hx,y-hy)]])
            curve.CreateWidthsAttr([.0006]*5);curve.CreateDisplayColorAttr([Gf.Vec3f(1,1,1)])
