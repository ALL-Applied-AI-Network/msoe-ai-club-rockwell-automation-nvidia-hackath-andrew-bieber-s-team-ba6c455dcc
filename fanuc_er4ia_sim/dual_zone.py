"""Two side-by-side grasp zones on each rigid finger. SI units.

Simulation prototype: no moving linkage, no increased actuator stroke.
The cap rail is displaced along local +Z, perpendicular to the closing axis Y.
No CAD mounting/strength certification is implied.
"""
import numpy as np
import scene_config as C

ZONE_OFFSET = 0.040
CAP_FACE = 0.007
CAP_PICK_HEIGHT = 0.005
BRIDGE_LENGTH = 0.020

def extra_boxes(sign):
    """(name, XYZ size, centre in finger-root frame, is_pad). Convex colliders."""
    L, W, t, thick = C.FINGER_LENGTH, C.FINGER_WIDTH, C.PAD_T, C.FINGER_THICK
    back = CAP_FACE + t
    return [
        ("CapRail", (L, W-back, thick),
         (L/2, sign*(W+back)/2, ZONE_OFFSET), False),
        ("CapPad", (L-C.PAD_STEP_X, t, thick),
         ((L+C.PAD_STEP_X)/2, sign*(CAP_FACE+t/2), ZONE_OFFSET), True),
        ("Bridge", (BRIDGE_LENGTH, W-back, ZONE_OFFSET+thick),
         (BRIDGE_LENGTH/2, sign*(W+back)/2, ZONE_OFFSET/2), False),
    ]

def tool_offset(category):
    result = np.asarray(C.TOOL_AXIS, float)*C.TOOL_LENGTH
    if category == 'cap':
        result += np.cross(C.TOOL_AXIS, C.FINGER_SPREAD_AXIS)*ZONE_OFFSET
    return result

def width(category):
    if category == 'cube': return C.CUBE_EDGE
    if category == 'vial': return 2*C.VIAL_RADIUS
    if category == 'cap':
        from padded_fingers import cap_width_at
        return cap_width_at(CAP_PICK_HEIGHT)
    raise ValueError(category)

def face(category): return CAP_FACE if category == 'cap' else C.PAD_FACE

def jaw_target(category):
    w, f = width(category), face(category)
    if w+0.002 > 2*(f+C.JAW_OPEN):
        raise ValueError(f'{category}: insufficient opening with 1 mm/side clearance')
    target = w/2-f-0.0003
    if not C.JAW_CLOSED <= target <= C.JAW_OPEN:
        raise ValueError(f'{category}: outside actual actuator stroke')
    return target

def pick_spec(category, index):
    """Configured-position diagnostic only; no measured poses used for commands."""
    if C.FINGER_SET != 'dual': raise ValueError('Launch with FINGER_SET=dual')
    if category == 'cube':
        x,y,colour=C.SORT_CUBES[index]; yaw=C.SORT_CUBE_YAWS_DEG[index]
        height=C.CUBE_TIP_CLEARANCE; prim=f'/World/SortCube{index}'
    elif category == 'vial':
        x,y,yaw=C.SORT_VIALS[index]; height=0.003; prim=f'/World/SortVial{index}'
    elif category == 'cap':
        x,y=C.SORT_CAPS[index]; yaw=0.; height=CAP_PICK_HEIGHT; prim=f'/World/SortCap{index}'
        if C.CAP_ORIENTATIONS[index] != 'narrow_end_up':
            raise ValueError('Cap diagnostic supports narrow-end-up only')
    else: raise ValueError(category)
    yaw=np.deg2rad(yaw)
    # Vial rail displacement is along its length; inactive wider zone clears it.
    spread=np.array([-np.sin(yaw),np.cos(yaw),0.])
    return dict(category=category, index=index, prim=prim, xy=[x,y],
                pick_z=C.SURFACE_DZ+height, place_xy=[0.28,-0.32],
                place_z=C.SURFACE_DZ+height, spread_dir=spread,
                place_spread_dir=np.array([0.,1.,0.]), jaw_close=jaw_target(category))

def centre(position, quaternion, category):
    """USD root origin -> geometric centre, used for evaluation only."""
    w,x,y,z=quaternion
    local_z=np.array([2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)])
    length={'cube':C.CUBE_EDGE,'vial':C.VIAL_LENGTH,'cap':C.CAP_HEIGHT}[category]
    return np.asarray(position)+local_z*length/2

def report():
    print('DUAL ZONE PROTOTYPE: 6 mm/jaw, two fixed rails separated by 40 mm')
    for kind in ('cube','vial','cap'):
        f=face(kind)
        print(f'{kind:5} width={width(kind)*1000:.2f} mm; gap={2*f*1000:.1f}'
              f'--{2*(f+C.JAW_OPEN)*1000:.1f} mm; jaw={jaw_target(kind)*1000:.2f} mm')
    print('Cap flange 31.75 mm stays BELOW the tips; this zone cannot grip the flange.')
    print('Fixed sizes/orientations only. Physics and clutter clearance require Rosie validation.')
if __name__=='__main__': report()
