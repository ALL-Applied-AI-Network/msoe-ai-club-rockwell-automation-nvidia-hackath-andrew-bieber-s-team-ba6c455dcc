"""Two side-by-side grasp zones on each rigid finger. SI units.

Simulation prototype: no moving linkage, no increased actuator stroke.
The secondary rail is displaced along local +Z, perpendicular to closing axis Y.
The optional dual_v2 profile widens that rail for real-size vials and narrows
the primary gap for screws; dual preserves Asher's original prototype.
No CAD mounting/strength certification is implied.
"""
import numpy as np
import scene_config as C

ZONE_OFFSET = 0.040
CAP_FACE = 0.009 if C.FINGER_SET == "dual_v2" else 0.007
CAP_OUTER_WIDTH = C.FINGER_WIDTH
# Place the wide rail 20 mm in from a real vial end; the inactive primary
# rail then lies 20 mm beyond the opposite near end of the cylinder.
VIAL_CONTACT_OFFSET = 0.0815 if C.FINGER_SET == "dual_v2" else 0.0
CAP_PICK_HEIGHT = 0.005
# v2.1: the wide rail and its pad run 50 mm along the vial (instead of 6 mm),
# extending away from the narrow rail toward the vial's centre. A 6 mm pad let
# the off-centre-gripped vial pivot ~45 deg; the long pad resists that twist.
WIDE_RAIL_LENGTH = 0.050 if C.FINGER_SET == "dual_v2" else C.FINGER_THICK
# The taller bridge stiffens the longer overhang (CAD bridge 25.8-45.8 mm).
BRIDGE_LENGTH = 0.0383 if C.FINGER_SET == "dual_v2" else 0.020

def extra_boxes(sign):
    """(name, XYZ size, centre in finger-root frame, is_pad). Convex colliders."""
    L, W, t, thick = C.FINGER_LENGTH, CAP_OUTER_WIDTH, C.PAD_T, C.FINGER_THICK
    back = CAP_FACE + t
    R = WIDE_RAIL_LENGTH
    rail_z = ZONE_OFFSET - thick/2 + R/2  # starts at the old rail's near face
    return [
        ("CapRail", (L, W-back, R),
         (L/2, sign*(W+back)/2, rail_z), False),
        ("CapPad", (L-C.PAD_STEP_X, t, R),
         ((L+C.PAD_STEP_X)/2, sign*(CAP_FACE+t/2), rail_z), True),
        ("Bridge", (BRIDGE_LENGTH, W-back, ZONE_OFFSET+R),
         (BRIDGE_LENGTH/2, sign*(W+back)/2, (ZONE_OFFSET-thick+R)/2), False),
    ]

def tool_offset(category):
    result = np.asarray(C.TOOL_AXIS, float)*C.TOOL_LENGTH
    lateral = ZONE_OFFSET if category == 'cap' else 0.0
    if category == 'vial' and C.FINGER_SET == 'dual_v2':
        # Virtual point above the object centre, not a second physical jaw.
        lateral = ZONE_OFFSET + VIAL_CONTACT_OFFSET
    result += np.cross(C.TOOL_AXIS, C.FINGER_SPREAD_AXIS)*lateral
    return result

def width(category):
    if category == 'cube': return C.CUBE_EDGE
    if category == 'screw': return C.SCREW_SHANK_D
    if category == 'vial': return 2*C.VIAL_RADIUS
    if category == 'cap':
        from padded_fingers import cap_width_at
        return cap_width_at(CAP_PICK_HEIGHT)
    raise ValueError(category)

def face(category):
    wide = category == 'cap' or (category == 'vial' and C.FINGER_SET == 'dual_v2')
    return CAP_FACE if wide else C.PAD_FACE

def jaw_target(category, measured_width=None):
    if category not in ("cube", "screw", "vial", "cap"):
        raise ValueError(f"Unsupported object category: {category}")
    w, f = width(category) if measured_width is None else float(measured_width), face(category)
    if not np.isfinite(w) or w <= 0:
        raise ValueError('Width must be finite and positive')
    clearance = 0.0005 if category == 'cube' and C.FINGER_SET == 'dual_v2' else 0.001
    if w+2*clearance > 2*(f+C.JAW_OPEN)+1e-12:
        raise ValueError(f'{category}: insufficient opening with {clearance*1000:g} mm/side clearance')
    target = w/2-f-squeeze(category)
    if not C.JAW_CLOSED <= target <= C.JAW_OPEN:
        raise ValueError(f'{category}: outside actual actuator stroke')
    return target

def squeeze(category):
    """Commanded pad overlap; the jaw drive turns it into grip force."""
    return C.VIAL_GRIP_SQUEEZE if category == 'vial' else 0.0003

def vial_hold():
    """Static check that the grip resists the vial's weight, slip and pivot.

    Off-centre grips (dual_v2) must resist the vial's weight torque about the
    finger closing axis through friction over the pad contact patch. A 1.5 g
    vertical load factor covers lift/carry accelerations.
    """
    force = min(C.JAW_DRIVE_STIFF*squeeze('vial'), C.JAW_MAX_FORCE)  # per finger
    mu = min(C.SOFT_TIP_STATIC_FRIC, C.SOFT_TIP_DYNAMIC_FRIC)
    load = C.VIAL_MASS*9.81*1.5
    if C.FINGER_SET == 'dual_v2':
        # Long pad: weight acts at the pad's centre, friction spread over its length
        # (uniform pressure: mean lever = length/4 per finger).
        lever = VIAL_CONTACT_OFFSET - (WIDE_RAIL_LENGTH - C.FINGER_THICK)/2
        twist_capacity = 2*mu*force*WIDE_RAIL_LENGTH/4
    else:
        lever = VIAL_CONTACT_OFFSET
        twist_capacity = 2*mu*force*C.GRIP_PATCH_RADIUS
    twist_needed = load*lever
    slip_capacity = 2*mu*force
    return dict(grip_force_n=force, friction=mu, contact_offset_m=lever,
                slip_margin=slip_capacity/load,
                twist_margin=(twist_capacity/twist_needed if twist_needed > 0 else float('inf')))

def pick_spec(category, index):
    """Configured-position diagnostic only; no measured poses used for commands."""
    if C.FINGER_SET not in ('dual', 'dual_v2'):
        raise ValueError('Launch with FINGER_SET=dual or dual_v2')
    size=None
    if category == 'cube':
        x,y=C.SORT_CUBES[index][:2]; yaw=C.SORT_CUBE_YAWS_DEG[index]
        height=C.CUBE_TIP_CLEARANCE; prim=f'/World/SortCube{index}'; size=C.cube_edge(index)
    elif category == 'screw':
        # Grip the shank at the screw's mid-length (the scanned silhouette centre).
        x,y,yaw=C.SORT_SCREWS[index]; height=C.TIP_CLEARANCE; prim=f'/World/SortScrew{index}'
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
    return dict(category=category, index=index, prim=prim, xy=[x,y], size=size,
                pick_z=C.SURFACE_DZ+height, place_xy=[0.28,-0.32],
                place_z=C.SURFACE_DZ+height, spread_dir=spread,
                place_spread_dir=np.array([0.,1.,0.]), jaw_close=jaw_target(category, size))

def centre(position, quaternion, category, size=None):
    """USD root origin -> geometric centre, used for evaluation only.
    size: cube edge (metres) when it differs from C.CUBE_EDGE."""
    w,x,y,z=quaternion
    local_z=np.array([2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)])
    length={'cube':size or C.CUBE_EDGE,'vial':C.VIAL_LENGTH,'cap':C.CAP_HEIGHT,
            'screw':C.SCREW_LENGTH}[category]
    return np.asarray(position)+local_z*length/2

def report():
    print(f'{C.FINGER_SET.upper()} PROTOTYPE: 6 mm/jaw, two fixed rails separated by 40 mm')
    kinds = ('screw', 'cube', 'vial', 'cap') if C.FINGER_SET == 'dual_v2' else ('cube','vial','cap')
    for kind in kinds:
        f=face(kind)
        print(f'{kind:5} width={width(kind)*1000:.2f} mm; gap={2*f*1000:.1f}'
              f'--{2*(f+C.JAW_OPEN)*1000:.1f} mm; jaw={jaw_target(kind)*1000:.2f} mm')
    if C.FINGER_SET == 'dual_v2':
        print(f'16 mm cube jaw target: {jaw_target("cube", .016)*1000:.2f} mm (0.5 mm/side entry clearance)')
        print(f'Vial: {C.VIAL_RADIUS*2000:.2f} x {C.VIAL_LENGTH*1000:.1f} mm; contact {VIAL_CONTACT_OFFSET*1000:.1f} mm from centre')
        print('Virtual vial-centre TCP: 121.5 mm from primary rail; off-centre grip stability unverified.')
    hold = vial_hold()
    print(f"Vial hold: {hold['grip_force_n']:.0f} N/finger, slip margin {hold['slip_margin']:.0f}x, "
          f"twist margin {hold['twist_margin']:.1f}x (wide pad {WIDE_RAIL_LENGTH*1000:g} mm long)")
    print('Cap flange 31.75 mm stays BELOW the tips; this zone cannot grip the flange.')
    print('Fixed sizes/orientations only. Physics and clutter clearance require Rosie validation.')
if __name__=='__main__': report()
