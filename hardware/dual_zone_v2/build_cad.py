"""Build the CSI dual-zone V2 geometric prototype with CadQuery 2.7.

Millimetres throughout. This preserves the real SMC jaw's 6 mm travel and
mounts outside its two stock D2 through-holes. It does not establish strength,
physical fit, gripping force, or controller tool calibration.

Run: python build_cad.py --reference /path/to/SMC-MHZL2-16D2.step
Outputs are regenerated below this script, without modifying the reference.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct

import cadquery as cq
import numpy as np

ROOT = Path(__file__).resolve().parent
D = json.loads((ROOT / "dimensions.json").read_text())


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1-x0, y1-y0, z1-z0,
                            cq.Vector(x0, y0, z0))


def bore(z):
    return cq.Solid.makeCylinder(
        D["mount_hole_diameter_print"]/2, 30,
        cq.Vector(D["mount_hole_x"], 0, z), cq.Vector(0, 1, 0))


def derived_gaps():
    return {name:[2*face, 2*(face+D['jaw_travel_each'])]
            for name, face in (('primary', D['primary_contact_y_closed']),
                               ('wide', D['wide_contact_y_closed']))}


def make_positive_body():
    """Positive-Y jaw attachment, in the fully closed SMC coordinate frame."""
    w = D["mount_plate_width_x"]/2
    back = D["outer_backplane_y_closed"]
    mount = box(-w, w, D["mount_outer_y_closed"], back,
                D["mount_plate_z_start"], D["bridge_z_end"])
    t = D["rail_thickness_x"]/2
    components = [mount]
    for x, face in ((0, D["primary_contact_y_closed"]),
                    (D["rail_separation_x"], D["wide_contact_y_closed"])):
        components.append(box(x-t, x+t,
            face+D["finished_pad_stack_thickness"], back,
            D["rail_start_z"], D["tip_z"]))
    components.append(box(-w, D["rail_separation_x"]+t,
        D["bridge_inner_y_closed"], back,
        D["bridge_z_start"], D["bridge_z_end"]))
    body = components[0].fuse(*components[1:]).clean()
    for z in D["mount_hole_z"]:
        body = body.cut(bore(z))
    return body.clean()


def make_positive_pads():
    t = D["rail_thickness_x"]/2
    return [box(x-t, x+t, face, face+D["finished_pad_stack_thickness"],
        D["tip_z"]-D["pad_length_z"], D["tip_z"])
        for x, face in ((0, D["primary_contact_y_closed"]),
                       (D["rail_separation_x"], D["wide_contact_y_closed"]))]


def print_orientation(shape, sign=1):
    """Put the common outer backplane on the bed; no automatic scaling."""
    oriented = shape.rotate((0, 0, 0), (1, 0, 0), -90*sign)
    b = oriented.BoundingBox()
    return oriented.translate((-b.xmin, -b.ymin, -b.zmin))


def mesh_report(path):
    """Count undirected STL edges after welding vertices at 1e-5 mm."""
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + 50*count:
        raise ValueError(f"Unexpected binary STL layout: {path.name}")
    dtype = np.dtype([('normal', '<f4', (3,)),
                      ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    records = np.frombuffer(raw, dtype=dtype, count=count, offset=84)
    vertices = records['vertices'].astype(float)
    flat = np.rint(vertices.reshape(-1, 3)*1e5).astype(np.int64)
    _, inverse = np.unique(flat, axis=0, return_inverse=True)
    triangles = inverse.reshape(-1, 3)
    edges = np.concatenate((triangles[:, [0, 1]], triangles[:, [1, 2]],
                            triangles[:, [2, 0]]))
    _, counts = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
    boundary = int(np.count_nonzero(counts != 2))
    if boundary:
        raise ValueError(f"{path.name}: {boundary} non-manifold/boundary edges")
    signed_volume = float(np.einsum('ij,ij->i', vertices[:,0],
        np.cross(vertices[:,1],vertices[:,2])).sum()/6)
    if signed_volume <= 0:
        raise ValueError(f"{path.name}: nonpositive oriented mesh volume")
    return dict(triangles=count, boundary_or_nonmanifold_edges=boundary,
                signed_volume_mm3=signed_volume,
                extents_mm=(vertices.max(axis=(0, 1))-vertices.min(axis=(0, 1))).tolist())


def save_stl(shape, path):
    shape.exportStl(str(path), tolerance=.01, angularTolerance=.08,
                    ascii=False, relative=False)
    return mesh_report(path)


def validate_shape(shape, name):
    if not shape.isValid() or len(shape.Solids()) != 1 or shape.Volume() <= 0:
        raise ValueError(f"Invalid or disconnected solid: {name}")
    return dict(valid=True, solids=1, volume_mm3=shape.Volume())


def assembly(body, pads, stock_body, jaws, jaw_open_mm, path):
    result = cq.Assembly(name=f'dual_zone_v2_open_{jaw_open_mm:g}mm_each')
    result.add(stock_body, name='SMC_body_reference', color=cq.Color(.65, .68, .72))
    for sign, jaw in ((1, jaws[0]), (-1, jaws[1])):
        label = 'left_positive_y' if sign > 0 else 'right_negative_y'
        result.add(jaw.translate((0, sign*(jaw_open_mm-6), 0)),
                   name=f'{label}_stock_jaw', color=cq.Color(.43, .45, .48))
        part = body if sign > 0 else body.mirror('XZ')
        part = part.translate((0, sign*jaw_open_mm, 0))
        result.add(part, name=f'{label}_printed_body', color=cq.Color(.10, .44, .72))
        for i, pad in enumerate(pads):
            piece = pad if sign > 0 else pad.mirror('XZ')
            result.add(piece.translate((0, sign*jaw_open_mm, 0)),
                       name=f'{label}_contact_pad_{i+1}', color=cq.Color(.95, .57, .12))
    result.export(str(path))


def verify_hardware_sweep(body, pads, stock_body, jaws):
    reports = []
    for travel in np.linspace(0, 6, 13):
        moving = []
        for sign, jaw in ((1, jaws[0]), (-1, jaws[1])):
            part = body if sign > 0 else body.mirror('XZ')
            printed = part.translate((0, sign*travel, 0))
            real_jaw = jaw.translate((0, sign*(travel-6), 0))
            # The intended mounting plane touches, but does not interpenetrate.
            other_jaw = jaws[1] if sign > 0 else jaws[0]
            other_jaw = other_jaw.translate((0, -sign*(travel-6), 0))
            tests = [('own_jaw', real_jaw), ('opposite_jaw', other_jaw),
                     ('actuator_body', stock_body)]
            for name, reference in tests:
                volume = printed.intersect(reference).Volume()
                if volume > 1e-5:
                    raise ValueError(f'{travel}mm side{sign}: {name} overlap {volume}mm^3')
            pieces = [printed]
            for pad in pads:
                p = pad if sign > 0 else pad.mirror('XZ')
                pieces.append(p.translate((0, sign*travel, 0)))
            moving.append(cq.Compound.makeCompound(pieces))
        volume = moving[0].intersect(moving[1]).Volume()
        if volume > 1e-5:
            raise ValueError(f'Opposed finger collision at {travel} mm: {volume}')
        reports.append(dict(jaw_open_mm_each=float(travel),
            printed_vs_stock_intersection_mm3=0.0, opposed_finger_intersection_mm3=volume))
    return reports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path,
        default=ROOT/'reference'/'SMC-MHZL2-16D2.step')
    args = parser.parse_args()
    out = ROOT/'cad'
    out.mkdir(exist_ok=True)
    body = make_positive_body()
    pads = make_positive_pads()
    negative = body.mirror('XZ')
    report = dict(design=D['design'], units='mm', scope='geometric CAD checks only',
                  stock_source='https://hackathon.sidclab.org/files/SMC-MHZL2-16D2.step',
                  stock_sha256=hashlib.sha256(args.reference.read_bytes()).hexdigest(),
                  solid_checks={}, mesh_checks={})
    for name, part, sign in (('left_body', body, 1), ('right_body', negative, -1)):
        report['solid_checks'][name] = validate_shape(part, name)
        report['mesh_checks'][name] = save_stl(print_orientation(part, sign), out/f'{name}.stl')
        cq.exporters.export(part, str(out/f'{name}.step'))
    # One pad STL, quantity FOUR. Finished thickness includes any adhesive stack.
    pad = box(0, D['rail_thickness_x'], 0, D['pad_length_z'],
              0, D['finished_pad_stack_thickness'])
    report['mesh_checks']['contact_pad_quantity_4'] = save_stl(pad, out/'contact_pad_quantity_4.stl')
    cq.exporters.export(pad, str(out/'contact_pad.step'))
    # Exact tongue mount alone, first physical check before the complete parts.
    w = D['mount_plate_width_x']/2
    coupon = box(-w, w, D['mount_outer_y_closed'], D['outer_backplane_y_closed'],
                 D['mount_plate_z_start'], D['stock_tongue_z_end'])
    for z in D['mount_hole_z']:
        coupon = coupon.cut(bore(z))
    coupon = coupon.clean()
    report['solid_checks']['mounting_coupon'] = validate_shape(coupon, 'mounting_coupon')
    report['mesh_checks']['mounting_coupon'] = save_stl(print_orientation(coupon), out/'mounting_coupon.stl')
    cq.exporters.export(coupon, str(out/'mounting_coupon.step'))
    solids = cq.importers.importStep(str(args.reference)).val().Solids()
    if len(solids) != 3:
        raise ValueError('Expected official body and two stock jaws')
    stock_body = max(solids, key=lambda x:x.Volume())
    jaws = sorted([s for s in solids if s is not stock_body], key=lambda s:s.Center().y, reverse=True)
    # CadQuery returns stable Python references in this list, but identify by volume too.
    if len(jaws) != 2:
        jaws = sorted(sorted(solids, key=lambda x:x.Volume())[:2], key=lambda s:s.Center().y, reverse=True)
    report['jaw_sweep'] = verify_hardware_sweep(body, pads, stock_body, jaws)
    for amount, name in ((0, 'assembly_closed'), (6, 'assembly_open')):
        assembly(body, pads, stock_body, jaws, amount, out/f'{name}.step')
    report['gap_endpoints_mm'] = derived_gaps()
    report['mounting_holes'] = dict(printed_diameter_mm=D['mount_hole_diameter_print'],
        stock_diameter_mm=D['stock_jaw_hole_diameter'],
        centers_stock_step_mm=[[D['mount_hole_x'],z] for z in D['mount_hole_z']],
        center_columns=['X','Z'], pitch_mm=D['mount_hole_z'][1]-D['mount_hole_z'][0],
        bore_axis='Y', mounting='outside each stock D2 tongue')
    report['tip_z_stock_step_mm'] = D['tip_z']
    report['bridge_clearance_above_tip_mm'] = D['tip_z']-D['bridge_z_end']
    report['low_rail_open_envelope_mm'] = {
        'thickness_along_offset':D['rail_thickness_x'],
        'full_outer_width':2*(D['outer_backplane_y_closed']+D['jaw_travel_each']),
        'offset':D['rail_separation_x']}
    report['nominal_object_fit'] = []
    objects = [('M6 shaft',6,'primary'),('9 mm cube',9,'primary'),
               ('10 mm cube',10,'primary'),('11 mm cube',11,'primary'),
               ('16 mm cube',16,'primary'),('cap plug at 5 mm',23.04825354655,'wide'),
               ('real vial',D['real_vial_diameter'],'wide')]
    for label,width,zone in objects:
        shut,opened = report['gap_endpoints_mm'][zone]
        entry = (opened-width)/2
        squeeze = D['modeled_squeeze_each']
        target = (width-shut)/2-squeeze
        required = D[f'{zone}_entry_clearance_each']
        fits = bool(entry+1e-9 >= required and 0 <= target <= D['jaw_travel_each'])
        if not fits:
            raise ValueError(f'Nominal geometric fit failed: {label}')
        report['nominal_object_fit'].append(dict(object=label,width_mm=width,zone=zone,
            entry_clearance_mm_each=entry,simulated_jaw_target_mm_each=target,
            binary_physical_command='close; jaw position is not controlled',geometric_fit=fits))
    vial_half = D['real_vial_length']/2
    grasp_bias = vial_half-D['vial_grasp_inset_from_end']
    report['vial_end_grasp'] = dict(centre_to_contact_mm=grasp_bias,
        inset_from_end_mm=D['vial_grasp_inset_from_end'],
        inactive_rail_axial_clearance_mm=grasp_bias+D['rail_separation_x']-
            D['rail_thickness_x']/2-vial_half,
        physical_retention_verified=False)
    report['limits'] = ['No physical mounting or gripping test', 'No structural or fatigue analysis',
                        'Reference is stock D2 actuator CAD, not a survey of the installed cell',
                        'Requires four finished 1 mm pads; adhesive thickness is included',
                        'Physical TCP and both grip-zone offsets require calibration',
                        'Real vial needs end-biased grasp so inactive narrow rail clears its length']
    (ROOT/'cad_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
