"""Launch the existing scan sorter with a separated initial object layout.
Place beside demo_scan_sort.py; run using the Isaac Python environment.
Only initial scene configuration changes. Dynamic objects may still roll.
"""
import argparse
import os
import runpy
import sys
from pathlib import Path


def configure():
    os.environ.setdefault('FINGER_SET', 'dual')
    import scene_config as C
    if C.FINGER_SET not in ('dual', 'dual_v2'):
        raise RuntimeError('scene_config was already loaded without dual fingers')
    # metres, world XY. Preserve category counts and colour/index mapping.
    C.SORT_CUBES = [
        (.23, .08, 'yellow'), (.23, .20, 'yellow'),
        (.35, .08, 'red'),    (.35, .20, 'red'),
        (.47, .08, 'blue'),   (.47, .20, 'blue'),
        (.53, .01, 'blue', C.CUBE_LARGE_EDGE),  # 16 mm cube from the organizer CAD
    ]
    C.SORT_CUBE_YAWS_DEG = [0., 26., 32., -23., 14., -36., 15.]
    # M6 x 30 countersunk screws (centre x, y, axis yaw). Only dual_v2's narrow
    # zone (5.2 mm closed) can grip the 6 mm shank; 'dual' closes to 8 mm.
    C.SORT_SCREWS = [(.47, -.145, 0.), (.15, .24, 90.)] if C.FINGER_SET == 'dual_v2' else []
    if not C.SORT_SCREWS:
        print('[layout] Screws skipped: FINGER_SET=dual cannot grip a 6 mm shank', flush=True)
    # Long axes along X, centres separated by 150 mm in Y.
    C.SORT_VIALS = [(.32, -.05, 0.), (.32, -.20, 0.)]
    C.SORT_CAPS = [(.52, -.08), (.52, -.23)]
    return C


def main():
    root = Path(__file__).resolve().parent
    target = root / 'demo_scan_sort.py'
    if not target.is_file():
        raise SystemExit('Put this launcher beside your existing demo_scan_sort.py')
    sys.path.insert(0, str(root))
    # Benchmark options are handled here; everything else goes to demo_scan_sort.
    bench = argparse.ArgumentParser(add_help=False)
    bench.add_argument('--layout-seed', type=int,
                       help='Random planner-checked layout (benchmark); default: fixed layout')
    bench.add_argument('--physics-seed', type=int,
                       help='Scale object friction and per-type mass by U[0.8, 1.2]')
    options, rest = bench.parse_known_args(sys.argv[1:])
    C = configure()
    info = {}
    if options.layout_seed is not None:
        import bench_layout as L
        attempt, layout = L.valid_layout(C, options.layout_seed)
        info.update(layout_seed=options.layout_seed, layout_attempt=attempt, layout=layout)
        print(f'[layout] Random layout: seed {options.layout_seed}, attempt {attempt}', flush=True)
    if options.physics_seed is not None:
        import bench_layout as L
        info.update(physics_seed=options.physics_seed, physics=L.apply_physics(C, options.physics_seed))
        print(f"[layout] Physics variation: {info['physics']}", flush=True)
    C.BENCHMARK_INFO = info or None
    sys.argv[1:] = rest
    print('[layout] Separated layout: 2 M6 screws, 6 x 10 mm + 1 x 16 mm cubes, 2 vials, 2 caps', flush=True)
    print('[layout] Existing scans, motion planning and collision gates remain active', flush=True)
    sys.argv[0] = str(target)
    runpy.run_path(str(target), run_name='__main__')


if __name__ == '__main__':
    main()
