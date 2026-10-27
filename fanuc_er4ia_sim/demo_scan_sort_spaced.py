"""Launch the existing scan sorter with a separated initial object layout.
Place beside demo_scan_sort.py; run using the Isaac Python environment.
Only initial scene configuration changes. Dynamic objects may still roll.
"""
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
    ]
    C.SORT_CUBE_YAWS_DEG = [0., 26., 32., -23., 14., -36.]
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
    configure()
    print('[layout] Separated layout: 6 cubes, 2 parallel vials, 2 caps', flush=True)
    print('[layout] Existing scans, motion planning and collision gates remain active', flush=True)
    sys.argv[0] = str(target)
    runpy.run_path(str(target), run_name='__main__')


if __name__ == '__main__':
    main()
