# SAM 2 First Coordinate Test

This is the first no-training perception test for the FANUC project:

```text
saved top-camera RGB image
    -> SAM 2.1 automatic masks
    -> filtered object candidates
    -> pixel centers and image-plane grasp angles
    -> annotated image + detections.json
```

It **does not move the robot**. Confirm the overlay is correct before connecting
perception output to `demo_pick.py`.

## Why SAM 2.1 Small

- Apache 2.0 code and model.
- No task-specific training required for the first test.
- Automatic masks can locate blocks, caps, vials, and unfamiliar items without
  requiring a class name.
- The Small checkpoint is a reasonable first choice for Rosie's T4.

## Environment

Do not install SAM 2 into `~/venvs/isaac-check`. Keep the working Isaac Sim
environment untouched.

```bash
python3 -m venv "$HOME/venvs/sam2-check"
"$HOME/venvs/sam2-check/bin/python" -m pip install --upgrade pip

mkdir -p "$HOME/innovation-lab/external"
cd "$HOME/innovation-lab/external"
git clone https://github.com/facebookresearch/sam2.git

"$HOME/venvs/sam2-check/bin/python" -m pip install \
  torch torchvision opencv-python-headless pillow huggingface_hub

"$HOME/venvs/sam2-check/bin/python" -m pip install -e sam2
```

The first inference downloads `facebook/sam2.1-hiera-small` from Hugging Face.
SAM 2 officially requires Python 3.10+ and PyTorch 2.5.1+.

## Put the script in the repo

Recommended location:

```text
fanuc_er4ia_sim/vision/sam2_coordinate_test.py
```

## Run the first image

From `fanuc_er4ia_sim`:

```bash
"$HOME/venvs/sam2-check/bin/python" vision/sam2_coordinate_test.py \
  --image PATH/TO/top_camera.png \
  --output-dir runs/sam2_top_test
```

If the full image includes the robot, walls, or UI, limit inference to the
table workspace:

```bash
"$HOME/venvs/sam2-check/bin/python" vision/sam2_coordinate_test.py \
  --image PATH/TO/top_camera.png \
  --roi 250,150,1100,850 \
  --output-dir runs/sam2_top_test
```

Replace those ROI numbers with the actual table rectangle. Inspect:

```text
runs/sam2_top_test/annotated.png
runs/sam2_top_test/detections.json
```

If tiny caps are missed, rerun with more prompt points:

```bash
"$HOME/venvs/sam2-check/bin/python" vision/sam2_coordinate_test.py \
  --image PATH/TO/top_camera.png \
  --roi 250,150,1100,850 \
  --points-per-side 48 \
  --min-area-px 100 \
  --output-dir runs/sam2_top_test_dense
```

## Optional pixel-to-world conversion

After calibrating the fixed top camera, save a JSON file such as:

```json
{
  "pixel_to_world_homography": [
    [0.001, 0.0, -0.5],
    [0.0, 0.001, -0.3],
    [0.0, 0.0, 1.0]
  ]
}
```

The values above are placeholders, not valid robot calibration. Run with:

```bash
"$HOME/venvs/sam2-check/bin/python" vision/sam2_coordinate_test.py \
  --image PATH/TO/top_camera.png \
  --roi 250,150,1100,850 \
  --homography calibration/top_pixel_to_world.json \
  --output-dir runs/sam2_top_world_test
```

Each detection will then contain `world_xy`. Keep robot motion disabled until
the world coordinates have been checked against several known table points.

## Definition of success

The first test passes when:

1. Every graspable tabletop object has one useful mask.
2. Large background/table masks are absent.
3. The drawn centroid lies on a safe grasp region.
4. The reported orientation is sensible for elongated objects.
5. Pixel-to-world validation is within the gripper's tolerated XY error.

Only then replace the hardcoded cube coordinates in `demo_pick.py` with the
selected perception result.
