# Run the FANUC simulation on Rosie

**Goal:** open the robot scene in a browser desktop, inspect it, and run the one-cube test.

**Status — October 10, 2026:** desktop viewing, camera captures, and manual joint movement work. The 33 starter host tests pass. The new `demo_pick.py` passes numerical IK checks; its actual simulated grasp is **not yet verified**.

## 1. Get a GPU desktop

1. Open **https://dh-ood.hpc.msoe.edu/** and sign in with your MSOE account.
2. Open **Interactive Apps** and select the Rosie desktop app.
3. Request **one GPU** and the time you need. For the team demo, request **3 hours if the form allows it**.
4. Launch, wait for the session to become ready, then connect to the desktop.
5. Open a **terminal inside that desktop**. Run:

```bash
hostname
nvidia-smi
```

You should see a compute-node hostname and a GPU. If you are on `dh-mgmt2`, you are in the management terminal; return to the browser desktop and open its terminal. Do not install NVIDIA drivers there.

## 2. Check your files

**The paths below describe Andrew's working setup.** `~` means the logged-in user's home directory. Teammates using their own accounts must first set up their own clone and environments; Andrew's files are not automatically in their home folders. Use the starter's `SETUP.md` for initial installation.

| Item | Path in Andrew's account |
| --- | --- |
| Team repository | `~/innovation-lab/team-repo` |
| Simulator source we develop | `~/innovation-lab/team-repo/fanuc_er4ia_sim` |
| Isaac Python | `~/venvs/isaac-check/bin/python` |
| Host Python for tests/textures | `~/innovation-lab/fanuc_er4ia_sim/.venv/bin/python` |
| Previously exported scene | `~/innovation-lab/fanuc_er4ia_sim/runs/scene.usd` |

The old extracted folder is still used for its host environment and saved scene. Make source-code changes in **team-repo**.

```bash
cd ~/innovation-lab/team-repo
git status
git pull --ff-only
cd fanuc_er4ia_sim
test -x "$HOME/venvs/isaac-check/bin/python" && echo "Isaac Python found"
```

If Git reports conflicting local changes, stop and resolve them; do not discard them to make the pull work.

## 3. Open the scene — working viewer procedure

Run this entire block in the **browser desktop's terminal**:

```bash
"$HOME/venvs/isaac-check/bin/python" -u - <<'PY'
from isaacsim import SimulationApp
app = SimulationApp({"headless": False, "limit_cpu_threads": 2})
print("WINDOW_STARTUP_OK", flush=True)
try:
    while app.is_running():
        app.update()
finally:
    app.close()
PY
```

Keep that terminal open. First startup took roughly 13 minutes in our session while shaders compiled. An unchanged loading percentage alone does not prove a crash. **Do not start a second Isaac instance while waiting.**

When `WINDOW_STARTUP_OK` appears:

1. In Isaac, choose **File → Open**.
2. Open Andrew's existing scene:
   `/home/ad.msoe.edu/biebera/innovation-lab/fanuc_er4ia_sim/runs/scene.usd`
3. Allow the scene assets to load.
4. Select **ER4iA** in the Stage tree, move the mouse over the viewport, and press **F** to frame it. Scroll to zoom; middle-mouse drag to pan.
5. Keep the renderer on **RTX Real-Time** for this walkthrough.

This viewer opens a **saved snapshot**. Pulling new Python code does not update that snapshot. A scene-building script must run again to use code/layout changes.

### Move a joint manually

Open **Tools → Robotics → Joint Inspector** and select the robot if needed. Find the **Target Position** column, scrolling the panel horizontally if necessary.

Click **▶ Play**, let the arm settle, then change J1's target by a small amount, such as **5 degrees**. J1 rotates the base; it does not command a straight X/Y movement. Leave joint limits, stiffness, and damping unchanged. Play starts physics; it does not automatically run a pick-and-place program.

## 4. One-cube pick test — next test, not yet verified

Place `demo_pick.py` beside `cell.py` in the team repository's `fanuc_er4ia_sim` directory. It was provided separately and is not yet confirmed committed to GitHub.

1. Close the viewer **without saving manual joint edits**. Wait for its terminal prompt to return.
2. Run:

```bash
cd ~/innovation-lab/team-repo/fanuc_er4ia_sim
mkdir -p runs
"$HOME/venvs/isaac-check/bin/python" -u demo_pick.py \
  2>&1 | tee runs/demo_pick_console.log
```

The script builds a fresh scene, sets the viewing angle, and starts automatically after initialization and a short delay. **Do not press Play or Stop during the test.**

Watch for `APPROACH → DESCEND → CLOSE → LIFT → CARRY → LOWER → RELEASE`. It checks actual cube movement and prints **PASS** or **FAIL**. The window remains open afterward.

- Report: `runs/demo_pick_<timestamp>/demo_report.json`
- Console log: `runs/demo_pick_console.log`
- Success requires `pickup_verified`, `placement_verified`, and `complete` to be `true`.
- This uses the configured position of one cube. It is a **simulation mechanics test**, not vision-based picking or real-arm validation. No model training or Cosmos installation is needed for this test.

If it fails, share the final traceback and report. Moving the arm without moving the cube does not count as a successful pick.

## 5. Finish and resume later

Close Isaac, then end the desktop session under **My Interactive Sessions** in the Rosie portal. Home-directory files remain; the running desktop and simulation end when the allocation ends.

For code work: edit in VS Code → commit/push your feature branch → pull that same branch on Rosie → rerun the script. Follow `CONTRIBUTING.md`; do not commit environments, generated textures, or `runs/` outputs. Use **Isaac Python for simulation** and **host Python for host tests**.
