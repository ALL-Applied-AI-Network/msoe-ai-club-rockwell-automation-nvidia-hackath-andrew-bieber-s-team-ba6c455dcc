#!/usr/bin/env python3
"""Tinker: the IRIS cell app. Run a sort or a benchmark and watch it, without the terminal.

It only starts the existing scripts (demo_scan_sort_spaced.py, run_benchmark.py)
with the same options you would type, and reads the files they already write.

    python3 tinker.py

Standard library only (Tkinter); run it with the system Python 3 on a desktop
session, not with the Isaac interpreter.
"""
from __future__ import annotations

from datetime import datetime
import json
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
import sys
import time

import bench_summary
import shutil

HERE = Path(__file__).resolve().parent
RUNS = HERE / "runs"
DEFAULT_ISAAC = os.path.expanduser("~/venvs/isaac-check/bin/python")
ONLY_CHOICES = ("all", "screw", "cube", "vial", "cap")
ACTIVE_SECONDS = 180  # a trial log written this recently means a benchmark is running there


# ------------------------------------------------------------- pure helpers (tested)

def sort_command(isaac_python, run_dir, *, seed=None, physics=True, only="all", headless=False,
                 careful=False, force_drop=False, verify="camera"):
    """Same command run_benchmark.py uses for one trial, with the chosen options."""
    cmd = [isaac_python, "-u", str(HERE / "demo_scan_sort_spaced.py"), "--run-dir", str(run_dir)]
    cmd += ["--headless", "--exit-on-finish"] if headless else ["--viewport-fps", "15"]
    if seed is not None:
        cmd += ["--layout-seed", str(seed)]
        if physics:
            cmd += ["--physics-seed", str(seed)]
    if only != "all":
        cmd += ["--only", only]
    if careful:
        cmd.append("--careful")
    if force_drop:
        cmd.append("--force-drop")
    if verify != "camera":
        cmd += ["--verify", verify]
    return cmd


def bench_command(out_dir, *, trials=50, start_seed=1000, gui=False, retry_crashed=False,
                  isaac_python=DEFAULT_ISAAC):
    cmd = [sys.executable, "-u", str(HERE / "run_benchmark.py"), "--trials", str(trials),
           "--start-seed", str(start_seed), "--out", str(out_dir), "--isaac-python", isaac_python]
    if gui:
        cmd.append("--gui")
    if retry_crashed:
        cmd.append("--retry-crashed")
    return cmd


PROGRESS = [  # (pattern, event kind)
    (re.compile(r"\[scan\] OBJECT (\d+)/(\d+): (\S+)"), "object"),
    (re.compile(r"\[verify\] Camera: (\S+) .*-> (PLACED|NOT PLACED)"), "verify"),
    (re.compile(r"\[recover\] (.*)"), "recover"),
    (re.compile(r"\[sort\] (PASS|FAIL): (.*)"), "result"),
    (re.compile(r"\[bench\] trial (\d+)/(\d+) seed (\d+): (\w+)"), "trial"),
]


def parse_line(line):
    """Return (kind, groups) for a line the progress panel cares about, else None."""
    for pattern, kind in PROGRESS:
        m = pattern.search(line)
        if m:
            return kind, m.groups()
    return None


def line_tag(line):
    low = line.lower()
    if "[sort] pass" in low or ": pass " in low or "-> placed" in low:
        return "good"
    if ("[sort] fail" in low or "not placed" in low or "traceback" in low or "error" in low
            or re.search(r": (fail|crash|timeout)\b", low)):
        return "bad"
    if "[recover]" in low or "warning" in low:
        return "warn"
    return None


def load_results(out_dir):
    path = Path(out_dir) / "results.jsonl"
    rows = []
    if path.is_file():
        for line in path.read_text(errors="replace").splitlines():
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                pass  # a line being written right now
    return rows


def benchmark_activity(out_dir, now=None):
    """Seconds since the newest trial log changed, or None if there are no trials."""
    logs = list(Path(out_dir).glob("trial_*/console.log"))
    if not logs:
        return None
    return (now or time.time()) - max(p.stat().st_mtime for p in logs)


def stats_line(rows):
    if not rows:
        return "No finished trials yet."
    s = bench_summary.summarize(rows)
    low, high = s["pass_rate_95ci"]
    text = (f"{s['passed']}/{s['trials']} full sorts passed ({100*s['pass_rate']:.0f}%, "
            f"95% CI {100*low:.0f}-{100*high:.0f}%)  ·  objects {s['objects_placed']}/{s['objects_expected']}"
            f"  ·  {s['recoveries_total']} recoveries")
    if s["mean_trial_wall_minutes"]:
        text += f"  ·  {s['mean_trial_wall_minutes']:.1f} min/trial"
    return text


def bench_log_path(out_dir):
    """Log inside the folder (started from this app) or beside it (BENCHMARK.md's nohup command)."""
    out_dir = Path(out_dir)
    inside, beside = out_dir / "benchmark.log", out_dir.with_suffix(".log")
    if inside.is_file() or not beside.is_file():
        return inside
    return beside


# ------------------------------------------------------------- physical lab (tested)

PHYSICAL = HERE.parent / "physical-client"
CALENDAR_URL = "https://hackathon.sidclab.org/#calendar"
PHYSICAL_STEPS = {  # step -> (arguments after `uv run`, short description)
    "preflight": (["preflight.py", "--sam", "--require-cuda"], "Local checks: calibration, poses, SAM + GPU"),
    "mock": (["run.py", "--mock"], "Rehearse against a local mock of the cell (no robot)"),
    "prepare": (["prepare_submission.py"], "Write submission.yaml and placement.yaml"),
    "check": (["preflight.py", "--submission"], "Check the prepared request files"),
    "submit": (["run.py", "--submit"], "Send the request to the organizers"),
    "status": (["run.py", "--status"], "Is the request approved? Shows your results link"),
    "slot": (["run.py"], "During your slot: wait for the operator and run each rollout"),
    "download": (["run.py", "--download"], "Download your run's recorded dataset"),
}


def uv_path():
    local = Path.home() / ".local" / "physical-uv" / "bin" / "uv"
    return shutil.which("uv") or (str(local) if local.is_file() else "uv")


def physical_command(step, *, trials=3, contact="", uv=None):
    args, _ = PHYSICAL_STEPS[step]
    cmd = [uv or uv_path(), "run", *args]
    if step == "mock":
        cmd += ["--trials", str(int(trials))]
    if step == "prepare":
        if not contact.strip():
            raise ValueError("enter a contact (name and email) first")
        cmd += ["--contact", contact.strip()]
    return cmd


def physical_env(base=None):
    env = dict(os.environ if base is None else base)
    env["PATH"] = str(Path.home() / ".local" / "physical-uv" / "bin") + os.pathsep + env.get("PATH", "")
    env.setdefault("SORT_SAM_PYTHON", str(Path.home() / "venvs" / "sam2-check" / "bin" / "python"))
    return env


def physical_flags(config=PHYSICAL / "sort_config.py"):
    """PHYSICAL_EXECUTE / PHYSICAL_POSES_CONFIRMED as written in the file (read, never changed)."""
    flags = {}
    try:
        text = Path(config).read_text(errors="replace")
    except OSError:
        return flags
    for name in ("PHYSICAL_EXECUTE", "PHYSICAL_POSES_CONFIRMED"):
        m = re.search(rf"^{name}\s*=\s*(True|False)\b", text, re.M)
        if m:
            flags[name] = m.group(1) == "True"
    return flags


def request_id(path=PHYSICAL / ".request.json"):
    """The submitted request's id only; the secret in that file is never shown."""
    try:
        return json.loads(Path(path).read_text()).get("request_id")
    except (OSError, ValueError, AttributeError):
        return None


CELL_EVENTS = [
    (re.compile(r"request (\S+) approved: run (\S+)"), "approved"),
    (re.compile(r"request (\S+): (\w+)"), "request"),
    (re.compile(r"waiting for the operator"), "waiting"),
    (re.compile(r"trial (\S+) \(layout (\S+)\) running"), "trial"),
    (re.compile(r"trial (\S+) ended"), "trial_end"),
    (re.compile(r"all trials finished"), "done"),
    (re.compile("(\u2713|\u2717) (.*)"), "verdict"),
]


def parse_cell_line(line):
    for pattern, kind in CELL_EVENTS:
        m = pattern.search(line)
        if m:
            return kind, m.groups()
    return None


def pid_file(out_dir):
    return Path(out_dir) / "app_benchmark.json"


# ------------------------------------------------------------------ processes

class Job:
    """A started script whose output goes to a log file, so it outlives the window if asked."""
    def __init__(self, cmd, log_path, env=None, append=False, cwd=HERE):
        self.cmd, self.log_path = cmd, Path(log_path)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        log = open(self.log_path, "a" if append else "w", buffering=1, encoding="utf-8")
        log.write("$ " + " ".join(shlex.quote(c) for c in cmd) + "\n")
        # Scripts print marks like the mock's check mark; a non-UTF-8 locale would crash on them.
        env = dict(os.environ if env is None else env, PYTHONIOENCODING="utf-8")
        kwargs = dict(start_new_session=True) if os.name == "posix" else {}
        self.proc = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, **kwargs)
        log.close()

    def running(self):
        return self.proc.poll() is None

    def stop(self):
        stop_pid(self.proc.pid)


def stop_pid(pid):
    """Ask the whole process group to stop (Ctrl+C), then kill it if it is still there."""
    if os.name != "posix":
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(pid)], capture_output=True)
        return
    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        return

    def later():
        time.sleep(15)
        try:
            os.killpg(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    import threading
    threading.Thread(target=later, daemon=True).start()


def pid_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except (ProcessLookupError, PermissionError, OSError):
        return False


class Tail:
    """Read new lines from a growing log file."""
    def __init__(self, path):
        self.path, self.pos = Path(path), 0

    def read(self):
        try:
            with open(self.path, "r", encoding="utf-8", errors="replace") as f:
                f.seek(self.pos)
                data = f.read()
                self.pos = f.tell()
        except OSError:
            return []
        return data.splitlines()


# ----------------------------------------------------------------------------- UI

C_BG = "#FAFAF7"        # warm off-white
C_SURFACE = "#FFFFFF"
C_INK = "#1F1F1F"       # the robot's dark base
C_MUTED = "#8A8A85"
C_LINE = "#E6E5DF"
C_LOG = "#F4F3EE"
C_YELLOW = "#FFC20E"    # robot yellow
C_YELLOW_DARK = "#E8AE00"
C_YELLOW_SOFT = "#FFF4CC"
C_RED = "#F28B82"       # light red
C_RED_SOFT = "#FDE8E6"
C_RED_INK = "#C0392B"
C_GOOD = "#3C7D22"
C_WARN = "#A86A00"


def robot_face(size):
    """Robot-face logo as (kind, coords, colour) shapes on a size x size square."""
    u = size / 32
    s = lambda *v: [round(x * u, 1) for x in v]
    return [
        ("line", s(16, 2, 16, 7), C_INK),          # antenna
        ("oval", s(13.5, 0.5, 18.5, 5.5), C_RED),  # antenna light
        ("rect", s(2, 13, 4, 21), C_INK),          # ears
        ("rect", s(28, 13, 30, 21), C_INK),
        ("rect", s(4, 7, 28, 29), C_YELLOW),       # head
        ("oval", s(9, 13, 14, 18), C_INK),         # eyes
        ("oval", s(18, 13, 23, 18), C_INK),
        ("rect", s(11, 22, 21, 24), C_INK),        # mouth
    ]


def draw_robot_face(canvas, size):
    for kind, xy, colour in robot_face(size):
        if kind == "line":
            canvas.create_line(*xy, fill=colour, width=max(2, size / 16), capstyle="round")
        elif kind == "oval":
            canvas.create_oval(*xy, fill=colour, outline="")
        else:
            canvas.create_rectangle(*xy, fill=colour, outline="")


def robot_icon(tk, size=32):
    """The same face as a window icon (pixel rectangles; ovals drawn as their boxes)."""
    img = tk.PhotoImage(width=size, height=size)
    for kind, xy, colour in robot_face(size):
        x1, y1, x2, y2 = (int(round(v)) for v in xy)
        if kind == "line":
            x1, x2 = x1 - 1, x1 + 1
        img.put(colour, to=(x1, y1, max(x2, x1 + 1), max(y2, y1 + 1)))
    return img


def apply_theme(root, ttk):
    """Minimal flat look: off-white, robot yellow for actions, light red for stop and caution."""
    style = ttk.Style(root)
    style.theme_use("clam")
    base = ("Segoe UI", 10) if sys.platform == "win32" else ("TkDefaultFont", 10)
    family = base[0]
    style.configure(".", background=C_BG, foreground=C_INK, font=base, bordercolor=C_LINE,
                    lightcolor=C_BG, darkcolor=C_BG, troughcolor=C_LINE, focuscolor=C_YELLOW,
                    selectbackground=C_YELLOW_SOFT, selectforeground=C_INK, fieldbackground=C_SURFACE)
    style.configure("TFrame", background=C_BG)
    style.configure("TLabel", background=C_BG, foreground=C_INK)
    style.configure("Title.TLabel", font=(family, 20, "bold"))
    style.configure("Big.TLabel", font=(family, 12))
    style.configure("Muted.TLabel", foreground=C_MUTED)
    style.configure("TLabelframe", background=C_BG, bordercolor=C_LINE, relief="solid", borderwidth=1)
    style.configure("TLabelframe.Label", background=C_BG, foreground=C_INK, font=(family, 10, "bold"))
    style.configure("TButton", background=C_SURFACE, foreground=C_INK, bordercolor=C_LINE,
                    relief="flat", padding=(12, 6), focusthickness=0)
    style.map("TButton", background=[("disabled", C_BG), ("active", C_LOG)],
              foreground=[("disabled", "#B8B7B0")], bordercolor=[("active", C_INK)])
    style.configure("Go.TButton", background=C_YELLOW, bordercolor=C_YELLOW, font=(family, 11, "bold"))
    style.map("Go.TButton", background=[("disabled", C_YELLOW_SOFT), ("active", C_YELLOW_DARK)],
              bordercolor=[("active", C_YELLOW_DARK)])
    style.configure("Stop.TButton", background=C_RED_SOFT, bordercolor=C_RED, foreground=C_RED_INK)
    style.map("Stop.TButton", background=[("disabled", C_BG), ("active", C_RED)],
              foreground=[("disabled", "#B8B7B0"), ("active", C_INK)], bordercolor=[("disabled", C_LINE)])
    style.configure("TNotebook", background=C_BG, borderwidth=0, tabmargins=(0, 0, 0, 0))
    style.configure("TNotebook.Tab", background=C_BG, foreground=C_MUTED, padding=(16, 8),
                    borderwidth=0, font=(family, 11))
    style.map("TNotebook.Tab", background=[("selected", C_YELLOW), ("active", C_YELLOW_SOFT)],
              foreground=[("selected", C_INK), ("active", C_INK)])
    style.configure("TEntry", fieldbackground=C_SURFACE, bordercolor=C_LINE, padding=4)
    style.configure("TSpinbox", fieldbackground=C_SURFACE, bordercolor=C_LINE, arrowsize=12)
    style.configure("TCombobox", fieldbackground=C_SURFACE, bordercolor=C_LINE, arrowcolor=C_INK)
    style.map("TCombobox", fieldbackground=[("readonly", C_SURFACE)], background=[("readonly", C_SURFACE)],
              selectbackground=[("readonly", C_SURFACE)], selectforeground=[("readonly", C_INK)])
    style.configure("TCheckbutton", background=C_BG, indicatorbackground=C_SURFACE)
    style.map("TCheckbutton", indicatorbackground=[("selected", C_YELLOW)], background=[("active", C_BG)])
    style.configure("TRadiobutton", background=C_BG, indicatorbackground=C_SURFACE)
    style.map("TRadiobutton", indicatorbackground=[("selected", C_YELLOW)], background=[("active", C_BG)])
    style.configure("Horizontal.TProgressbar", background=C_YELLOW, troughcolor=C_LINE,
                    bordercolor=C_LINE, lightcolor=C_YELLOW, darkcolor=C_YELLOW, thickness=8)
    style.configure("Treeview", background=C_SURFACE, fieldbackground=C_SURFACE, bordercolor=C_LINE,
                    rowheight=26)
    style.configure("Treeview.Heading", background=C_BG, foreground=C_MUTED, relief="flat",
                    font=(family, 9, "bold"))
    style.map("Treeview", background=[("selected", C_YELLOW_SOFT)], foreground=[("selected", C_INK)])
    style.configure("Vertical.TScrollbar", background=C_LINE, troughcolor=C_BG, bordercolor=C_BG,
                    arrowcolor=C_MUTED)
    style.configure("Horizontal.TScrollbar", background=C_LINE, troughcolor=C_BG, bordercolor=C_BG,
                    arrowcolor=C_MUTED)


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=RUNS / "benchmark", help="benchmark results folder to watch")
    args = parser.parse_args()
    import tkinter as tk
    from tkinter import ttk, messagebox, filedialog

    root = tk.Tk()
    root.title("Tinker")
    root.geometry("1100x920")
    root.minsize(860, 600)
    root.configure(background=C_BG)
    root.iconphoto(True, robot_icon(tk))
    apply_theme(root, ttk)

    def log_widget(parent):
        frame = ttk.Frame(parent)
        text = tk.Text(frame, wrap="none", height=12, font=("TkFixedFont", 10), relief="flat",
                       borderwidth=0, padx=10, pady=8, background=C_LOG, foreground=C_INK,
                       insertbackground=C_INK, highlightthickness=1, highlightbackground=C_LINE)
        ys = ttk.Scrollbar(frame, orient="vertical", command=text.yview)
        xs = ttk.Scrollbar(frame, orient="horizontal", command=text.xview)
        text.configure(yscrollcommand=ys.set, xscrollcommand=xs.set, state="disabled")
        text.tag_configure("good", foreground=C_GOOD)
        text.tag_configure("bad", foreground=C_RED_INK)
        text.tag_configure("warn", foreground=C_WARN)
        text.grid(row=0, column=0, sticky="nsew")
        ys.grid(row=0, column=1, sticky="ns")
        xs.grid(row=1, column=0, sticky="ew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)
        return frame, text

    def append(text, lines, limit=5000):
        if not lines:
            return
        at_end = text.yview()[1] > 0.98
        text.configure(state="normal")
        for line in lines:
            text.insert("end", line + "\n", line_tag(line) or ())
        extra = int(text.index("end-1c").split(".")[0]) - limit
        if extra > 0:
            text.delete("1.0", f"{extra}.0")
        text.configure(state="disabled")
        if at_end:
            text.see("end")

    def clear(text):
        text.configure(state="normal")
        text.delete("1.0", "end")
        text.configure(state="disabled")

    def show_text(title, body):
        win = tk.Toplevel(root)
        win.title(title)
        win.configure(background=C_BG)
        win.geometry("900x600")
        frame, text = log_widget(win)
        frame.pack(fill="both", expand=True)
        append(text, body.splitlines(), limit=100000)
        text.see("1.0")

    header = ttk.Frame(root, padding=(18, 14, 18, 10))
    header.pack(fill="x")
    logo = tk.Canvas(header, width=44, height=44, highlightthickness=0, background=C_BG)
    draw_robot_face(logo, 44)
    logo.pack(side="left")
    ttk.Label(header, text="Tinker", style="Title.TLabel").pack(side="left", padx=(10, 0))
    ttk.Label(header, text="IRIS sorting cell", style="Muted.TLabel").pack(side="left", padx=(10, 0), pady=(6, 0))
    isaac_var = tk.StringVar(value=DEFAULT_ISAAC)
    ttk.Entry(header, textvariable=isaac_var, width=42).pack(side="right")
    ttk.Label(header, text="Isaac Python  ", style="Muted.TLabel").pack(side="right")
    tk.Frame(root, height=3, background=C_YELLOW).pack(fill="x")

    tabs = ttk.Notebook(root)
    tabs.pack(fill="both", expand=True, padx=14, pady=(10, 14))

    # ------------------------------------------------------------- sort tab
    sort_tab = ttk.Frame(tabs, padding=10)
    tabs.add(sort_tab, text="  Run a sort  ")
    opts = ttk.LabelFrame(sort_tab, text="Options", padding=10)
    opts.pack(fill="x")
    layout_var = tk.StringVar(value="fixed")
    seed_var = tk.IntVar(value=1000)
    physics_var = tk.BooleanVar(value=True)
    only_var = tk.StringVar(value="all")
    headless_var = tk.BooleanVar(value=False)
    careful_var = tk.BooleanVar(value=False)
    drop_var = tk.BooleanVar(value=False)

    ttk.Label(opts, text="Layout").grid(row=0, column=0, sticky="w")
    ttk.Radiobutton(opts, text="Fixed demo layout", variable=layout_var, value="fixed").grid(row=0, column=1, sticky="w")
    ttk.Radiobutton(opts, text="Random, seed", variable=layout_var, value="seed").grid(row=0, column=2, sticky="w")
    ttk.Spinbox(opts, from_=0, to=999999, textvariable=seed_var, width=8).grid(row=0, column=3, sticky="w")
    ttk.Checkbutton(opts, text="Vary friction and mass (same seed)", variable=physics_var).grid(row=0, column=4, sticky="w", padx=(10, 0))
    ttk.Label(opts, text="Objects").grid(row=1, column=0, sticky="w", pady=(8, 0))
    ttk.Combobox(opts, textvariable=only_var, values=ONLY_CHOICES, state="readonly", width=10).grid(row=1, column=1, sticky="w", pady=(8, 0))
    ttk.Checkbutton(opts, text="Headless (no viewport)", variable=headless_var).grid(row=1, column=2, sticky="w", pady=(8, 0))
    ttk.Checkbutton(opts, text="Careful (slower) timing", variable=careful_var).grid(row=1, column=3, columnspan=2, sticky="w", pady=(8, 0))
    ttk.Checkbutton(opts, text="Demo recovery: drop the first object mid-carry", variable=drop_var).grid(row=2, column=1, columnspan=3, sticky="w", pady=(8, 0))
    replay_hint = ttk.Label(opts, text="Tip: to replay a benchmark trial, choose Random and use its seed.", style="Muted.TLabel")
    replay_hint.grid(row=3, column=0, columnspan=5, sticky="w", pady=(8, 0))

    bar = ttk.Frame(sort_tab, padding=(0, 10))
    bar.pack(fill="x")
    start_sort = ttk.Button(bar, text="Start sort", style="Go.TButton")
    stop_sort = ttk.Button(bar, text="Stop", state="disabled", style="Stop.TButton")
    open_dir = ttk.Button(bar, text="Open run folder", state="disabled")
    start_sort.pack(side="left")
    stop_sort.pack(side="left", padx=6)
    open_dir.pack(side="left")
    sort_status = ttk.Label(bar, text="Idle", style="Big.TLabel")
    sort_status.pack(side="left", padx=16)
    sort_progress = ttk.Progressbar(sort_tab, maximum=1, value=0)
    sort_progress.pack(fill="x")
    counts = ttk.Label(sort_tab, text="", padding=(0, 6))
    counts.pack(fill="x")
    sort_log_frame, sort_log = log_widget(sort_tab)
    sort_log_frame.pack(fill="both", expand=True)

    state = {"sort": None, "sort_tail": None, "sort_dir": None, "placed": 0, "missed": 0, "recoveries": 0}

    def do_start_sort():
        isaac = isaac_var.get().strip()
        if not Path(isaac).is_file():
            messagebox.showerror("Isaac Python not found", f"No file at:\n{isaac}\n\nSet the path at the top right.")
            return
        run_dir = RUNS / ("app_sort_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
        seed = seed_var.get() if layout_var.get() == "seed" else None
        cmd = sort_command(isaac, run_dir, seed=seed, physics=physics_var.get(), only=only_var.get(),
                           headless=headless_var.get(), careful=careful_var.get(), force_drop=drop_var.get())
        env = dict(os.environ, FINGER_SET=os.environ.get("FINGER_SET", "dual_v2"))
        clear(sort_log)
        state.update(sort=Job(cmd, run_dir / "app_console.log", env), sort_dir=run_dir,
                     placed=0, missed=0, recoveries=0)
        state["sort_tail"] = Tail(run_dir / "app_console.log")
        sort_progress.configure(maximum=1, value=0)
        sort_status.configure(text="Starting Isaac Sim (about a minute)…")
        counts.configure(text="")
        start_sort.configure(state="disabled")
        stop_sort.configure(state="normal")
        open_dir.configure(state="normal")

    def do_stop_sort():
        if state["sort"] and state["sort"].running():
            state["sort"].stop()
            sort_status.configure(text="Stopping…")

    def open_folder(path):
        opener = "xdg-open" if sys.platform.startswith("linux") else ("open" if sys.platform == "darwin" else "explorer")
        try:
            subprocess.Popen([opener, str(path)])
        except OSError:
            messagebox.showinfo("Run folder", str(path))

    start_sort.configure(command=do_start_sort)
    stop_sort.configure(command=do_stop_sort)
    open_dir.configure(command=lambda: state["sort_dir"] and open_folder(state["sort_dir"]))

    def poll_sort():
        job, tail = state["sort"], state["sort_tail"]
        if tail:
            lines = tail.read()
            append(sort_log, lines)
            for line in lines:
                event = parse_line(line)
                if not event:
                    continue
                kind, g = event
                if kind == "object":
                    sort_progress.configure(maximum=int(g[1]), value=int(g[0]) - 1)
                    sort_status.configure(text=f"Object {g[0]} of {g[1]}: {g[2]}")
                elif kind == "verify":
                    state["placed" if g[1] == "PLACED" else "missed"] += 1
                elif kind == "recover":
                    state["recoveries"] += 1
                elif kind == "result":
                    sort_progress.configure(value=sort_progress["maximum"])
                    sort_status.configure(text=f"{g[0]}: {g[1]}")
                counts.configure(text=f"Camera checks: {state['placed']} placed, {state['missed']} not placed"
                                      f"  ·  recovery messages: {state['recoveries']}")
        if job and not job.running():
            code = job.proc.returncode
            report = state["sort_dir"] / "sort_report.json"
            if not report.is_file() and "PASS" not in sort_status.cget("text") and "FAIL" not in sort_status.cget("text"):
                sort_status.configure(text=f"Stopped (exit code {code}); no report written")
            state["sort"] = None
            start_sort.configure(state="normal")
            stop_sort.configure(state="disabled")
        root.after(400, poll_sort)

    # -------------------------------------------------------- benchmark tab
    bench_tab = ttk.Frame(tabs, padding=10)
    tabs.add(bench_tab, text="  Benchmark  ")
    top = ttk.Frame(bench_tab)
    top.pack(fill="x")
    out_var = tk.StringVar(value=str(args.out.expanduser().resolve()))
    ttk.Label(top, text="Results folder").pack(side="left")
    ttk.Entry(top, textvariable=out_var, width=60).pack(side="left", padx=6)
    ttk.Button(top, text="Browse…", command=lambda: out_var.set(
        filedialog.askdirectory(initialdir=out_var.get()) or out_var.get())).pack(side="left")

    stats = ttk.Label(bench_tab, text="", style="Big.TLabel", padding=(0, 10))
    stats.pack(fill="x")
    activity = ttk.Label(bench_tab, text="", style="Muted.TLabel")
    activity.pack(fill="x")

    columns = ("seed", "status", "placed", "recoveries", "minutes", "failed")
    table_frame = ttk.Frame(bench_tab)
    table_frame.pack(fill="x", pady=(8, 4))
    table = ttk.Treeview(table_frame, columns=columns, show="headings", height=7)
    for col, width in zip(columns, (70, 80, 80, 115, 80, 425)):
        table.heading(col, text=col.capitalize())
        table.column(col, width=width, anchor="w", stretch=col == "failed")
    table.tag_configure("pass", foreground=C_GOOD)
    table.tag_configure("problem", foreground=C_RED_INK)
    tys = ttk.Scrollbar(table_frame, orient="vertical", command=table.yview)
    table.configure(yscrollcommand=tys.set)
    table.pack(side="left", fill="both", expand=True)
    tys.pack(side="right", fill="y")
    ttk.Label(bench_tab, text="Double-click a trial to read its console log.", style="Muted.TLabel").pack(anchor="w")

    run_box = ttk.LabelFrame(bench_tab, text="Start a benchmark", padding=10)
    run_box.pack(fill="x", pady=(8, 0))
    trials_var, start_seed_var = tk.IntVar(value=50), tk.IntVar(value=1000)
    gui_var, retry_var = tk.BooleanVar(value=True), tk.BooleanVar(value=False)
    row1, row2 = ttk.Frame(run_box), ttk.Frame(run_box)
    row1.pack(fill="x")
    row2.pack(fill="x", pady=(8, 0))
    ttk.Label(row1, text="Trials").pack(side="left")
    ttk.Spinbox(row1, from_=1, to=1000, textvariable=trials_var, width=6).pack(side="left", padx=(4, 12))
    ttk.Label(row1, text="First seed").pack(side="left")
    ttk.Spinbox(row1, from_=0, to=999999, textvariable=start_seed_var, width=8).pack(side="left", padx=(4, 12))
    ttk.Checkbutton(row1, text="Show viewport (10 fps)", variable=gui_var).pack(side="left")
    ttk.Checkbutton(row1, text="Retry crashed seeds", variable=retry_var).pack(side="left", padx=8)
    start_bench = ttk.Button(row2, text="Start benchmark", style="Go.TButton")
    stop_bench = ttk.Button(row2, text="Stop benchmark", state="disabled", style="Stop.TButton")
    summary_btn = ttk.Button(row2, text="Full summary")
    start_bench.pack(side="left")
    stop_bench.pack(side="left", padx=6)
    summary_btn.pack(side="left")
    bench_log_frame, bench_log = log_widget(bench_tab)
    bench_log.configure(height=7)
    bench_log_frame.pack(fill="both", expand=True, pady=(8, 0))

    bstate = {"out": None, "tail": None, "seen": -1}

    def out_dir():
        return Path(out_var.get()).expanduser()

    def own_benchmark():
        """The benchmark this app started on this machine, if it is still running."""
        try:
            info = json.loads(pid_file(out_dir()).read_text())
        except (OSError, ValueError):
            return None
        if info.get("host") == socket.gethostname() and pid_alive(info.get("pid", -1)):
            return info
        return None

    def do_start_bench():
        out = out_dir()
        idle = benchmark_activity(out)
        if idle is not None and idle < ACTIVE_SECONDS:
            messagebox.showwarning("Benchmark already running",
                f"A trial in\n{out}\nwrote output {idle:.0f} s ago, so a benchmark is probably running "
                "into this folder (maybe on another node).\n\nTwo runners in one folder would repeat seeds. "
                "Choose a different results folder.")
            return
        isaac = isaac_var.get().strip()
        if not Path(isaac).is_file():
            messagebox.showerror("Isaac Python not found", f"No file at:\n{isaac}")
            return
        cmd = bench_command(out, trials=trials_var.get(), start_seed=start_seed_var.get(),
                            gui=gui_var.get(), retry_crashed=retry_var.get(), isaac_python=isaac)
        job = Job(cmd, bench_log_path(out), dict(os.environ), append=True)
        pid_file(out).write_text(json.dumps({"pid": job.proc.pid, "host": socket.gethostname(),
                                             "started": datetime.now().isoformat(timespec="seconds")}))
        bstate.update(out=None)  # re-attach the log tail
        messagebox.showinfo("Benchmark started",
            "The benchmark keeps running if you close this window.\nReopen the app to watch or stop it.")

    def do_stop_bench():
        info = own_benchmark()
        if info and messagebox.askyesno("Stop benchmark",
                "Stop after killing the current trial? Finished trials are kept; "
                "starting again with the same folder resumes where it stopped."):
            stop_pid(info["pid"])

    def do_summary():
        rows = load_results(out_dir())
        if not rows:
            messagebox.showinfo("Summary", "No finished trials in this folder yet.")
            return
        show_text("Benchmark summary", bench_summary.markdown(bench_summary.summarize(rows)))

    def open_trial(_event):
        sel = table.selection()
        if not sel:
            return
        log = out_dir() / f"trial_{int(sel[0]):05d}" / "console.log"
        try:
            lines = log.read_text(errors="replace").splitlines()
        except OSError:
            messagebox.showinfo("Trial log", f"No log at\n{log}")
            return
        show_text(f"Trial {sel[0]} (last 2000 lines)", "\n".join(lines[-2000:]))

    start_bench.configure(command=do_start_bench)
    stop_bench.configure(command=do_stop_bench)
    summary_btn.configure(command=do_summary)
    table.bind("<Double-1>", open_trial)

    def refresh_bench():
        out = out_dir()
        if bstate["out"] != out:
            bstate.update(out=out, tail=Tail(bench_log_path(out)), seen=-1)
            clear(bench_log)
            table.delete(*table.get_children())
        append(bench_log, bstate["tail"].read())
        rows = load_results(out)
        if len(rows) != bstate["seen"]:
            bstate["seen"] = len(rows)
            table.delete(*table.get_children())
            for r in rows:
                failed = ", ".join(r.get("failed_objects") or []) or (r.get("error") or "")[:120]
                table.insert("", "end", iid=str(r["seed"]), tags=("pass" if r["status"] == "pass" else "problem",),
                             values=(r["seed"], r["status"].upper(), f"{r.get('placed') if r.get('placed') is not None else '-'}/{r.get('expected') or '-'}",
                                     len(r.get("recoveries") or []), f"{r['wall_s']/60:.1f}", failed))
            if rows:
                table.see(str(rows[-1]["seed"]))
            stats.configure(text=stats_line(rows))
        idle = benchmark_activity(out) if out.is_dir() else None
        mine = own_benchmark()
        if idle is None:
            activity.configure(text="No trials in this folder yet.")
        elif idle < ACTIVE_SECONDS:
            where = "started from this app" if mine else "started from a terminal or another node"
            activity.configure(text=f"Running: a trial wrote output {idle:.0f} s ago; {where}.")
        else:
            activity.configure(text=f"Not running: last trial output {idle/60:.0f} min ago.")
        stop_bench.configure(state="normal" if mine else "disabled")
        root.after(3000, refresh_bench)

    # ------------------------------------------------------- physical lab tab
    import webbrowser
    lab_tab = ttk.Frame(tabs, padding=10)
    tabs.add(lab_tab, text="  Physical lab  ")
    safety = tk.Label(lab_tab, anchor="w", justify="left", padx=12, pady=10, wraplength=1000, font=("TkDefaultFont", 11, "bold"))
    safety.pack(fill="x")
    req_label = ttk.Label(lab_tab, text="", style="Big.TLabel", padding=(0, 6, 0, 0))
    req_label.pack(fill="x")
    slot_label = ttk.Label(lab_tab, text="", padding=(0, 2, 0, 6))
    slot_label.pack(fill="x")

    steps = ttk.Frame(lab_tab)
    steps.pack(fill="x")
    for i in range(3):
        steps.columnconfigure(i, weight=1, uniform="step")
    trials_mock = tk.IntVar(value=3)
    contact_var = tk.StringVar(value="")
    lab = {"job": None, "tail": None, "step": None, "link": None}
    lab_buttons = []

    def step_box(col, title, hint):
        box = ttk.LabelFrame(steps, text=title, padding=8)
        box.grid(row=0, column=col, sticky="nsew", padx=(0 if col == 0 else 6, 0))
        ttk.Label(box, text=hint, style="Muted.TLabel", wraplength=300).pack(anchor="w", pady=(0, 6))
        return box

    def lab_button(parent, text, step, confirm=None, style="TButton"):
        b = ttk.Button(parent, text=text, command=lambda: run_lab(step, confirm), style=style)
        b.pack(fill="x", pady=2)
        lab_buttons.append(b)
        return b

    b1 = step_box(0, "1. Check (no robot)", "Run these first, any time.")
    lab_button(b1, "Preflight checks", "preflight")
    mock_row = ttk.Frame(b1)
    mock_row.pack(fill="x", pady=2)
    ttk.Label(mock_row, text="Mock rollouts").pack(side="left")
    ttk.Spinbox(mock_row, from_=1, to=20, textvariable=trials_mock, width=4).pack(side="left", padx=6)
    lab_button(b1, "Mock rehearsal", "mock")

    b2 = step_box(1, "2. Request a slot", "Contact for the organizers (name, email):")
    ttk.Entry(b2, textvariable=contact_var).pack(fill="x", pady=(0, 4))
    lab_button(b2, "Prepare request files", "prepare")
    lab_button(b2, "Check request files", "check")
    lab_button(b2, "Submit request...", "submit", confirm=(
        "Submit to the organizers?",
        "This sends physical-client/submission/submission.yaml (and placement.yaml) to "
        "hackathon.sidclab.org for approval. Continue?"), style="Stop.TButton")
    lab_button(b2, "Check status", "status")
    ttk.Button(b2, text="Open booking calendar", command=lambda: webbrowser.open(CALENDAR_URL)).pack(fill="x", pady=2)

    b3 = step_box(2, "3. Your slot", "Start this at the beginning of your booked slot and leave it running.")
    lab_button(b3, "Start slot run...", "slot", confirm="slot", style="Stop.TButton")
    lab_button(b3, "Download recording", "download")
    results_btn = ttk.Button(b3, text="Open results page", state="disabled",
                             command=lambda: lab["link"] and webbrowser.open(lab["link"]))
    results_btn.pack(fill="x", pady=2)

    lab_bar = ttk.Frame(lab_tab, padding=(0, 8))
    lab_bar.pack(fill="x")
    lab_stop = ttk.Button(lab_bar, text="Stop", state="disabled", style="Stop.TButton")
    lab_stop.pack(side="left")
    lab_status = ttk.Label(lab_bar, text="Idle", style="Big.TLabel")
    lab_status.pack(side="left", padx=12)
    lab_log_frame, lab_log = log_widget(lab_tab)
    lab_log_frame.pack(fill="both", expand=True)

    def motion_on():
        flags = physical_flags()
        return bool(flags.get("PHYSICAL_EXECUTE") and flags.get("PHYSICAL_POSES_CONFIRMED"))

    def refresh_lab_header():
        if not PHYSICAL.is_dir():
            safety.configure(text=f"physical-client folder not found at {PHYSICAL}", bg=C_RED_SOFT, fg=C_RED_INK)
        elif motion_on():
            safety.configure(text="Robot motion is ON: the slot run will move the real arm "
                                  "(PHYSICAL_EXECUTE and PHYSICAL_POSES_CONFIRMED are True).",
                             bg=C_RED_SOFT, fg=C_RED_INK)
        else:
            safety.configure(text="Robot motion is OFF: preview only, the arm holds its pose. "
                                  "Enable it by hand in physical-client/sort_config.py after the README checks.",
                             bg=C_YELLOW_SOFT, fg=C_INK)
        rid = request_id()
        req_label.configure(text=f"Request: {rid}" if rid else "Request: none submitted yet")
        root.after(5000, refresh_lab_header)

    def run_lab(step, confirm=None):
        if lab["job"] and lab["job"].running():
            messagebox.showinfo("Busy", "Another physical-lab command is still running. Stop it first.")
            return
        if confirm == "slot":
            confirm = ("Start the slot run?",
                       "This connects to the real cell and runs your approved submission each time the "
                       "operator starts a rollout.\n\nRobot motion is "
                       + ("ON: the arm will move." if motion_on() else "OFF: preview only.")
                       + "\n\nOnly start it during your booked slot. Continue?")
        if confirm and not messagebox.askyesno(*confirm):
            return
        try:
            cmd = physical_command(step, trials=trials_mock.get(), contact=contact_var.get())
        except ValueError as e:
            messagebox.showerror("Missing input", str(e))
            return
        log = RUNS / "physical" / f"{step}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        try:
            lab["job"] = Job(cmd, log, physical_env(), cwd=PHYSICAL)
        except OSError as e:
            messagebox.showerror("Could not start", f"{e}\n\nIs uv installed (README: setup_rosie.sh)?")
            return
        lab.update(tail=Tail(log), step=step)
        clear(lab_log)
        lab_status.configure(text=f"Running: {PHYSICAL_STEPS[step][1]}")
        slot_label.configure(text="")
        lab_stop.configure(state="normal")
        for b in lab_buttons:
            b.configure(state="disabled")

    def stop_lab():
        if lab["job"] and lab["job"].running():
            lab["job"].stop()
            lab_status.configure(text="Stopping...")
    lab_stop.configure(command=stop_lab)

    def poll_lab():
        if lab["tail"]:
            lines = lab["tail"].read()
            append(lab_log, lines)
            for line in lines:
                link = re.search(r"https?://\S+#key=\S+", line)
                if link:
                    lab["link"] = link.group(0)
                    results_btn.configure(state="normal")
                event = parse_cell_line(line)
                if not event:
                    continue
                kind, g = event
                if kind == "request":
                    slot_label.configure(text=f"Request {g[0]} is {g[1]}.")
                elif kind == "waiting":
                    slot_label.configure(text="Waiting for the operator to approve and start...")
                elif kind == "approved":
                    slot_label.configure(text=f"Approved: run {g[1]}. Waiting for the operator to start a rollout.")
                elif kind == "trial":
                    slot_label.configure(text=f"Rollout {g[0]} (layout {g[1]}) running")
                elif kind == "trial_end":
                    slot_label.configure(text=f"Rollout {g[0]} ended; waiting for the next one")
                elif kind == "done":
                    slot_label.configure(text="All rollouts finished.")
                elif kind == "verdict":
                    slot_label.configure(text=f"{g[0]} {g[1][:140]}")
        job = lab["job"]
        if job and not job.running():
            code = job.proc.returncode
            lab_status.configure(text=f"Finished: {PHYSICAL_STEPS[lab['step']][1]} "
                                      + ("(OK)" if code == 0 else f"(exit code {code})"))
            lab["job"] = None
            lab_stop.configure(state="disabled")
            for b in lab_buttons:
                b.configure(state="normal")
        root.after(400, poll_lab)

    def on_close():
        if state["sort"] and state["sort"].running():
            if not messagebox.askyesno("Sort still running", "Closing stops the running sort. Close anyway?"):
                return
            state["sort"].stop()
        if lab["job"] and lab["job"].running():
            if messagebox.askyesno("Physical-lab command running",
                                   "Stop it too? Choose No to leave it running in the background "
                                   f"(its log is in {RUNS / 'physical'})."):
                lab["job"].stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    poll_sort()
    refresh_bench()
    refresh_lab_header()
    poll_lab()
    root.mainloop()


if __name__ == "__main__":
    main()
