#!/usr/bin/env python3
"""Compose the README GIFs from recorded frames (scripts/record_gifs.sh calls this).

    compose.py hero --frames hero/ --out docs/assets/hero.gif
    compose.py keep_upright --urdf u.urdf --none frames/q5_none --upright frames/q5_orientation \
        --out docs/assets/keep_upright.gif
    compose.py direct_vs_moveit --live frames/live --csv direct.csv --queries q.json --query 5 \
        --out docs/assets/direct_vs_moveit.gif
    compose.py check-live --live frames/live --csv direct.csv --queries q.json --query 5

Every number drawn on a frame is computed here from recorded data: the tool's tilt from the joint
positions recorded with each frame (forward kinematics from the URDF), the planning times from
the CSV the planning run wrote. Text is burned in with Pillow, the GIF is encoded by ffmpeg
(two-pass palette). Prints one JSON line per GIF with what README captions need.
"""

import argparse
import csv
import json
import math
import pathlib
import shutil
import subprocess
import sys

import numpy as np
from PIL import Image, ImageChops, ImageDraw, ImageFont
from urdf_fk import UrdfFk, rpy_matrix, tilt_deg

WIDTH = 800
FPS = 10
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
BAND = (24, 26, 30)
GOAL_TOL = 0.05  # rad on every joint: "at the query start" / "at its goal"


def speed_phrase(f):
    """f = GIF playback speed / planned trajectory speed."""
    if 0.8 <= f <= 1.25:
        return f"shown at about the planned speed ({f:.2f}x)"
    if f < 1.0:
        return f"shown {1 / f:.1f} times slower than planned"
    return f"shown {f:.1f} times faster than planned"


def font(size, bold=False):
    return ImageFont.truetype(BOLD if bold else FONT, size)


def wrap(text, fnt, width):
    words, lines, line = text.split(), [], ""
    for w in words:
        trial = f"{line} {w}".strip()
        if fnt.getlength(trial) <= width or not line:
            line = trial
        else:
            lines.append(line)
            line = w
    return lines + ([line] if line else [])


def caption_band(text, note=None):
    """Dark band with the caption (and an optional smaller note) for the bottom of every frame."""
    f, fn = font(17), font(13)
    lines = wrap(text, f, WIDTH - 28)
    notes = wrap(note, fn, WIDTH - 28) if note else []
    h = 14 + 23 * len(lines) + (18 * len(notes) + 4 if notes else 0) + 8
    band = Image.new("RGB", (WIDTH, h), BAND)
    d = ImageDraw.Draw(band)
    y = 12
    for line in lines:
        d.text((14, y), line, font=f, fill=(240, 240, 240))
        y += 23
    y += 4
    for line in notes:
        d.text((14, y), line, font=fn, fill=(170, 175, 182))
        y += 18
    return band


def encode(frames, out, colors=128, stats="diff"):
    """frames: list of PIL images (same size) -> GIF at FPS via ffmpeg's palette filters."""
    out = pathlib.Path(out)
    tmp = out.with_suffix(".frames")
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    for i, f in enumerate(frames):
        f.save(tmp / f"f_{i:04d}.png", compress_level=1)
    out.parent.mkdir(parents=True, exist_ok=True)
    for c in (colors, 96, 64, 48):  # fewer colours only if the file breaks the 3 MB limit
        if c > colors:
            continue
        subprocess.run(
            [
                "ffmpeg",
                "-loglevel",
                "error",
                "-y",
                "-framerate",
                str(FPS),
                "-i",
                str(tmp / "f_%04d.png"),
                "-filter_complex",
                f"split[s0][s1];[s0]palettegen=max_colors={c}:stats_mode={stats}[p];"
                "[s1][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle",
                str(out),
            ],
            check=True,
        )
        if out.stat().st_size <= 3_000_000:
            break
    shutil.rmtree(tmp)
    return c


def report(name, out, frames, colors, **extra):
    info = {
        "gif": str(out),
        "bytes": pathlib.Path(out).stat().st_size,
        "frames": len(frames),
        "fps": FPS,
        "duration_s": len(frames) / FPS,
        "width": frames[0].width,
        "height": frames[0].height,
        "palette_colors": colors,
        **extra,
    }
    print(f"[compose] {name} {json.dumps(info)}", flush=True)


# --- hero -------------------------------------------------------------------------------------

HERO_CAPTION = (
    "the arm's only knowledge of the shelf is this map; the path avoids boards it has never been"
    " told about"
)
HERO_LABELS = {
    "points": "1  the camera measures points on the shelf",
    "map": "2  the points become a 3-D map of occupied 2 cm cubes",
    "path": "3  a path for the tool is planned through the map",
    "motion": "4  the arm follows the planned path",
    "hold": "4  the arm follows the planned path",
}


def hero(args):
    meta = json.loads((pathlib.Path(args.frames) / "frames.json").read_text())
    rate = meta["executed_sim_s"] / meta["motion_gif_s"] / meta["time_scale"]
    note = (
        f"Simulated depth camera, map and plan from one run (query {meta['query']}); the arm's"
        f" motion is its recorded execution in the simulator, {speed_phrase(rate)}."
    )
    band = caption_band(HERO_CAPTION, note)
    frames = []
    for i, fm in enumerate(meta["frames"]):
        img = Image.open(pathlib.Path(args.frames) / f"frame_{i:04d}.png").convert("RGB")
        img = img.resize((WIDTH, round(img.height * WIDTH / img.width)), Image.LANCZOS)
        canvas = Image.new("RGB", (WIDTH, 40 + img.height + band.height), BAND)
        ImageDraw.Draw(canvas).text(
            (14, 10), HERO_LABELS[fm["phase"]], font=font(17, True), fill=(255, 255, 255)
        )
        canvas.paste(img, (0, 40))
        canvas.paste(band, (0, 40 + img.height))
        frames.append(canvas)
    colors = encode(frames, args.out, stats="full")
    report(
        "hero",
        args.out,
        frames,
        colors,
        caption=HERO_CAPTION,
        note=note,
        query=meta["query"],
        motion_speed_vs_planned=round(rate, 3),
    )


# --- keep upright -----------------------------------------------------------------------------


def project(camera, pts):
    """Pinhole projection into the Gazebo demo camera (x forward, y left, z up)."""
    x, y, z, r, p, yaw = camera["pose_xyz_rpy"]
    xc = (np.atleast_2d(pts) - np.array([x, y, z])) @ rpy_matrix(r, p, yaw)
    f = (camera["width"] / 2) / math.tan(camera["horizontal_fov"] / 2)
    return np.stack(
        [
            camera["width"] / 2 - f * xc[:, 1] / xc[:, 0],
            camera["height"] / 2 - f * xc[:, 2] / xc[:, 0],
        ],
        axis=1,
    )


def header(width, title, line):
    """Panel header strip (text above the picture, so it never hides the arm)."""
    img = Image.new("RGB", (width, 50), BAND)
    d = ImageDraw.Draw(img)
    d.text((8, 5), title, font=font(16, True), fill=(255, 255, 255))
    d.text((8, 27), line, font=font(15), fill=(255, 214, 90))
    return img


def draw_cup(img, centre, angle_deg):
    """A schematic glass of water at `centre`, tilted by angle_deg (positive leans right), with
    its water surface kept level and a dashed true-vertical line behind it."""
    ss, half = 4, 40  # supersampled patch of 80 x 80 px
    n = 2 * half * ss
    c = n / 2
    patch = Image.new("RGBA", (n, n), (0, 0, 0, 0))
    d = ImageDraw.Draw(patch)
    for y in range(int(c - 30 * ss), int(c + 30 * ss), 8 * ss):
        d.line([(c, y), (c, y + 4 * ss)], fill=(20, 20, 20, 255), width=4 * ss)
        d.line([(c, y), (c, y + 4 * ss)], fill=(250, 250, 250, 255), width=2 * ss)
    a = math.radians(angle_deg)
    local = [(-12, -16), (12, -16), (8, 16), (-8, 16)]
    poly = [
        (c + ss * (x * math.cos(a) - y * math.sin(a)), c + ss * (x * math.sin(a) + y * math.cos(a)))
        for x, y in local
    ]
    cup = Image.new("L", (n, n), 0)
    ImageDraw.Draw(cup).polygon(poly, fill=255)
    level = Image.new("L", (n, n), 0)
    ImageDraw.Draw(level).rectangle([0, c - 3 * ss, n, n], fill=255)
    water = ImageChops.multiply(cup, level)
    patch.paste((70, 160, 255, 255), (0, 0), water)
    glass = Image.new("RGBA", (n, n), (225, 240, 255, 70))
    patch = Image.composite(
        Image.alpha_composite(patch, glass), patch, ImageChops.subtract(cup, water)
    )
    d = ImageDraw.Draw(patch)
    ring = poly + [poly[0]]
    d.line(ring, fill=(20, 20, 20, 255), width=6 * ss, joint="curve")
    d.line(ring, fill=(255, 255, 255, 255), width=3 * ss, joint="curve")
    patch = patch.resize((2 * half, 2 * half), Image.LANCZOS)
    base = img.convert("RGBA")
    base.alpha_composite(patch, (int(centre[0]) - half, int(centre[1]) - half))
    return base.convert("RGB")


def panel_frames(rec_dir):
    meta = json.loads((pathlib.Path(rec_dir) / "frames.json").read_text())
    stamps = np.array([f["stamp"] for f in meta["frames"]])
    return meta, stamps - stamps[0]


KEEP_CAPTION = (
    "same start and goal, planned twice: without a constraint the tool tilts; with the"
    " keep-upright constraint it stays within 5 degrees of vertical"
)


def keep_upright(args):
    fk = UrdfFk(args.urdf)
    panels = []
    for rec, title in ((args.none, "no constraint"), (args.upright, "keep upright")):
        meta, t = panel_frames(rec)
        names = meta["joint_names"]
        poses = [fk.pose("tool0", dict(zip(names, f["q"], strict=True))) for f in meta["frames"]]
        panels.append(
            {
                "dir": pathlib.Path(rec),
                "meta": meta,
                "t": t,
                "poses": poses,
                "title": title,
                "tilt": [tilt_deg(p) for p in poses],
                "side": 1.0,
                "max": 0.0,
            }
        )
    # Fit the longer recording's motion into args.motion_s GIF seconds; both panels share the
    # clock (sim time since each execution was sent).
    t_max = max(p["t"][-1] for p in panels)
    speedup = t_max / args.motion_s
    scale = panels[0]["meta"]["time_scale"]
    n_pre, n_post = round(args.pre_s * FPS), round(args.post_s * FPS)
    n_motion = round(args.motion_s * FPS)
    times = [0.0] * n_pre + [i * speedup / FPS for i in range(n_motion + 1)] + [t_max] * n_post
    gap = 8
    pw = (WIDTH - gap) // 2
    cam = panels[0]["meta"]["camera"]
    ph = round(cam["height"] * pw / cam["width"])
    band = caption_band(
        KEEP_CAPTION,
        "Glass: drawn at the tool and tilted by the tool's tilt, computed from the joint"
        " positions recorded with each frame (dashed line: vertical). Same query,"
        f" {speed_phrase(speedup / scale)}.",
    )
    frames = []
    for tt in times:
        canvas = Image.new("RGB", (WIDTH, 50 + ph + band.height), BAND)
        for k, p in enumerate(panels):
            i = int(np.clip(np.searchsorted(p["t"], tt, side="right") - 1, 0, len(p["t"]) - 1))
            img = Image.open(p["dir"] / p["meta"]["frames"][i]["file"]).convert("RGB")
            img = img.resize((pw, ph), Image.LANCZOS)
            pose = p["poses"][i]
            origin, z_tool = pose[:3, 3], pose[:3, 2]
            # tool0's z axis points down into the shelf; a held glass's up is -z. The glass is
            # drawn 7 cm out along the tool axis and leans to the side the tool's up direction
            # leans in the picture (side kept unless the lean clearly changes side).
            uv = project(cam, [origin + 0.07 * z_tool, origin, origin - 0.1 * z_tool])
            uv *= pw / cam["width"]
            lean = math.degrees(math.atan2(uv[2, 0] - uv[1, 0], uv[1, 1] - uv[2, 1]))
            if abs(lean) > 4.0:
                p["side"] = math.copysign(1.0, lean)
            p["max"] = max(p["max"], p["tilt"][i])
            img = draw_cup(img, uv[0], p["side"] * p["tilt"][i])
            canvas.paste(
                header(
                    pw, p["title"], f"tool tilt {p['tilt'][i]:4.1f}°    max so far {p['max']:4.1f}°"
                ),
                (k * (pw + gap), 0),
            )
            canvas.paste(img, (k * (pw + gap), 50))
        canvas.paste(band, (0, 50 + ph))
        frames.append(canvas)
    colors = encode(frames, args.out)
    report(
        "keep_upright",
        args.out,
        frames,
        colors,
        caption=KEEP_CAPTION,
        query=int(pathlib.Path(args.none).name.split("_")[0][1:]),
        max_tilt_recorded_deg=[round(max(p["tilt"]), 2) for p in panels],
        executed_time_scale=scale,
        gif_speedup_of_recording=round(speedup, 3),
        motion_speed_vs_planned=round(speedup / scale, 3),
        attempts=[p["meta"].get("attempts") for p in panels],
    )


# --- direct vs MoveIt -------------------------------------------------------------------------


def plan_plus_simplify(row):
    """Like-for-like planning time, as scripts/aggregate_direct.py plan_plus_simplify: MoveIt's
    planning_time already is OMPL solve + simplification; pcp_direct records the two apart."""
    if row["pipeline"] == "moveit":
        return float(row["plan_time_s"])
    return float(row["plan_time_s"]) + float(row["simplify_time_s"])


def query_motions(live, csv_path, queries, qid):
    """The two executions of query `qid` in a live recording, by pipeline: (rows, segments).

    Every /joint_states sample is labelled "at start" or "at goal" (all joints within GOAL_TOL
    of the query's start or goal) or neither. An execution runs from the last sample at the start
    to the next sample at the goal, with no sample at either in between (repositioning moves go
    from the goal back to the start, so they never match); stops along the way do not split it.
    In time order, the CSV's `order` column says which pipeline ran first.
    """
    rows = {r["pipeline"]: r for r in csv.DictReader(open(csv_path)) if int(r["query"]) == qid}
    if set(rows) != {"direct", "moveit"}:
        raise RuntimeError(f"{csv_path}: no direct and moveit rows for query {qid}")
    bad = [p for p, r in rows.items() if r["exec_success"] != "1"]
    if bad:
        raise RuntimeError(f"query {qid}: execution failed for {bad}")
    q = next(x for x in json.loads(pathlib.Path(queries).read_text())["queries"] if x["id"] == qid)
    if np.max(np.abs(np.subtract(q["start"], q["goal"]))) <= 2 * GOAL_TOL:
        raise RuntimeError(f"query {qid}: start and goal too close to tell apart")
    js = np.array(live["joint_states"])
    t, pos = js[:, 0], js[:, 1:]
    at_start = np.max(np.abs(pos - q["start"]), axis=1) <= GOAL_TOL
    at_goal = np.max(np.abs(pos - q["goal"]), axis=1) <= GOAL_TOL
    hits, last_start = [], None
    for i in range(len(t)):
        if at_start[i]:
            last_start = i
        elif at_goal[i]:
            if last_start is not None:
                hits.append((t[last_start], t[i]))
            last_start = None
    if len(hits) != 2:

        def runs(mask):
            edges = np.flatnonzero(np.diff(np.concatenate([[0], mask.astype(int), [0]])))
            return [
                (round(t[a], 2), round(t[b - 1], 2))
                for a, b in zip(edges[::2], edges[1::2], strict=True)
            ]

        raise RuntimeError(
            f"expected 2 start-to-goal motions of query {qid}, found {len(hits)}; joint states"
            f" {t[0]:.2f}-{t[-1]:.2f} s ({len(t)} samples, largest gap {np.diff(t).max():.3f} s);"
            f" at start {runs(at_start)}; at goal {runs(at_goal)}"
        )
    first = min(rows, key=lambda p: int(rows[p]["order"]))
    second = "moveit" if first == "direct" else "direct"
    return rows, {first: hits[0], second: hits[1]}


DVM_CAPTION = "same query, same settings, two pipelines"


def direct_vs_moveit(args):
    live = json.loads((pathlib.Path(args.live) / "frames.json").read_text())
    rows, segs = query_motions(live, args.csv, args.queries, args.query)
    stamps = np.array([f["stamp"] for f in live["frames"]])
    slow = args.slowdown
    pre, post = args.pre_s, args.post_s
    dur = max(b - a for a, b in segs.values())
    n = round((pre + dur * slow + post) * FPS)
    gap = 8
    pw = (WIDTH - gap) // 2
    cam = live["camera"]
    ph = round(cam["height"] * pw / cam["width"])
    times = {p: plan_plus_simplify(rows[p]) for p in rows}
    band = caption_band(
        DVM_CAPTION,
        "Planning time = path search plus shortening, measured once in this recording run (not"
        " a benchmark result). Each arm then executes its own plan on the same simulated"
        f" controller, {speed_phrase(1 / slow)}.",
    )
    titles = {"moveit": "MoveIt 2", "direct": "direct rebuild, no MoveIt"}
    frames = []
    for k in range(n):
        tg = k / FPS
        canvas = Image.new("RGB", (WIDTH, 50 + ph + band.height), BAND)
        for j, p in enumerate(("moveit", "direct")):
            a, b = segs[p]
            ts = min(a + max(0.0, tg - pre) / slow, b + 1.0)
            i = int(np.clip(np.searchsorted(stamps, ts, side="right") - 1, 0, len(stamps) - 1))
            img = Image.open(pathlib.Path(args.live) / live["frames"][i]["file"]).convert("RGB")
            img = img.resize((pw, ph), Image.LANCZOS)
            canvas.paste(
                header(pw, titles[p], f"planning time, this run: {times[p]:.3f} s"),
                (j * (pw + gap), 0),
            )
            canvas.paste(img, (j * (pw + gap), 50))
        canvas.paste(band, (0, 50 + ph))
        frames.append(canvas)
    colors = encode(frames, args.out)
    report(
        "direct_vs_moveit",
        args.out,
        frames,
        colors,
        caption=DVM_CAPTION,
        query=args.query,
        planning_time_this_run_s={p: round(v, 6) for p, v in times.items()},
        executed_s={p: round(b - a, 3) for p, (a, b) in segs.items()},
        playback_slowdown=slow,
    )


def check_live(args):
    live = json.loads((pathlib.Path(args.live) / "frames.json").read_text())
    rows, segs = query_motions(live, args.csv, args.queries, args.query)
    print(
        "[compose] live recording ok: "
        + ", ".join(f"{p} {b - a:.2f} s executed" for p, (a, b) in segs.items()),
        flush=True,
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    h = sub.add_parser("hero")
    h.add_argument("--frames", required=True)
    h.add_argument("--out", required=True)
    k = sub.add_parser("keep_upright")
    k.add_argument("--urdf", required=True)
    k.add_argument("--none", required=True)
    k.add_argument("--upright", required=True)
    k.add_argument("--out", required=True)
    k.add_argument("--motion-s", type=float, default=11.5)
    k.add_argument("--pre-s", type=float, default=1.0)
    k.add_argument("--post-s", type=float, default=1.8)
    for name in ("direct_vs_moveit", "check-live"):
        d = sub.add_parser(name)
        d.add_argument("--live", required=True)
        d.add_argument("--csv", required=True)
        d.add_argument("--queries", required=True)
        d.add_argument("--query", type=int, required=True)
        d.add_argument("--out")
        d.add_argument("--slowdown", type=float, default=2.0)
        d.add_argument("--pre-s", type=float, default=1.5)
        d.add_argument("--post-s", type=float, default=1.8)
    args = ap.parse_args()
    {
        "hero": hero,
        "keep_upright": keep_upright,
        "direct_vs_moveit": direct_vs_moveit,
        "check-live": check_live,
    }[args.cmd](args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
