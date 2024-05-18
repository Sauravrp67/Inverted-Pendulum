"""Separate, time-aligned mechanism and trace videos plus a static trace figure."""

from contextlib import contextmanager
from pathlib import Path
import subprocess

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from simulation.scene import ReplayScene


def render(name, runs, c, dest):
    scene = ReplayScene(c["plant"], 600, 456)
    font = ImageFont.truetype(
        str(Path(matplotlib.get_data_path()) / "fonts/ttf/DejaVuSans-Bold.ttf"), 17
    )

    for run, trace in runs.items():
        stem = f"{name}-{run}"
        title = "Lead" if run == "lead" else run.upper()
        fig, axes = plt.subplots(4, 1, sharex=True, figsize=(8, 5.6))
        ref = c["arm_target_rad"]
        fig.suptitle(title, fontsize=13, fontweight="bold")

        tilt = np.degrees(np.abs(np.arctan2(np.sin(trace.angle), np.cos(trace.angle))))
        for ax, data, label in zip(
            axes,
            (tilt, np.degrees(trace.arm), trace.limited_steps_s, trace.applied_Nm),
            ("Tilt (deg)", "Arm (deg)", "Step rate (steps/s)", "Torque (Nm)"),
        ):
            ax.plot(trace.t, data, color="#096a84")
            ax.set_ylabel(label)
            ax.grid(alpha=0.2)
            ax.spines[["right", "top"]].set_visible(False)

        if ref is not None:
            axes[1].axhline(np.degrees(ref), color="#b3562c", ls="--")
        axes[0].axhline(0, color="#b3562c", ls="--", lw=0.8)
        for torque in (-c["limit_Nm"], c["limit_Nm"]):
            axes[3].axhline(torque, color="#687887", ls=":", lw=0.8)

        capture = np.flatnonzero(trace.mode)
        if capture.size:
            for ax in axes:
                ax.axvline(trace.t[capture[0]], color="#687887", ls="--", lw=0.8)
            axes[0].annotate(
                "Handoff",
                (trace.t[capture[0]], 0.85),
                xycoords=("data", "axes fraction"),
                xytext=(4, 0),
                textcoords="offset points",
                color="#43545e",
            )

        pulse = np.flatnonzero(trace.push_Nm)
        
        if pulse.size:
            start = trace.t[pulse[0]]
            stop = trace.t[pulse[-1]] + c["sample_s"]
            for ax in axes:
                ax.axvspan(start, stop, color="#b3562c", alpha=0.3)

        failed = np.flatnonzero(trace.feasible == 0)
        
        if failed.size:
            stopped = trace.t[failed[0]]
            for ax in axes:
                ax.axvline(stopped, color="#b3261e", lw=1)
            axes[0].annotate(
                "Solver stop",
                (stopped, 0.6),
                xycoords=("data", "axes fraction"),
                xytext=(4, 0),
                textcoords="offset points",
                color="#b3261e",
            )

        axes[-1].set_xlabel("Time (s)")
        fig.tight_layout()
        fig.savefig(dest / f"{stem}.png", dpi=200)
        fig.canvas.draw()
        chart = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())).convert("RGB")
        ranges = [(ax.get_xlim(), ax.get_window_extent().bounds) for ax in axes]
        plt.close(fig)

        fps = min(25, 1 / c["sample_s"])
        frames = np.rint(
            np.linspace(0, len(trace) - 1, round(c["duration"] * fps) + 1)
        ).astype(int)
        with (
            encode_motion(dest, 600, 32 + 456, fps, stem) as motion,
            encode_motion(
                dest, chart.width, chart.height, fps, f"{stem}-traces"
            ) as traces,
        ):
            for k in frames:
                frame = Image.new("RGB", (600, 32 + 456), "white")
                scene.position(trace.arm[k], trace.angle[k])
                frame.paste(scene.image(), (0, 32))
                draw = ImageDraw.Draw(frame)
                draw.text(
                    (frame.width // 2, 16),
                    title,
                    anchor="mm",
                    font=font,
                    fill="#22313a",
                )
                motion.write(frame.tobytes())

                graph = chart.copy()
                draw = ImageDraw.Draw(graph)
                for (lo, hi), (left, bottom, w, h) in ranges:
                    px = left + w * (trace.t[k] - lo) / (hi - lo)
                    draw.line(
                        (
                            px,
                            chart.height - bottom - h,
                            px,
                            chart.height - bottom,
                        ),
                        fill="#43545e",
                    )
                traces.write(graph.tobytes())


@contextmanager
def encode_motion(dest, width, height, fps, stem="motion"):
    video = dest / f"{stem}.mp4"
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgb24",
        "-video_size",
        f"{width}x{height}",
        "-framerate",
        str(fps),
        "-i",
        "pipe:0",
        "-c:v",
        "libx264",
        "-threads",
        "2",
        "-pix_fmt",
        "yuv420p",
        "-crf",
        "19",
        "-movflags",
        "+faststart",
        str(video),
    ]
    with subprocess.Popen(command, stdin=subprocess.PIPE) as encoder:
        yield encoder.stdin
        encoder.stdin.close()
        if encoder.wait():
            raise RuntimeError("ffmpeg failed while encoding the Drake replay")
