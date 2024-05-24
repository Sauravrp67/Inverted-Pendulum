import hashlib
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

import numpy as np

from simulation import ROOT
from simulation.configuration import CONTROLLERS, config
from simulation import run_demo
from simulation.scene import ReplayScene

showcase = None
if importlib.util.find_spec("simulation.showcase") is not None:
    from simulation import showcase


class PresentationTests(unittest.TestCase):
    def test_saved_manifest_matches_files(self):
        c = config("pendulum-regulation", "pid")
        c["duration"] = 0.02
        payload = b"frame" * 300000

        def render(name, runs, settings, dest):
            (dest / "frames.bin").write_bytes(payload)

        with TemporaryDirectory() as folder:
            dest = Path(folder)
            with patch.object(run_demo, "config", return_value=c), patch.object(
                run_demo, "render", side_effect=render
            ):
                run_demo.generate("pendulum-regulation", dest, runs=("pid",))

            manifest = json.loads((dest / "manifest.json").read_text())
            self.assertIn("frames.bin", manifest["files"])
            for name, digest in manifest["files"].items():
                self.assertEqual(
                    digest, hashlib.sha256((dest / name).read_bytes()).hexdigest()
                )

    def test_drake_renderer_tracks_joint_positions(self):
        c = config("pendulum-regulation", "pid")
        scene = ReplayScene(c["plant"], 160, 128)
        scene.position(0, 0)
        upright = np.asarray(scene.image())
        scene.position(0.5, np.pi)
        hanging = np.asarray(scene.image())

        self.assertEqual(upright.shape, (128, 160, 3))
        self.assertGreater(np.ptp(upright), 50)
        self.assertGreater(np.count_nonzero(upright != hanging), 100)

    def test_selected_controller_has_separate_output(self):
        argv = [
            "run_demo",
            "pendulum-regulation",
            "--controller",
            "pid",
            "--seed",
            "43",
            "--estimator",
            "difference",
        ]
        with patch("sys.argv", argv), patch.object(run_demo, "reproduce") as run:
            run_demo.main()
        run.assert_called_once_with(
            "pendulum-regulation",
            ROOT / "build/showcase/pendulum-regulation-seed-43-difference-pid",
            43,
            "difference",
            ("pid",),
        )

    def test_default_still_runs_all_controllers(self):
        with (
            patch("sys.argv", ["run_demo", "pendulum-regulation"]),
            patch.object(run_demo, "reproduce") as run,
        ):
            run_demo.main()
        self.assertEqual(run.call_args.args[-1], CONTROLLERS)

    @unittest.skipIf(showcase is None, "README publishing script is local-only")
    def test_panel_requires_all_four_sources(self):
        with TemporaryDirectory() as folder:
            dest = Path(folder)
            with self.assertRaises(FileNotFoundError):
                showcase.panel("pid", dict.fromkeys(showcase.OBJECTIVES, dest), dest)

    @unittest.skipIf(showcase is None, "README publishing script is local-only")
    def test_panel_rejects_mismatched_timelines(self):
        with TemporaryDirectory() as folder:
            dest = Path(folder)
            for objective in showcase.OBJECTIVES:
                for suffix in ("", "-traces"):
                    (dest / f"{objective}-pid{suffix}.mp4").touch()
            probes = [
                Mock(
                    stdout=json.dumps(
                        {"streams": [{"nb_frames": str(n), "r_frame_rate": "25/1"}]}
                    )
                )
                for n in (501, 501, 501, 251)
            ]
            with patch.object(showcase.subprocess, "run", side_effect=probes):
                with self.assertRaisesRegex(ValueError, "matching frame counts"):
                    showcase.panel(
                        "pid", dict.fromkeys(showcase.OBJECTIVES, dest), dest
                    )


if __name__ == "__main__":
    unittest.main()
