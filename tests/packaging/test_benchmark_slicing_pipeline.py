import importlib.util
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "benchmark_slicing_pipeline.py"
spec = importlib.util.spec_from_file_location("slicing_benchmark", SCRIPT)
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


class CommandComparisonTests(unittest.TestCase):
    def test_preview_tags_and_their_positions_are_part_of_equivalence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.gcode"
            text = ("; timestamp one\n; CHANGE_LAYER\n; LAYER_HEIGHT: 0.2\n"
                    "; LINE_WIDTH: 0.4\n; FEATURE: Outer wall\nG1 X10 E1\nG1 X20 E2\n")
            path.write_text(text)
            baseline = benchmark.preview_fingerprint(path)
            path.write_text(text.replace("timestamp one", "timestamp two"))
            self.assertEqual(benchmark.preview_fingerprint(path), baseline)
            for old, new in (("Outer wall", "Support"), ("0.4", "0.5"), ("0.2", "0.3"),
                             ("; CHANGE_LAYER\n", ""),
                             ("; FEATURE: Outer wall\nG1 X10 E1", "G1 X10 E1\n; FEATURE: Outer wall")):
                with self.subTest(change=(old, new)):
                    path.write_text(text.replace(old, new))
                    self.assertNotEqual(benchmark.preview_fingerprint(path), baseline)

    def test_equivalence_requires_whole_observed_baseline_stream(self):
        original = {"commands": {"sha256": "commands-a"},
                    "preview": {"sha256": "preview-a"},
                    "output_metadata": {"filaments": [1], "total_predication": 30}}
        alternate = {"commands": {"sha256": "commands-b"},
                     "preview": {"sha256": "preview-b"},
                     "output_metadata": {"filaments": [1], "total_predication": 31}}
        references = [benchmark.equivalence_record(original), benchmark.equivalence_record(alternate)]
        self.assertIn(benchmark.equivalence_record(alternate), references)
        # Matching a baseline's commands alone must not admit different preview
        # semantics, metadata, or a mixture of two independently observed runs.
        for key in ("commands", "preview", "output_metadata"):
            with self.subTest(changed=key):
                mixed = {**original, key: alternate[key]}
                self.assertNotIn(benchmark.equivalence_record(mixed), references)

    def test_comments_are_ignored_but_extrusion_changes_are_detected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.gcode"
            path.write_text("; timestamp one\nG1 X10 Y20 E1.23 ; wall\nG1 E0.23\n")
            baseline = benchmark.command_fingerprint(path)
            path.write_text("; timestamp two\n G1 X10 Y20 E1.23 ; other comment\nG1 E0.23\n\n")
            self.assertEqual(benchmark.command_fingerprint(path), baseline)
            self.assertEqual(baseline["commands"], 2)
            path.write_text("G1 X10 Y20 E1.24\nG1 E0.23\n")
            self.assertNotEqual(benchmark.command_fingerprint(path), baseline)


if __name__ == "__main__":
    unittest.main()
