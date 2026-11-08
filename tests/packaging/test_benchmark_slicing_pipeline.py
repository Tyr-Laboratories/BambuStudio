import importlib.util
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "benchmark_slicing_pipeline.py"
spec = importlib.util.spec_from_file_location("slicing_benchmark", SCRIPT)
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


class CommandComparisonTests(unittest.TestCase):
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
