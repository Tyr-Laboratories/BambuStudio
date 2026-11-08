import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


@unittest.skipUnless(sys.platform == "darwin", "macOS process instruction counters")
class ProcessMetricsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.root = Path(cls.directory.name)
        source = Path(__file__).resolve().parents[2] / "scripts" / "measure_macos_process.c"
        cls.runner = cls.root / "measure"
        cls.worker = cls.root / "worker"
        subprocess.run(["cc", "-O2", str(source), "-o", str(cls.runner)], check=True)
        worker = cls.root / "worker.c"
        worker.write_text("""
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
static void *work(void *argument) {
    const unsigned count = *(const unsigned *)argument;
    volatile uint64_t value = 7;
    for (unsigned i=0; i<count; ++i) value = value * 1664525u + i;
    return 0;
}
int main(int argc, char **argv) {
    const unsigned count = (unsigned)strtoul(argv[1], 0, 10);
    if (atoi(argv[4]) == 2) {
        pthread_t first, second;
        if (pthread_create(&first, 0, work, (void *)&count) ||
            pthread_create(&second, 0, work, (void *)&count)) return 99;
        pthread_join(first, 0);
        pthread_join(second, 0);
    } else work((void *)&count);
    if (atoi(argv[2])) usleep(150000);
    return atoi(argv[3]);
}
""")
        subprocess.run(["cc", "-O2", str(worker), "-o", str(cls.worker)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def measure(self, work, wait=0, exit_code=0, threads=1):
        path = self.root / "metrics.json"
        result = subprocess.run([str(self.runner), str(path), "--", str(self.worker),
                                 str(work), str(wait), str(exit_code), str(threads)], check=False)
        self.assertEqual(result.returncode, exit_code)
        return json.loads(path.read_text())["retired_instructions"]

    def test_sleep_does_not_count_as_computational_work(self):
        work = self.measure(10000000)
        sleeping = self.measure(10000000, wait=1)
        self.assertGreater(work, 0)
        self.assertLess(abs(sleeping / work - 1), 0.05)

    def test_instruction_count_tracks_work_after_startup(self):
        startup = self.measure(0)
        once = self.measure(10000000) - startup
        twice = self.measure(20000000) - startup
        self.assertGreater(once, 0)
        self.assertLess(abs(twice / once - 2), 0.1)

    def test_failed_child_exit_status_is_preserved(self):
        self.assertGreater(self.measure(1000000, exit_code=7), 0)

    def test_finished_worker_threads_are_included(self):
        startup = self.measure(0)
        serial = self.measure(20000000) - startup
        threaded = self.measure(10000000, threads=2) - startup
        self.assertLess(abs(threaded / serial - 1), 0.1)


if __name__ == "__main__":
    unittest.main()
