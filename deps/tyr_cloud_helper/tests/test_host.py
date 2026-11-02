"""Run the actual framed host against an offline fake plugin, without account data."""
import hashlib
import json
import os
from pathlib import Path
import queue
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

HOST, PLUGIN = map(Path, sys.argv[1:3])
del sys.argv[1:3]
MAGIC = 0x52424A50


class Host:
    def __init__(self):
        self.directory = tempfile.TemporaryDirectory()
        root = Path(self.directory.name)
        for name in ("libbambu_networking.so", "libBambuSource.so"):
            shutil.copyfile(PLUGIN, root / name)
        (root / "linux_payload_manifest.json").write_text(json.dumps({"files": [
            {"name": name, "abi_version": "02.05.02.58",
             "sha256": hashlib.sha256((root / name).read_bytes()).hexdigest()}
            for name in ("libbambu_networking.so", "libBambuSource.so")]}))
        self.stderr = tempfile.TemporaryFile()
        self.process = subprocess.Popen([str(HOST)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=self.stderr, env={**os.environ,
                "HOME": str(root), "PJARCZAK_BAMBU_PLUGIN_DIR": str(root),
                "PJARCZAK_BAMBU_NETWORK_SO": str(root / "libbambu_networking.so"),
                "PJARCZAK_BAMBU_SOURCE_SO": str(root / "libBambuSource.so"),
                "PJARCZAK_EXPECTED_BAMBU_NETWORK_VERSION": "02.05.02.58"})
        self.next_id = 0
        self.replies = {}
        self.queue = queue.Queue()
        self.reader = threading.Thread(target=self.read, daemon=True)
        self.reader.start()

    def exact(self, count):
        result = b""
        while len(result) < count:
            block = self.process.stdout.read(count - len(result))
            if not block:
                raise EOFError("Host closed its response pipe")
            result += block
        return result

    def read(self):
        try:
            while True:
                magic, kind, request_id, size = struct.unpack("<IIII", self.exact(16))
                if magic != MAGIC or kind != 2 or size > 16 * 1024 * 1024:
                    raise ValueError("Invalid response frame")
                self.queue.put((request_id, json.loads(self.exact(size))))
        except (EOFError, ValueError, OSError) as exc:
            self.queue.put((None, exc))

    def request(self, method, payload=None):
        self.next_id += 1
        body = json.dumps({"method": method, "payload": payload or {}}).encode()
        self.process.stdin.write(struct.pack("<IIII", MAGIC, 1, self.next_id, len(body)) + body)
        self.process.stdin.flush()
        return self.next_id

    def response(self, request_id, timeout=3):
        deadline = time.monotonic() + timeout
        while request_id not in self.replies:
            rid, reply = self.queue.get(timeout=max(0.001, deadline - time.monotonic()))
            if rid is None:
                raise reply
            self.replies[rid] = reply
        return self.replies.pop(request_id)

    def call(self, method, payload=None):
        return self.response(self.request(method, payload))

    def event(self, name):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            for item in self.call("bridge.poll_events")["events"]:
                if item["name"] == name:
                    return item["payload"]
            time.sleep(0.005)
        raise AssertionError(f"No {name} event received")

    def close(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.process.stdin.close()
        self.process.stdout.close()
        self.reader.join(timeout=1)
        self.stderr.close()
        self.directory.cleanup()


class HostTests(unittest.TestCase):
    def setUp(self):
        self.host = Host()
        self.addCleanup(self.host.close)
        handshake = self.host.call("bridge.handshake")
        self.assertTrue(handshake["network_loaded"], handshake)
        self.agent = self.host.call("net.create_agent", {"log_dir": self.host.directory.name})["value"]

    def submit(self, mode="wait", job=1):
        return self.host.request("net.start_print", {"agent": self.agent, "client_job_id": job,
            "params": {"task_name": mode, "connection_type": "cloud", "plate_index": 7,
                "filename": "/artifacts/plate.gcode.3mf", "ams_mapping": "[2]",
                "task_bed_type": "textured_plate"}})

    def reply(self, event, value, request_id=None):
        self.assertIn("request_id", event)
        self.host.call("bridge.job_wait_reply", {"job_id": event["job_id"],
            "request_id": event["request_id"] if request_id is None else request_id, "reply": value})

    def test_wait_false_is_returned_to_the_plugin(self):
        request = self.submit()
        event = self.host.event("job.wait")
        self.assertIn("request_id", event)
        self.assertEqual(json.loads(event["job_info"])["job_id"], "offline-job")
        self.reply(event, False)
        self.assertEqual(self.host.response(request)["value"], -18)

    def test_wait_true_finishes_and_delivers_final_progress(self):
        request = self.submit()
        event = self.host.event("job.wait")
        self.reply(event, True)
        self.assertEqual(self.host.response(request)["value"], 0)
        self.assertEqual(self.host.event("job.update_status")["status"], 6)

    def test_stale_wait_reply_does_not_release_a_new_wait(self):
        request = self.submit("two_waits")
        first = self.host.event("job.wait")
        self.reply(first, True)
        second = self.host.event("job.wait")
        self.assertNotEqual(first["request_id"], second["request_id"])
        self.reply(second, True, request_id=first["request_id"])
        with self.assertRaises(queue.Empty):
            self.host.response(request, timeout=0.1)
        self.reply(second, False)
        self.assertEqual(self.host.response(request)["value"], -18)

    def test_cancellation_releases_pending_wait(self):
        request = self.submit()
        self.host.event("job.wait")
        self.host.call("bridge.job_cancel", {"job_id": 1, "cancel": True})
        self.assertEqual(self.host.response(request)["value"], -18)

    def test_cancellation_reaches_running_upload(self):
        request = self.submit("cancel")
        self.assertEqual(self.host.event("job.update_status")["code"], 25)
        self.host.call("bridge.job_cancel", {"job_id": 1, "cancel": True})
        self.assertEqual(self.host.response(request)["value"], -18)

    def test_cancel_before_submission_registration_never_calls_the_plugin(self):
        self.host.call("bridge.job_cancel", {"job_id": 31, "cancel": True})
        request = self.submit("cancel", job=31)
        self.assertEqual(self.host.response(request)["value"], -18)
        self.assertEqual(self.host.call("bridge.poll_events")["events"], [])

    def test_duplicate_job_cannot_replace_cancellation_ownership(self):
        original = self.submit("cancel", job=42)
        self.host.event("job.update_status")
        duplicate = self.host.call("net.start_print", {"agent": self.agent,
            "client_job_id": 42, "params": {}})
        self.assertFalse(duplicate["ok"])
        self.host.call("bridge.job_cancel", {"job_id": 42, "cancel": True})
        self.assertEqual(self.host.response(original)["value"], -18)

    def test_uploader_process_cannot_be_reused_after_completion(self):
        original = self.submit("cancel", job=42)
        self.host.event("job.update_status")
        self.host.call("bridge.job_cancel", {"job_id": 42, "cancel": True})
        self.assertEqual(self.host.response(original)["value"], -18)
        replay = self.host.call("net.start_print", {"agent": self.agent,
            "client_job_id": 43, "params": {}})
        self.assertFalse(replay["ok"])

    def test_missing_wait_reply_is_not_acceptance(self):
        request = self.submit()
        event = self.host.event("job.wait")
        incomplete = self.host.call("bridge.job_wait_reply", {
            "job_id": event["job_id"], "request_id": event["request_id"]})
        self.assertFalse(incomplete["ok"])
        with self.assertRaises(queue.Empty):
            self.host.response(request, timeout=0.1)
        self.reply(event, False)
        self.assertEqual(self.host.response(request)["value"], -18)

    def test_auth_diagnostics_do_not_echo_credentials(self):
        reply = self.host.call("net.change_user", {"agent": self.agent,
            "user_info": json.dumps({"command": "user_login", "data": {"token": "PRIVATE-SESSION-SENTINEL"}})})
        self.assertNotIn("PRIVATE-SESSION-SENTINEL", json.dumps(reply))
        headers = self.host.call("net.set_extra_http_header", {"agent": self.agent,
            "headers": {"Authorization": "PRIVATE-HEADER-SENTINEL"}})
        self.assertNotIn("PRIVATE-HEADER-SENTINEL", json.dumps(headers))
        self.host.stderr.seek(0)
        self.assertNotIn(b"PRIVATE-", self.host.stderr.read())

    def test_handshake_advertises_required_semantics(self):
        caps = self.host.call("bridge.handshake")["auth_capabilities"]
        for key in ("tyr_cloud_helper_version", "tyr_redaction_version", "tyr_wait_reply_version"):
            self.assertEqual(caps.get(key), 1, key)
        self.assertTrue(caps["bambu_network_start_print"])

    def test_handshake_lists_only_the_pinned_sdk_print_parameters(self):
        fields = self.host.call("bridge.handshake").get("supported_print_params", [])
        self.assertIn("filename", fields)
        self.assertIn("ams_mapping2", fields)
        self.assertIn("auto_flow_cali", fields)
        self.assertNotIn("task_timelapse_use_internal", fields)
        self.assertNotIn("queue_plate_id", fields)

    def test_invalid_field_returns_an_error_without_killing_the_host(self):
        reply = self.host.call("net.start_print", {"agent": self.agent,
            "params": {"plate_index": "PRIVATE-INVALID-SENTINEL"}})
        self.assertFalse(reply["ok"])
        self.assertNotIn("PRIVATE-INVALID-SENTINEL", json.dumps(reply))
        self.assertEqual(self.host.call("bridge.ping")["value"], "pong")


if __name__ == "__main__":
    unittest.main()
