#!/usr/bin/env python3
"""Common application + real capability lifecycle + synthetic worker, no audio."""
import argparse
import collections
import json
import os
import signal
import subprocess
import threading
import time

import rclpy
from hri_capability_interfaces.msg import SpeechTranscript
from hri_capability_interfaces.srv import TranscribeSpeech
from std_msgs.msg import String


def isolated_probe_node(name):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--software-only", action="store_true", required=True)
    parser.parse_args()
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if not domain.isdigit() or not 100 <= int(domain) <= 220:
        raise SystemExit("Use a dedicated ROS_DOMAIN_ID between 100 and 220.")
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("Set ROS_LOCALHOST_ONLY=1; no physical hardware allowed.")
    rclpy.init()
    node = rclpy.create_node(name)
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        strangers = list(node.get_node_names_and_namespaces())
        strangers.remove((name, "/"))
        services = [service for service, _ in node.get_service_names_and_types()
                    if not service.startswith(f"/{name}/")]
        topics = [topic for topic, _ in node.get_topic_names_and_types()
                  if topic not in ("/rosout", "/parameter_events")]
        if strangers or services or topics:
            raise SystemExit("Domain is not isolated; refusing to create fake speech endpoints: "
                             f"nodes={strangers}, services={services}, topics={topics}")
        return node
    except BaseException:
        node.destroy_node()
        rclpy.shutdown()
        raise


def main():
    node = isolated_probe_node("transcribe_app_fake_worker")
    publisher = node.create_publisher(SpeechTranscript, "/whisper/transcript", 10)
    pending, requests, leases = [], [], []
    behavior = ["success"]
    node.create_subscription(String, "/whisper/lease", lambda msg: leases.append(msg.data), 10)

    def admit(request, response):
        requests.append(request.request_id)
        retry_lease = behavior[0] == "lease_retry" and len(requests) == 1
        response.accepted = behavior[0] != "reject" and not retry_lease
        response.request_id = request.request_id
        response.message = ("lease_unavailable" if retry_lease else
                            "local_model_unavailable" if behavior[0] == "reject" else "capture_started")
        if response.accepted and behavior[0] in ("success", "lease_retry"):
            pending.append((time.monotonic() + .5, request.request_id))
        return response

    node.create_service(TranscribeSpeech, "/whisper/transcribe", admit)

    def emit():
        for due, identifier in list(pending):
            if time.monotonic() >= due:
                pending.remove((due, identifier))
                result = SpeechTranscript()
                result.header.stamp = node.get_clock().now().to_msg()
                result.request_id = identifier
                result.text, result.language = "synthetic phrase", "es"
                result.success, result.message = True, "transcribed"
                publisher.publish(result)

    node.create_timer(.02, emit)
    evidence = []
    try:
        for case in ("success", "reject", "cancel", "lease_retry"):
            behavior[0] = case
            requests.clear()
            leases.clear()
            lines = collections.deque(maxlen=500)
            process = subprocess.Popen([
                "ros2", "launch", "hri_reference_app", "hri_app.launch.py", "robot:=nao",
                "mode:=transcribe_speech", "confirm:=ESCUCHAR", "record_duration:=0.5", "timeout:=8.0"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)

            def read_output():
                for line in process.stdout:
                    lines.append(line.rstrip())

            reader = threading.Thread(target=read_output, daemon=True)
            reader.start()
            interrupted = False
            deadline = time.monotonic() + 25
            try:
                while process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.02)
                    if case == "cancel" and requests and not interrupted:
                        os.killpg(process.pid, signal.SIGINT)
                        interrupted = True
                if process.poll() is None:
                    raise AssertionError("Application did not terminate")
                reader.join(timeout=2)
                results = []
                for line in lines:
                    start = line.find('{"mode": "transcribe_speech"')
                    if start >= 0:
                        results.append(json.loads(line[start:]))
                assert results, "No final app JSON: " + "\n".join(lines)
                result = results[-1]
                expected = {"success": "transcription_completed", "lease_retry": "transcription_completed", "reject": "local_model_unavailable",
                            "cancel": "cancelled"}[case]
                assert result["message"] == expected, result
                assert result["released"] is True, result
                assert result["outcome"] == ("application_succeeded" if case in ("success", "lease_retry") else "application_failed"), result
                if case == "lease_retry":
                    assert len(requests) == 2 and requests[0] != requests[1]
                assert "text" not in result and not any("synthetic phrase" in line for line in lines)
                evidence.append({"case": case, "message": result["message"], "released": True,
                                 "text_not_logged": True, "exit_code": process.returncode})
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=10)
                process.stdout.close()
        print(json.dumps({"passed": len(evidence), "cases": evidence, "real_audio": False,
                          "real_whisper": False}))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
