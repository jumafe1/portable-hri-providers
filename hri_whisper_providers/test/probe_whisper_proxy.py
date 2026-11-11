#!/usr/bin/env python3
"""ROS proxy integration with fake worker, isolated domain required by caller."""
import argparse
import json
import os
import signal
import subprocess
import time

import rclpy
from hri_capability_interfaces.msg import SpeechCaptureStatus, SpeechTranscript
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
    node = isolated_probe_node("whisper_proxy_probe")
    received, requests, leases = [], [], []
    publisher = node.create_publisher(SpeechTranscript, "/whisper/transcript", 10)
    status_publisher = node.create_publisher(SpeechCaptureStatus, "/whisper/capture_status", 1)
    statuses = []
    node.create_subscription(SpeechCaptureStatus, "/hri/speech/capture_status", statuses.append, 1)
    control = node.create_publisher(String, "/test/whisper/control", 10)
    node.create_subscription(SpeechTranscript, "/hri/speech/transcript", received.append, 10)
    node.create_subscription(String, "/whisper/lease", lambda msg: leases.append(msg.data), 10)
    client = node.create_client(TranscribeSpeech, "/hri/speech/transcribe")
    backend = None
    process = None
    checks = []

    def pump(predicate, seconds=5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
        raise AssertionError("Timed out waiting for ROS condition")

    def drain(seconds=.3):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    def launch():
        return subprocess.Popen(["ros2", "run", "hri_whisper_providers", "whisper_proxy_fixture"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def stop():
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5)
        drain()

    def request(public_id="public-test", duration=.5, continuous=False, silence=.8):
        req = TranscribeSpeech.Request()
        req.duration_sec, req.language, req.request_id = duration, "es", public_id
        req.continuous, req.silence_sec = continuous, silence
        future = client.call_async(req)
        pump(future.done)
        return future.result()

    def on_request(req, response):
        requests.append(req.request_id)
        response.request_id = req.request_id
        response.accepted = True
        response.message = "capture_started"
        return response

    def emit(private_id, text="synthetic test", segment_index=0, final=True):
        result = SpeechTranscript()
        result.header.stamp = node.get_clock().now().to_msg()
        result.request_id, result.text = private_id, text
        result.language, result.success, result.message = "es", True, "transcribed"
        result.segment_index, result.final = segment_index, final
        publisher.publish(result)

    try:
        process = launch()
        pump(lambda: client.service_is_ready() and bool(leases))
        result = request()
        assert not result.accepted and result.message == "whisper_worker_unavailable"
        checks.append("missing_worker_rejected")
        result = request(duration=float("nan"))
        assert not result.accepted and result.message == "duration_out_of_range"
        checks.append("invalid_duration_rejected")
        backend = node.create_service(TranscribeSpeech, "/whisper/transcribe", on_request)
        drain(1)
        result = request()
        assert result.accepted and result.request_id == "public-test"
        assert requests[-1] != result.request_id and requests[-1].startswith(leases[-1] + ":")
        private_id = requests[-1]
        checks.append("public_private_correlation")
        busy = request("other")
        assert not busy.accepted and busy.message == "provider_busy"
        checks.append("bounded_single_request")
        emit("wrong-owner:1")
        drain()
        assert not received
        emit(private_id)
        pump(lambda: bool(received))
        assert received[-1].request_id == "public-test" and received[-1].text == "synthetic test"
        checks.append("matching_result_only")
        emit(private_id)
        drain()
        assert len(received) == 1
        checks.append("duplicate_result_ignored")
        assert request("cancel-me").accepted
        old_id = requests[-1]
        command = String()
        command.data = "stop"
        control.publish(command)
        pump(lambda: bool(leases) and leases[-1] == "")
        emit(old_id)
        drain()
        assert len(received) == 1
        checks.append("release_suppresses_results")
        command.data = "start"
        control.publish(command)
        pump(lambda: bool(leases[-1]))
        drain(.5)
        assert request("new-owner-request").accepted
        emit(old_id)
        drain()
        assert len(received) == 1
        emit(requests[-1])
        pump(lambda: len(received) == 2)
        assert received[-1].request_id == "new-owner-request"
        checks.append("reacquire_rejects_old_owner")
        invalid = request("stream", 10.0, True, float("nan"))
        assert not invalid.accepted and invalid.message == "silence_out_of_range"
        checks.append("invalid_endpoint_rejected")
        assert request("stream", 10.0, True).accepted
        stream_id = requests[-1]
        status = SpeechCaptureStatus(request_id=stream_id, state="listening")
        for _ in range(12):
            status.header.stamp = node.get_clock().now().to_msg()
            status_publisher.publish(status)
            drain(.25)
        assert statuses and statuses[-1].request_id == "stream" and len(received) == 2
        assert not request("busy").accepted
        checks.append("silent_stream_lives_with_heartbeat")
        emit(stream_id, segment_index=1, final=False)
        emit(stream_id, segment_index=2, final=False)
        pump(lambda: len(received) == 4)
        assert [r.segment_index for r in received[-2:]] == [1, 2]
        assert all(r.request_id == "stream" for r in received[-2:])
        emit(stream_id, segment_index=1, final=False)
        drain()
        assert len(received) == 4
        checks.append("continuous_results_correlated_and_duplicates_ignored")
        pump(lambda: len(received) == 5, seconds=4)
        assert received[-1].message == "worker_stream_timeout" and received[-1].final
        emit(stream_id, segment_index=3, final=False)
        drain()
        assert len(received) == 5
        checks.append("stream_watchdog_invalidates_late_results")
        print(json.dumps({"passed": len(checks), "checks": checks, "real_audio": False, "real_whisper": False}))
    finally:
        if process is not None and process.poll() is None:
            stop()
        if backend is not None:
            node.destroy_service(backend)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
