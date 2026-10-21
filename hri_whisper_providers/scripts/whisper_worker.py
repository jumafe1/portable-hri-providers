#!/usr/bin/env python3
"""Opt-in local ASR worker. Audio and transcripts are never written to disk."""

import importlib.util
import pathlib
import queue
import threading
import time

from transcription_core import CaptureSession, validate_format


def preflight(model_path):
    if not model_path or not pathlib.Path(model_path).is_file():
        return "local_model_unavailable"
    if pathlib.Path(model_path).suffix != ".pt":
        return "local_model_must_be_pt"
    for package in ("whisper", "torch", "numpy"):
        if importlib.util.find_spec(package) is None:
            return "dependency_unavailable:" + package
    return ""


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from std_msgs.msg import String
    from hri_capability_interfaces.msg import SpeechTranscript
    from hri_capability_interfaces.srv import TranscribeSpeech
    try:
        from audio_common_msgs.msg import AudioDataStamped, AudioInfo
    except ImportError as error:
        raise SystemExit("dependency_unavailable:audio_common_msgs (no audio captured)") from error

    class WhisperWorker(Node):
        def __init__(self):
            super().__init__("whisper_worker")
            self.declare_parameter("model_path", "")
            self.declare_parameter("audio_topic", "/nao/mic_stamped")
            self.declare_parameter("audio_info_topic", "/nao/mic_info")
            # Explicit adapter acknowledgement: never infer byte layout from a topic name.
            self.declare_parameter("input_format", "")
            self.model_path = self.get_parameter("model_path").value
            self.input_format = self.get_parameter("input_format").value
            self.session = CaptureSession()
            self.format_error = "audio_info_unavailable"
            self.completed = queue.Queue(maxsize=1)
            self.thread = None
            self.audio_subscription = None
            self.audio_after_ns = 0
            self.inference_deadline = 0.0
            self.model = None
            self.publisher = self.create_publisher(SpeechTranscript, "/whisper/transcript", 10)
            self.create_subscription(String, "/whisper/lease", self.on_lease, 1)
            self.create_subscription(AudioInfo, self.get_parameter("audio_info_topic").value,
                                     self.on_info, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_service(TranscribeSpeech, "/whisper/transcribe", self.on_request)
            self.create_timer(.05, self.poll)
            error = preflight(self.model_path)
            if error:
                self.get_logger().warning(error + "; requests rejected until worker is configured/restarted")

        def on_lease(self, message):
            self.session.lease(message.data, time.monotonic())
            if self.session.capture is None:
                self.stop_audio()

        def stop_audio(self):
            if self.audio_subscription is not None:
                self.destroy_subscription(self.audio_subscription)
                self.audio_subscription = None

        def on_info(self, message):
            self.format_error = validate_format(message.channels, message.sample_rate,
                                                message.sample_format, message.coding_format)
            if self.format_error and self.session.capture is not None:
                self.fail_capture(self.format_error)

        def on_request(self, request, response):
            error = preflight(self.model_path)
            if self.input_format != "naoqi_pcm_s16le":
                error = "input_format_not_acknowledged"
            if not error:
                error = self.format_error
            if not error and ((self.thread is not None and self.thread.is_alive()) or not self.completed.empty()):
                error = "inference_busy"
            if not error:
                error = self.session.admit(request.request_id, request.duration_sec,
                                            request.language, time.monotonic())
            if not error:
                try:
                    self.audio_after_ns = self.get_clock().now().nanoseconds
                    self.audio_subscription = self.create_subscription(
                        AudioDataStamped, self.get_parameter("audio_topic").value,
                        self.on_audio, qos_profile_sensor_data)
                except Exception:
                    self.session.clear()
                    error = "audio_subscription_failed"
            response.accepted = not bool(error)
            response.request_id = request.request_id
            response.provider = "hri_whisper_providers/WhisperTranscribeSpeech"
            response.message = error or "capture_started"
            return response

        def emit(self, request_id, success, message, text="", language="", stamp_ns=0):
            result = SpeechTranscript()
            result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            result.request_id = request_id
            result.provider = "hri_whisper_providers/WhisperTranscribeSpeech"
            result.success, result.message = success, message
            result.text, result.language = text, language
            self.publisher.publish(result)

        def fail_capture(self, reason):
            capture = self.session.capture
            if capture is not None:
                self.emit(capture.request_id, False, reason, stamp_ns=capture.stamp_ns)
            self.session.clear()
            self.stop_audio()

        def on_audio(self, message):
            if self.session.capture is None:
                return
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if 0 < stamp_ns < self.audio_after_ns:
                return  # Ignore buffered samples produced before this explicit request.
            try:
                capture = self.session.audio(message.audio.data, stamp_ns,
                                               self.get_clock().now().nanoseconds, time.monotonic())
            except ValueError as error:
                self.fail_capture(str(error))
                return
            if capture is not None:
                self.stop_audio()
                self.inference_deadline = time.monotonic() + 60.0
                self.thread = threading.Thread(target=self.infer, args=(capture,), daemon=True)
                self.thread.start()

        def infer(self, capture):
            # This thread never calls ROS: inference cannot block the executor or publish after release.
            text, language, success, message = "", "", False, "inference_failed"
            audio = None
            try:
                import numpy as np
                import torch
                import whisper
                audio = np.frombuffer(capture.data, dtype="<i2").astype(np.float32) / 32768.0
                capture.data.clear()
                if float(np.sqrt(np.mean(audio * audio))) < .005:
                    message = "no_speech_energy"
                else:
                    torch.set_num_threads(2)
                    if self.model is None:
                        # An existing explicit local file avoids Whisper's model-name download path.
                        self.model = whisper.load_model(self.model_path, device="cpu")
                    output = self.model.transcribe(audio, language=capture.language or None,
                                                   task="transcribe", fp16=False,
                                                   condition_on_previous_text=False, verbose=None)
                    text, language = str(output.get("text", "")).strip()[:8192], str(output.get("language", ""))[:32]
                    success, message = bool(text), "transcribed" if text else "no_speech"
            except Exception:
                # No exception payload: third-party exceptions can contain user data.
                message = "inference_failed"
            finally:
                if audio is not None:
                    audio.fill(0)
                capture.data.clear()
                self.completed.put_nowait((capture.request_id, success, message, text, language, capture.stamp_ns))

        def poll(self):
            now = time.monotonic()
            if not self.session.alive(now):
                self.session.clear()
                self.stop_audio()
            capture = self.session.capture
            if capture is not None and now > capture.deadline:
                self.fail_capture("audio_capture_timeout")
            if self.session.inflight and now > self.inference_deadline:
                self.emit(self.session.inflight, False, "inference_timeout")
                self.session.clear()
            try:
                result = self.completed.get_nowait()
            except queue.Empty:
                return
            if self.session.accepts_result(result[0], now):
                self.emit(*result)
                self.session.clear()

    rclpy.init(args=args)
    node = WhisperWorker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.session.clear()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
