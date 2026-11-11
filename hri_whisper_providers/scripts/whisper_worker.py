#!/usr/bin/env python3
"""Opt-in local ASR worker. Audio and transcripts are never written to disk."""

import importlib.util
import pathlib
import queue
import threading
import time

from transcription_core import CaptureSession, validate_format
from speech_stream import SpeechStream, make_vad, validate_stream


def preflight(model_path):
    if not model_path or not pathlib.Path(model_path).is_file():
        return "local_model_unavailable"
    if pathlib.Path(model_path).suffix != ".pt":
        return "local_model_must_be_pt"
    for package in ("whisper", "torch", "numpy"):
        if importlib.util.find_spec(package) is None:
            return "dependency_unavailable:" + package
    return ""


def load_local_model(model_path):
    """Load and warm up before exposing the capture service; no microphone input."""
    import numpy as np
    import torch
    import whisper
    torch.set_num_threads(2)
    model = whisper.load_model(model_path, device="cpu")
    silence = np.zeros(16000, dtype=np.float32)
    try:
        model.transcribe(silence, language="es", task="transcribe", fp16=False,
                         condition_on_previous_text=False, verbose=None)
    finally:
        silence.fill(0)
    return model


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from std_msgs.msg import String
    from hri_capability_interfaces.msg import SpeechCaptureStatus, SpeechTranscript
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
            self.stream = None
            self.status_at = 0.0
            self.format_error = "audio_info_unavailable"
            self.completed = queue.Queue(maxsize=1)
            self.thread = None
            self.audio_subscription = None
            self.audio_after_ns = 0
            self.inference_deadline = 0.0
            self.model = None
            self.model_error = "model_not_ready"
            self.publisher = self.create_publisher(SpeechTranscript, "/whisper/transcript", 10)
            self.status_publisher = self.create_publisher(SpeechCaptureStatus, "/whisper/capture_status", 1)
            self.create_subscription(String, "/whisper/lease", self.on_lease, 1)
            self.create_subscription(AudioInfo, self.get_parameter("audio_info_topic").value,
                                     self.on_info, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_timer(.05, self.poll)

        def prepare(self):
            self.model_error = preflight(self.model_path)
            if not self.model_error:
                self.get_logger().info("model_loading: cargando y preparando Whisper local en CPU")
                started = time.monotonic()
                try:
                    self.model = load_local_model(self.model_path)
                except Exception:
                    self.model_error = "model_initialization_failed"
                else:
                    self.get_logger().info(
                        f"model_ready: Whisper listo en {time.monotonic() - started:.1f} s; esperando solicitud")
            if self.model_error:
                self.get_logger().warning(self.model_error + "; revisar modelo y dependencias del worker")
            # Discovery during startup yields explicit non-admission from the proxy.
            # No accepted capture or lease can be interrupted by cold model loading.
            self.create_service(TranscribeSpeech, "/whisper/transcribe", self.on_request)

        def cancel_capture(self, reason):
            capture = self.session.capture
            if capture is not None or self.session.inflight or self.stream is not None:
                size = len(capture.data) if capture is not None else 0
                self.get_logger().warning(f"capture_cancelled: {reason}; buffered_bytes={size}")
            self.session.clear()
            if self.stream is not None:
                self.stream.clear()
                self.stream = None
            self.stop_audio()

        def on_lease(self, message):
            now = time.monotonic()
            if message.data != self.session.owner or not self.session.alive(now):
                self.cancel_capture("lease_changed_or_expired")
            self.session.lease(message.data, now)
            if self.session.capture is None and self.stream is None:
                self.stop_audio()

        def stop_audio(self):
            if self.audio_subscription is not None:
                self.destroy_subscription(self.audio_subscription)
                self.audio_subscription = None

        def on_info(self, message):
            previous = self.format_error
            self.format_error = validate_format(message.channels, message.sample_rate,
                                                message.sample_format, message.coding_format)
            if previous != self.format_error:
                if self.format_error:
                    self.get_logger().warning(self.format_error)
                else:
                    self.get_logger().info("audio_format_ready: PCM mono, 16000 Hz, S16LE")
            if self.format_error and (self.session.capture is not None or self.stream is not None):
                self.fail_capture(self.format_error)

        def on_request(self, request, response):
            continuous = request.continuous
            silence = request.silence_sec or .8
            error = self.model_error
            if self.input_format != "naoqi_pcm_s16le":
                error = "input_format_not_acknowledged"
            if not error:
                error = self.format_error
            if not error and self.stream is not None:
                error = "worker_busy"
            if not error and ((self.thread is not None and self.thread.is_alive()) or not self.completed.empty()):
                error = "inference_busy"
            vad = None
            if not error and continuous:
                error = validate_stream(request.duration_sec, silence)
                if not error:
                    try:
                        vad = make_vad()
                    except Exception:
                        error = "dependency_unavailable:webrtcvad"
            if not error:
                error = self.session.admit(request.request_id, request.duration_sec,
                                            request.language, time.monotonic())
            if not error:
                try:
                    if continuous:
                        self.session.capture = None
                        self.stream = SpeechStream(request.request_id, request.language,
                                                   request.duration_sec, silence, vad, time.monotonic())
                        self.status_at = 0.0
                    self.audio_after_ns = self.get_clock().now().nanoseconds
                    self.audio_subscription = self.create_subscription(
                        AudioDataStamped, self.get_parameter("audio_topic").value,
                        self.on_audio, qos_profile_sensor_data)
                except Exception:
                    self.cancel_capture("audio_subscription_failed")
                    error = "audio_subscription_failed"
            response.accepted = not bool(error)
            response.request_id = request.request_id
            response.provider = "hri_whisper_providers/WhisperTranscribeSpeech"
            response.message = error or "capture_started"
            if error:
                self.get_logger().warning("capture_rejected: " + error)
            else:
                self.get_logger().info(
                    (f"capture_started: escucha continua, pausa={silence:g} s, max_frase={request.duration_sec:g} s en "
                     if continuous else f"capture_started: escuchando {request.duration_sec:g} s en ")
                    + self.get_parameter("audio_topic").value)
            return response

        def emit(self, request_id, success, message, text="", language="", stamp_ns=0,
                 segment_index=0, final=True):
            result = SpeechTranscript()
            result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            result.request_id = request_id
            result.provider = "hri_whisper_providers/WhisperTranscribeSpeech"
            result.success, result.message = success, message
            result.text, result.language = text, language
            result.segment_index, result.final = segment_index, final
            self.publisher.publish(result)

        def fail_capture(self, reason):
            if self.stream is not None:
                self.emit(self.stream.request_id, False, reason, stamp_ns=self.stream.last_stamp)
            capture = self.session.capture
            if capture is not None:
                self.get_logger().warning(
                    f"{reason}: audio recibido={len(capture.data)}/{capture.expected_bytes} bytes")
                self.emit(capture.request_id, False, reason, stamp_ns=capture.stamp_ns)
            self.cancel_capture(reason)

        def on_audio(self, message):
            if self.session.capture is None and self.stream is None:
                return
            if not self.session.alive(time.monotonic()):
                self.cancel_capture("lease_expired")
                return
            first_audio = not self.stream.last_stamp if self.stream is not None else not self.session.capture.data
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if 0 < stamp_ns < self.audio_after_ns:
                return  # Ignore buffered samples produced before this explicit request.
            if self.stream is not None:
                try:
                    previous_drops = self.stream.dropped
                    self.stream.feed(message.audio.data, stamp_ns,
                                     self.get_clock().now().nanoseconds, time.monotonic())
                    if self.stream.dropped != previous_drops:
                        self.get_logger().warning(
                            f"speech_queue_full: descartada frase pendiente antigua; dropped={self.stream.dropped}")
                except ValueError as error:
                    self.fail_capture(str(error))
                    return
                if first_audio:
                    self.get_logger().info("audio_received: llegan muestras del microfono; VAD activo")
                self.start_next_segment()
                return
            try:
                capture = self.session.audio(message.audio.data, stamp_ns,
                                               self.get_clock().now().nanoseconds, time.monotonic())
            except ValueError as error:
                self.fail_capture(str(error))
                return
            if first_audio:
                self.get_logger().info("audio_received: llegan muestras del microfono")
            if capture is not None:
                self.stop_audio()
                self.get_logger().info(
                    f"transcribing: captura completa ({len(capture.data)} bytes); procesando Whisper")
                self.inference_deadline = time.monotonic() + 60.0
                self.thread = threading.Thread(target=self.infer, args=(capture,), daemon=True)
                self.thread.start()

        def start_next_segment(self):
            if (self.stream is None or not self.session.alive(time.monotonic())
                    or self.session.inflight or not self.completed.empty()
                    or (self.thread is not None and self.thread.is_alive())):
                return
            segment = self.stream.take()
            if segment is None:
                return
            self.session.inflight = segment.request_id
            self.inference_deadline = time.monotonic() + 60.0
            self.get_logger().info(f"transcribing: frase {segment.segment_index}; microfono sigue activo")
            self.thread = threading.Thread(target=self.infer, args=(segment,), daemon=True)
            self.thread.start()

        def infer(self, capture):
            # This thread never calls ROS; publication still checks the current lease.
            text, language, success, message = "", "", False, "inference_failed"
            audio = None
            try:
                import numpy as np
                audio = np.frombuffer(capture.data, dtype="<i2").astype(np.float32) / 32768.0
                capture.data.clear()
                if float(np.sqrt(np.mean(audio * audio))) < .005:
                    message = "no_speech_energy"
                else:
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
                index = getattr(capture, "segment_index", 0)
                self.completed.put_nowait((capture.request_id, success, message, text, language,
                                          capture.stamp_ns, index, not bool(index)))

        def poll(self):
            now = time.monotonic()
            if not self.session.alive(now):
                self.cancel_capture("lease_expired")
            capture = self.session.capture
            if capture is not None and now > capture.deadline:
                self.fail_capture("audio_capture_timeout")
            if self.stream is not None and now - self.stream.last_audio_at > 2.0:
                self.fail_capture("audio_capture_timeout")
            if self.session.inflight and now > self.inference_deadline:
                self.get_logger().warning("inference_timeout: Whisper no termino en 60 s")
                self.emit(self.session.inflight, False, "inference_timeout")
                self.cancel_capture("inference_timeout")
            try:
                result = self.completed.get_nowait()
            except queue.Empty:
                result = None
            if result is not None:
                if self.session.accepts_result(result[0], now):
                    if self.stream is not None and result[2] == "inference_failed":
                        self.fail_capture("inference_failed")
                    else:
                        self.emit(*result)
                        self.get_logger().info("transcription_result: " + result[2])
                        self.session.inflight = ""
                else:
                    self.get_logger().warning("result_discarded: solicitud cancelada o autorizacion vencida")
            self.start_next_segment()
            if self.stream is not None and now >= self.status_at:
                status = SpeechCaptureStatus()
                stamp = self.get_clock().now().nanoseconds
                status.header.stamp.sec, status.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                status.request_id = self.stream.request_id
                status.provider = "hri_whisper_providers/WhisperTranscribeSpeech"
                status.state = ("speech_and_processing" if self.stream.active else "processing") if self.session.inflight else (
                    "speech" if self.stream.active else "listening")
                status.queued_segments, status.dropped_segments = len(self.stream.pending), self.stream.dropped
                self.status_publisher.publish(status)
                self.status_at = now + .25

    rclpy.init(args=args)
    node = WhisperWorker()
    try:
        node.prepare()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cancel_capture("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
