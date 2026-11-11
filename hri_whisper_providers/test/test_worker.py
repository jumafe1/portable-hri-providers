"""Worker callback tests with fake ROS; do not require microphones or model weights."""
import pathlib
import importlib.util
import sys
import types
import unittest
import threading
from unittest.mock import Mock, patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
import whisper_worker
from transcription_core import Capture

# Import before patch.dict(sys.modules) so NumPy's extension modules are not unloaded between tests.
if importlib.util.find_spec("numpy"):
    import numpy


class Transcript:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0))


class Node:
    def __init__(self, _name):
        self.params = {}
        self.subscriptions = []
        self.messages = []
        self.status_messages = []
        self.logs = []
        self.services = []

    def declare_parameter(self, key, value):
        self.params[key] = value

    def get_parameter(self, key):
        return types.SimpleNamespace(value=self.params[key])

    def create_subscription(self, _type, topic, callback, _qos):
        self.subscriptions.append(topic)
        return topic

    def destroy_subscription(self, subscription):
        self.subscriptions.remove(subscription)

    def create_publisher(self, *_args):
        target = self.status_messages if "capture_status" in _args[1] else self.messages
        return types.SimpleNamespace(publish=target.append)

    def create_service(self, *_args):
        self.services.append(_args[1])

    def create_timer(self, *_args):
        pass

    def get_logger(self):
        return types.SimpleNamespace(warning=self.logs.append, info=self.logs.append)

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=1_000_000_000))

    def destroy_node(self):
        pass


class WorkerTests(unittest.TestCase):
    def run_worker(self, scenario, loader=None):
        modules = {
            "rclpy": types.SimpleNamespace(init=lambda **_kw: None, spin=scenario,
                                            ok=lambda: True, shutdown=lambda: None),
            "rclpy.node": types.SimpleNamespace(Node=Node),
            "rclpy.executors": types.SimpleNamespace(ExternalShutdownException=type("ExternalShutdownException", (Exception,), {})),
            "rclpy.qos": types.SimpleNamespace(DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1),
                                                QoSProfile=lambda **_kw: None, qos_profile_sensor_data=None),
            "std_msgs.msg": types.SimpleNamespace(String=object),
            "hri_capability_interfaces.msg": types.SimpleNamespace(SpeechTranscript=Transcript, SpeechCaptureStatus=Transcript),
            "hri_capability_interfaces.srv": types.SimpleNamespace(TranscribeSpeech=object),
            "audio_common_msgs.msg": types.SimpleNamespace(AudioDataStamped=object, AudioInfo=object),
        }
        with patch.dict(sys.modules, modules), patch.object(whisper_worker, "preflight", return_value=""), \
                patch.object(whisper_worker, "load_local_model", loader or Mock()), \
                patch.object(whisper_worker.time, "monotonic", return_value=0):
            whisper_worker.main()

    def prepare(self, node):
        node.input_format = "naoqi_pcm_s16le"
        node.on_info(types.SimpleNamespace(channels=1, sample_rate=16000,
                                           sample_format="S16LE", coding_format="wave"))
        node.on_lease(types.SimpleNamespace(data="owner"))

    def request(self, node):
        return node.on_request(types.SimpleNamespace(request_id="owner:1", duration_sec=.5,
                                                     language="es", continuous=False, silence_sec=0.0), types.SimpleNamespace())

    def test_microphone_only_subscribed_during_capture(self):
        def scenario(node):
            self.assertNotIn("/nao/mic_stamped", node.subscriptions)
            self.prepare(node)
            self.assertNotIn("/nao/mic_stamped", node.subscriptions)
            self.assertTrue(self.request(node).accepted)
            self.assertIn("/nao/mic_stamped", node.subscriptions)
            node.on_lease(types.SimpleNamespace(data=""))
            self.assertNotIn("/nao/mic_stamped", node.subscriptions)
            self.assertIsNone(node.session.capture)
        self.run_worker(scenario)

    def test_requires_explicit_format_and_metadata(self):
        def scenario(node):
            self.assertEqual(self.request(node).message, "input_format_not_acknowledged")
            node.input_format = "naoqi_pcm_s16le"
            self.assertEqual(self.request(node).message, "audio_info_unavailable")
            self.prepare(node)
            node.on_info(types.SimpleNamespace(channels=4, sample_rate=16000,
                                               sample_format="S16LE", coding_format="wave"))
            self.assertEqual(self.request(node).message, "unsupported_audio_format")
            self.assertNotIn("/nao/mic_stamped", node.subscriptions)
        self.run_worker(scenario)

    def test_bad_pcm_fails_without_inference(self):
        def scenario(node):
            self.prepare(node)
            self.request(node)
            node.infer = Mock()
            node.on_audio(types.SimpleNamespace(header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=1, nanosec=0)), audio=types.SimpleNamespace(data=b"1")))
            self.assertEqual(node.messages[-1].message, "malformed_pcm")
            self.assertFalse(node.messages[-1].success)
            node.infer.assert_not_called()
            self.assertNotIn("/nao/mic_stamped", node.subscriptions)
        self.run_worker(scenario)

    def test_expired_lease_discards_late_result(self):
        def scenario(node):
            self.prepare(node)
            node.session.inflight = "owner:1"
            node.inference_deadline = 100
            node.completed.put(("owner:1", True, "transcribed", "test", "es", 1_000_000_000))
            with patch.object(whisper_worker.time, "monotonic", return_value=1):
                node.poll()
            self.assertEqual(node.messages, [])
        self.run_worker(scenario)

    def test_current_result_emits_once_with_capture_stamp(self):
        def scenario(node):
            self.prepare(node)
            node.session.inflight = "owner:1"
            node.inference_deadline = 100
            node.completed.put(("owner:1", True, "transcribed", "test", "es", 1_000_000_123))
            node.poll()
            node.poll()
            self.assertEqual(len(node.messages), 1)
            self.assertEqual(node.messages[0].header.stamp.nanosec, 123)
            self.assertEqual(node.messages[0].text, "test")
            self.assertEqual(node.session.inflight, "")
        self.run_worker(scenario)

    def test_pre_request_audio_is_not_captured(self):
        def scenario(node):
            self.prepare(node)
            self.request(node)
            node.on_audio(types.SimpleNamespace(header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=0, nanosec=999_999_999)),
                audio=types.SimpleNamespace(data=b"12")))
            self.assertEqual(node.session.capture.data, bytearray())
            self.assertEqual(node.messages, [])
        self.run_worker(scenario)

    @unittest.skipUnless(importlib.util.find_spec("numpy"), "numpy is available in ROS container")
    def test_pcm_normalization_and_local_model_call(self):
        def scenario(node):
            model = Mock()
            observed_levels = []

            def transcribe(audio, **_kwargs):
                observed_levels.append((float(audio.min()), float(audio.max())))
                return {"text": " synthetic text ", "language": "es"}

            model.transcribe.side_effect = transcribe
            loader = Mock(return_value=model)
            node.model_path = "/explicit/local/model.pt"
            node.model = model
            capture = Capture("owner:1", "es", 16000, 10, bytearray(b"\x00\x40" * 8000), 1)
            with patch.dict(sys.modules, {"whisper": types.SimpleNamespace(load_model=loader),
                                          "torch": types.SimpleNamespace(set_num_threads=Mock())}):
                node.infer(capture)
            loader.assert_not_called()  # Capture uses the already loaded model.
            args, kwargs = model.transcribe.call_args
            self.assertEqual(len(args[0]), 8000)
            self.assertEqual(observed_levels, [(0.5, 0.5)])
            # Worker erases its float input after inference completes.
            self.assertTrue((args[0] == 0).all())
            self.assertFalse(kwargs["condition_on_previous_text"])
            self.assertEqual(kwargs["task"], "transcribe")
            self.assertEqual(capture.data, bytearray())
            result = node.completed.get_nowait()
            self.assertTrue(result[1])
            self.assertEqual(result[3], "synthetic text")
        self.run_worker(scenario)

    @unittest.skipUnless(importlib.util.find_spec("numpy"), "numpy is available in ROS container")
    def test_silence_never_calls_model(self):
        def scenario(node):
            loader = Mock()
            capture = Capture("owner:1", "es", 16000, 10, bytearray(16000), 1)
            with patch.dict(sys.modules, {"whisper": types.SimpleNamespace(load_model=loader),
                                          "torch": types.SimpleNamespace(set_num_threads=Mock())}):
                node.infer(capture)
            loader.assert_not_called()
            result = node.completed.get_nowait()
            self.assertFalse(result[1])
            self.assertEqual(result[2], "no_speech_energy")
        self.run_worker(scenario)

    def test_startup_loads_before_service_and_never_captures(self):
        model = Mock()
        loader = Mock(return_value=model)
        def scenario(node):
            loader.assert_called_once_with(node.model_path)
            self.assertIs(node.model, model)
            self.assertEqual(node.services, ["/whisper/transcribe"])
            self.assertNotIn("/nao/mic_stamped", node.subscriptions)
            self.assertIsNone(node.session.capture)
            self.assertTrue(any("model_ready" in line for line in node.logs))
        self.run_worker(scenario, loader)

    def test_failed_model_is_reported_and_requests_cannot_capture(self):
        def scenario(node):
            self.prepare(node)
            response = self.request(node)
            self.assertFalse(response.accepted)
            self.assertEqual(response.message, "model_initialization_failed")
            self.assertNotIn("/nao/mic_stamped", node.subscriptions)
            self.assertNotIn("PRIVATE_EXCEPTION", str(node.logs))
        self.run_worker(scenario, Mock(side_effect=RuntimeError("PRIVATE_EXCEPTION")))

    def test_lease_loss_is_visible_and_never_logs_transcript(self):
        def scenario(node):
            self.prepare(node)
            self.request(node)
            node.session.capture.data.extend(b"12")
            with patch.object(whisper_worker.time, "monotonic", return_value=1):
                node.poll()
            self.assertTrue(any("lease_expired" in line for line in node.logs))
            self.assertIsNone(node.session.capture)
            self.assertNotIn("/nao/mic_stamped", node.subscriptions)
            node.completed.put(("owner:1", True, "transcribed", "PRIVATE_TEXT", "es", 1))
            node.poll()
            self.assertEqual(node.messages, [])
            self.assertTrue(any("result_discarded" in line for line in node.logs))
            self.assertNotIn("PRIVATE_TEXT", str(node.logs))
        self.run_worker(scenario)

    def test_missing_audio_reports_zero_bytes(self):
        def scenario(node):
            self.prepare(node)
            self.request(node)
            for now in (.4, .8, 1.2, 1.6):
                with patch.object(whisper_worker.time, "monotonic", return_value=now):
                    node.on_lease(types.SimpleNamespace(data="owner"))
                    node.poll()
            self.assertEqual(node.messages[-1].message, "audio_capture_timeout")
            self.assertTrue(any("audio recibido=0/16000" in line for line in node.logs))
            self.assertNotIn("/nao/mic_stamped", node.subscriptions)
        self.run_worker(scenario)

    @unittest.skipUnless(importlib.util.find_spec("numpy"), "numpy required")
    def test_two_requested_captures_reuse_model_and_publish_each_result(self):
        model = Mock()
        model.transcribe.return_value = {"text": "PRIVATE_TEXT", "language": "es"}
        loader = Mock(return_value=model)
        def scenario(node):
            self.prepare(node)
            for sequence in (1, 2):
                request = types.SimpleNamespace(request_id=f"owner:{sequence}", duration_sec=.5, language="es",
                                                continuous=False, silence_sec=0.0)
                self.assertTrue(node.on_request(request, types.SimpleNamespace()).accepted)
                node.on_audio(types.SimpleNamespace(header=types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=1, nanosec=0)),
                    audio=types.SimpleNamespace(data=b"\x00\x40" * 8000)))
                node.thread.join(timeout=2)
                self.assertFalse(node.thread.is_alive())
                node.poll()
                self.assertNotIn("/nao/mic_stamped", node.subscriptions)
            self.assertEqual(len(node.messages), 2)
            self.assertTrue(all(message.success for message in node.messages))
            self.assertEqual(model.transcribe.call_count, 2)
            loader.assert_called_once()
            self.assertNotIn("PRIVATE_TEXT", str(node.logs))
        self.run_worker(scenario, loader)

    def stream_request(self, node):
        from test_speech_stream import FakeVad
        with patch.object(whisper_worker, "make_vad", return_value=FakeVad()):
            return node.on_request(types.SimpleNamespace(
                request_id="owner:1", duration_sec=10.0, language="es", continuous=True,
                silence_sec=.8), types.SimpleNamespace())

    def stream_audio(self, node, voice, frames=1):
        for _ in range(frames):
            node.test_stamp += 20_000_000
            node.on_audio(types.SimpleNamespace(header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=node.test_stamp // 1_000_000_000,
                                            nanosec=node.test_stamp % 1_000_000_000)),
                audio=types.SimpleNamespace(data=(b"\0\x40" if voice else b"\0\0") * 320)))

    def prepare_stream(self, node):
        self.prepare(node)
        node.test_stamp = 1_000_000_000
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=node.test_stamp))
        self.assertTrue(self.stream_request(node).accepted)

    def test_stream_captures_during_slow_inference_and_cancels_queue(self):
        entered, finish = threading.Event(), threading.Event()
        model = Mock()
        def transcribe(*_args, **_kwargs):
            entered.set()
            if not finish.wait(3):
                raise RuntimeError("test deadline")
            return {"text": "PRIVATE_LATE_RESULT", "language": "es"}
        model.transcribe.side_effect = transcribe
        def scenario(node):
            self.prepare_stream(node)
            try:
                self.stream_audio(node, True, 10)
                self.stream_audio(node, False, 40)
                self.assertTrue(entered.wait(1))
                self.assertIn("/nao/mic_stamped", node.subscriptions)
                self.stream_audio(node, True, 10)
                self.stream_audio(node, False, 40)
                queued = node.stream.pending[0].data
                self.assertTrue(queued)
                node.poll()
                self.assertEqual(node.status_messages[-1].queued_segments, 1)
                node.on_lease(types.SimpleNamespace(data=""))
                self.assertIsNone(node.stream)
                self.assertFalse(queued)
                self.assertNotIn("/nao/mic_stamped", node.subscriptions)
            finally:
                finish.set()
                node.thread.join(timeout=3)
            node.poll()
            self.assertEqual(node.messages, [])
            self.assertNotIn("PRIVATE_LATE_RESULT", str(node.logs))
        self.run_worker(scenario, Mock(return_value=model))

    def test_stream_emits_multiple_ordered_results_without_new_admission(self):
        model = Mock()
        model.transcribe.return_value = {"text": "test", "language": "es"}
        def scenario(node):
            self.prepare_stream(node)
            for _ in range(2):
                self.stream_audio(node, True, 10)
                self.stream_audio(node, False, 40)
                node.thread.join(timeout=2)
                node.poll()
                self.assertIn("/nao/mic_stamped", node.subscriptions)
            self.assertEqual([r.segment_index for r in node.messages], [1, 2])
            self.assertTrue(all(not r.final and r.request_id == "owner:1" for r in node.messages))
            self.assertEqual(model.transcribe.call_count, 2)
        self.run_worker(scenario, Mock(return_value=model))

    def test_stream_silence_keeps_heartbeat_without_empty_transcripts(self):
        def scenario(node):
            self.prepare_stream(node)
            self.stream_audio(node, False, 200)
            node.poll()
            self.assertEqual(node.messages, [])
            self.assertEqual(node.status_messages[-1].state, "listening")
            node.model.transcribe.assert_not_called()
        self.run_worker(scenario)

    def test_stream_lost_audio_is_terminal_and_clears_subscription(self):
        def scenario(node):
            self.prepare_stream(node)
            for n in range(1, 12):
                with patch.object(whisper_worker.time, "monotonic", return_value=n * .2):
                    node.on_lease(types.SimpleNamespace(data="owner"))
                    node.poll()
            self.assertIsNone(node.stream)
            self.assertEqual(node.messages[-1].message, "audio_capture_timeout")
            self.assertTrue(node.messages[-1].final)
            self.assertNotIn("/nao/mic_stamped", node.subscriptions)
        self.run_worker(scenario)


@unittest.skipUnless(importlib.util.find_spec("numpy"), "numpy required")
class ModelPreparationTests(unittest.TestCase):
    def test_loads_local_file_and_warms_up_with_synthetic_silence(self):
        model, torch = Mock(), types.SimpleNamespace(set_num_threads=Mock())
        loader = Mock(return_value=model)
        with patch.dict(sys.modules, {"whisper": types.SimpleNamespace(load_model=loader), "torch": torch}):
            self.assertIs(whisper_worker.load_local_model("/local/base.pt"), model)
        loader.assert_called_once_with("/local/base.pt", device="cpu")
        torch.set_num_threads.assert_called_once_with(2)
        audio = model.transcribe.call_args.args[0]
        self.assertEqual(len(audio), 16000)
        self.assertTrue((audio == 0).all())
        self.assertFalse(model.transcribe.call_args.kwargs["fp16"])


if __name__ == "__main__":
    unittest.main()
