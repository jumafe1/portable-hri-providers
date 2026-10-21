"""Worker callback tests with fake ROS; do not require microphones or model weights."""
import pathlib
import importlib.util
import sys
import types
import unittest
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
        return types.SimpleNamespace(publish=self.messages.append)

    def create_service(self, *_args):
        pass

    def create_timer(self, *_args):
        pass

    def get_logger(self):
        return types.SimpleNamespace(warning=lambda _text: None)

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=1_000_000_000))

    def destroy_node(self):
        pass


class WorkerTests(unittest.TestCase):
    def run_worker(self, scenario):
        modules = {
            "rclpy": types.SimpleNamespace(init=lambda **_kw: None, spin=scenario,
                                            ok=lambda: True, shutdown=lambda: None),
            "rclpy.node": types.SimpleNamespace(Node=Node),
            "rclpy.qos": types.SimpleNamespace(DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1),
                                                QoSProfile=lambda **_kw: None, qos_profile_sensor_data=None),
            "std_msgs.msg": types.SimpleNamespace(String=object),
            "hri_capability_interfaces.msg": types.SimpleNamespace(SpeechTranscript=Transcript),
            "hri_capability_interfaces.srv": types.SimpleNamespace(TranscribeSpeech=object),
            "audio_common_msgs.msg": types.SimpleNamespace(AudioDataStamped=object, AudioInfo=object),
        }
        with patch.dict(sys.modules, modules), patch.object(whisper_worker, "preflight", return_value=""), \
                patch.object(whisper_worker.time, "monotonic", return_value=0):
            whisper_worker.main()

    def prepare(self, node):
        node.input_format = "naoqi_pcm_s16le"
        node.on_info(types.SimpleNamespace(channels=1, sample_rate=16000,
                                           sample_format="S16LE", coding_format="wave"))
        node.on_lease(types.SimpleNamespace(data="owner"))

    def request(self, node):
        return node.on_request(types.SimpleNamespace(request_id="owner:1", duration_sec=.5,
                                                     language="es"), types.SimpleNamespace())

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
            capture = Capture("owner:1", "es", 16000, 10, bytearray(b"\x00\x40" * 8000), 1)
            with patch.dict(sys.modules, {"whisper": types.SimpleNamespace(load_model=loader),
                                          "torch": types.SimpleNamespace(set_num_threads=Mock())}):
                node.infer(capture)
            loader.assert_called_once_with(node.model_path, device="cpu")
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


if __name__ == "__main__":
    unittest.main()
