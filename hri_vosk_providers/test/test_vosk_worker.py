"""Admission, lease revocation and PCM validation with real ROS messages, no DDS."""
from contextlib import ExitStack
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import vosk_worker
from hri_capability_interfaces.srv import TranscribeSpeech
from audio_common_msgs.msg import AudioDataStamped, AudioInfo
from std_msgs.msg import String


class WorkerTest(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.tmp = self.stack.enter_context(tempfile.TemporaryDirectory())
        (Path(self.tmp)/'am').mkdir()
        (Path(self.tmp)/'am/final.mdl').touch()
        self.now, self.ros = 10., 100_000_000_000
        self.stack.enter_context(patch.object(vosk_worker.time, 'monotonic', side_effect=lambda: self.now))
        mock_vosk = MagicMock()
        mock_vosk.Model.return_value.vosk_model_find_word.return_value = 1
        self.stack.enter_context(patch.dict(sys.modules, {'vosk': mock_vosk}))
        owner = self
        class Node:
            def __init__(self, _):
                self.parameters = {'model_path': owner.tmp, 'grammar': '["nao detente", "[unk]"]',
                                   'input_format': 'naoqi_pcm_s16le'}
                self.create_publisher = MagicMock(side_effect=lambda *_: MagicMock())
                self.create_subscription = MagicMock(side_effect=lambda *_: object())
                self.destroy_subscription = MagicMock()
                self.create_service, self.create_timer = MagicMock(), MagicMock()
            def declare_parameter(self, name, default):
                self.parameters.setdefault(name, default)
            def get_parameter(self, name):
                return SimpleNamespace(value=self.parameters[name])
            def get_clock(self):
                return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=owner.ros, to_msg=MagicMock()))
        self.stack.enter_context(patch('rclpy.node.Node', Node))
        self.worker = vosk_worker.worker_class()()
        self.worker.on_info(AudioInfo(channels=1, sample_rate=16000, sample_format='S16LE', coding_format='wave'))

    def admit(self, request_id='owner:1', continuous=True):
        return self.worker.admit(TranscribeSpeech.Request(request_id=request_id, duration_sec=8.,
            continuous=continuous, language='es'), TranscribeSpeech.Response())

    def ready(self):
        self.worker.on_lease(String(data='owner'))
        self.assertTrue(self.admit().accepted)

    def audio(self, payload, stamp=None):
        msg = AudioDataStamped()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp or self.ros, 1_000_000_000)
        msg.audio.data = list(payload)
        return msg

    def test_no_microphone_subscription_before_live_matching_lease(self):
        self.assertIsNone(self.worker.subscription)
        self.assertFalse(self.admit().accepted)
        self.worker.on_lease(String(data='owner'))
        self.assertFalse(self.admit(request_id='other:1').accepted)
        self.assertIsNone(self.worker.subscription)
        self.assertTrue(self.admit().accepted)
        self.assertIsNotNone(self.worker.subscription)
        self.assertEqual(self.admit().message, 'worker_busy')

    def test_invalid_audio_info_prevents_capture(self):
        self.worker.on_lease(String(data='owner'))
        self.worker.on_info(AudioInfo(channels=4, sample_rate=48000))
        self.assertEqual(self.admit().message, 'audio_info_unavailable')
        self.assertIsNone(self.worker.subscription)

    def test_expired_or_changed_lease_destroys_subscription_and_drops_result(self):
        self.ready()
        self.now += .6
        self.worker.audio(self.audio(bytes(3200)))
        self.assertIsNone(self.worker.subscription)
        self.assertFalse(self.worker.request_id)
        self.worker.publisher.publish.assert_not_called()
        self.ready()
        self.worker.on_lease(String(data='new'))
        self.assertIsNone(self.worker.subscription)

    def test_malformed_audio_ends_stream(self):
        self.ready()
        self.worker.audio(self.audio(b'1'))
        result = self.worker.publisher.publish.call_args.args[0]
        self.assertEqual(result.message, 'malformed_pcm')
        self.assertTrue(result.final)
        self.assertIsNone(self.worker.subscription)

    def test_audio_timeout_is_not_hidden_by_status_heartbeats(self):
        self.ready()
        self.now += 3.1
        self.worker.on_lease(String(data='owner'))  # A gap expires the old capture.
        self.assertFalse(self.worker.request_id)
        self.assertTrue(self.admit().accepted)
        self.now += 3.1
        self.worker.lease_at = self.now  # Proxy alive, microphone absent.
        self.worker.tick()
        self.assertEqual(self.worker.publisher.publish.call_args.args[0].message, 'audio_capture_timeout')
        self.assertIsNone(self.worker.subscription)

    def test_final_results_preserve_request_identity_and_audio_timestamp(self):
        self.ready()
        self.worker.stream = MagicMock(last_stamp=0)
        self.worker.stream.feed.return_value = ('nao detente', self.ros-50_000_000)
        self.worker.audio(self.audio(bytes(3200)))
        msg = self.worker.publisher.publish.call_args.args[0]
        self.assertEqual(msg.request_id, 'owner:1')
        self.assertEqual(msg.provider, 'hri_vosk_providers/VoskTranscribeSpeech')
        self.assertEqual(msg.segment_index, 1)
        self.assertFalse(msg.final)
        self.assertEqual(msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec, self.ros-50_000_000)

if __name__ == '__main__': unittest.main()
