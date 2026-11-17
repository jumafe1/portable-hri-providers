#!/usr/bin/env python3
"""Local Vosk streaming ASR. Subscribes to PCM only under a live capability lease."""
import json
import signal
import time
from pathlib import Path

from vosk_core import RecognizerStream


def worker_class():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from audio_common_msgs.msg import AudioDataStamped, AudioInfo
    from hri_capability_interfaces.msg import SpeechCaptureStatus, SpeechTranscript
    from hri_capability_interfaces.srv import TranscribeSpeech
    from std_msgs.msg import String
    from vosk import Model, KaldiRecognizer, SetLogLevel

    provider = 'hri_vosk_providers/VoskTranscribeSpeech'

    class Worker(Node):
        def __init__(self):
            super().__init__('vosk_worker')
            for key, default in {'model_path': '', 'grammar': '["[unk]"]',
                                 'minimum_word_confidence': .80, 'audio_topic': '/nao/mic_stamped',
                                 'audio_info_topic': '/nao/mic_info', 'input_format': ''}.items():
                self.declare_parameter(key, default)
            param = lambda key: self.get_parameter(key).value
            if param('input_format') != 'naoqi_pcm_s16le':
                raise RuntimeError('Set input_format:=naoqi_pcm_s16le for the inspected PCM adapter')
            path = Path(param('model_path')).expanduser()
            if not (path / 'am/final.mdl').is_file():
                raise RuntimeError('Missing local Vosk model: ' + str(path))
            grammar = json.loads(param('grammar'))
            if not isinstance(grammar, list) or not all(isinstance(w, str) for w in grammar) or '[unk]' not in grammar:
                raise RuntimeError('Grammar must contain phrases and [unk]')
            confidence = param('minimum_word_confidence')
            if not .5 <= confidence <= 1.0:
                raise RuntimeError('minimum_word_confidence must be in [.5,1]')
            SetLogLevel(-1)
            self.model = Model(str(path))  # No implicit model download.
            # Refuse an incomplete grammar: Vosk silently drops out-of-vocabulary words.
            missing = {w for phrase in grammar if phrase != '[unk]' for w in phrase.split()
                       if self.model.vosk_model_find_word(w) < 0}
            if missing:
                raise RuntimeError('Vosk model lacks configured vocabulary: ' + ', '.join(sorted(missing)))
            self.factory = lambda: KaldiRecognizer(self.model, 16000, json.dumps(grammar, ensure_ascii=False))
            self.factory()  # Preflight graph before exposing admission.
            self.confidence = confidence
            self.owner, self.request_id = '', ''
            self.lease_at = -float('inf')
            self.stream = self.subscription = None
            self.format_ok = False
            self.last_audio = self.started = 0.0
            self.audio_after = self.segment = 0
            self.publisher = self.create_publisher(SpeechTranscript, '/vosk/transcript', 10)
            self.status = self.create_publisher(SpeechCaptureStatus, '/vosk/capture_status', 1)
            self.create_subscription(String, '/vosk/lease', self.on_lease, 1)
            self.create_subscription(AudioInfo, param('audio_info_topic'), self.on_info,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_service(TranscribeSpeech, '/vosk/transcribe', self.admit)
            self.create_timer(.1, self.tick)

        def alive(self):
            return bool(self.owner) and 0 <= time.monotonic() - self.lease_at <= .5

        def clear(self):
            self.request_id, self.stream = '', None
            if self.subscription is not None:
                self.destroy_subscription(self.subscription)
                self.subscription = None

        def on_lease(self, message):
            if message.data != self.owner or not self.alive():
                self.clear()
            self.owner, self.lease_at = message.data, time.monotonic()

        def on_info(self, msg):
            self.format_ok = (msg.channels, msg.sample_rate, msg.sample_format, msg.coding_format) == (1, 16000, 'S16LE', 'wave')
            if not self.format_ok and self.request_id:
                self.fail('unsupported_audio_format')

        def admit(self, request, response):
            error = ''
            if not self.alive() or not request.request_id.startswith(self.owner + ':'):
                error = 'lease_unavailable'
            elif self.request_id:
                error = 'worker_busy'
            elif not self.format_ok:
                error = 'audio_info_unavailable'
            elif request.language not in ('es', ''):
                error = 'unsupported_language'
            elif not request.continuous:
                error = 'continuous_required'
            else:
                self.stream = RecognizerStream(self.factory, self.confidence)
                self.request_id = request.request_id
                self.segment = 0
                self.started = self.last_audio = time.monotonic()
                self.audio_after = self.get_clock().now().nanoseconds
                self.subscription = self.create_subscription(
                    AudioDataStamped, self.get_parameter('audio_topic').value, self.audio,
                    QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT))
            response.accepted, response.message = not error, error or 'capture_started'
            response.request_id, response.provider = request.request_id, provider
            return response

        def audio(self, msg):
            if not self.request_id or not self.alive():
                self.clear()
                return
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            now_ns = self.get_clock().now().nanoseconds
            if stamp < self.audio_after:
                return
            payload = msg.audio.data
            if not payload or len(payload) % 2 or len(payload) > 16000:
                self.fail('malformed_pcm')
                return
            valid_time = (-100_000_000 <= now_ns - stamp <= 500_000_000 and stamp > self.stream.last_stamp)
            try:
                result = self.stream.feed(payload, stamp, now_ns)
            except Exception:
                self.fail('inference_failed')
                return
            if valid_time:
                self.last_audio = time.monotonic()
            if result:
                text, start = result
                self.segment += 1
                transcript = SpeechTranscript(request_id=self.request_id, provider=provider, text=text,
                    language='es', success=True, message='recognized', segment_index=self.segment, final=False)
                transcript.header.stamp.sec, transcript.header.stamp.nanosec = divmod(start, 1_000_000_000)
                self.publisher.publish(transcript)

        def fail(self, reason):
            self.publisher.publish(SpeechTranscript(request_id=self.request_id, provider=provider,
                                                     success=False, message=reason, final=True))
            self.clear()

        def tick(self):
            if not self.alive():
                self.clear()
            if not self.request_id:
                return
            if time.monotonic() - self.last_audio > 3.0:
                self.fail('audio_capture_timeout')
                return
            msg = SpeechCaptureStatus(request_id=self.request_id, provider=provider, state='listening')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.status.publish(msg)

    return Worker


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    Worker = worker_class()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = Worker()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if node is not None:
            node.clear()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
