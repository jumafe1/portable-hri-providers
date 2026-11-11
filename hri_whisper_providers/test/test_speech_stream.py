"""Offline phrase segmentation, framing and memory limits; no microphone access."""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
from speech_stream import SpeechStream, validate_stream


class FakeVad:
    def is_speech(self, data, rate):
        assert rate == 16000 and len(data) == 640
        return data[:2] != b"\0\0"


class StreamTests(unittest.TestCase):
    def setUp(self):
        self.stream = SpeechStream("owner:1", "es", 10, .8, FakeVad(), 0)
        self.stamp = 1_000_000_000

    def feed(self, count, voice=True, stream=None):
        target = stream or self.stream
        for _ in range(count):
            self.stamp += 20_000_000
            target.feed((b"\x00\x40" if voice else b"\0\0") * 320,
                        self.stamp, self.stamp, self.stamp / 1e9)

    def phrase(self):
        self.feed(10)
        self.feed(40, False)

    def test_silence_has_bounded_preroll_and_no_inference_jobs(self):
        self.feed(3000, False)
        self.assertFalse(self.stream.pending)
        self.assertFalse(self.stream.data)
        self.assertEqual(sum(map(len, self.stream.preroll)), 6400)

    def test_endpoint_waits_800ms_and_keeps_initial_speech(self):
        self.feed(10, False)
        self.feed(20)
        self.feed(39, False)
        self.assertFalse(self.stream.pending)
        self.feed(1, False)
        segment = self.stream.take()
        self.assertEqual(segment.data.count(b"\x00\x40"), 20 * 320)
        self.assertEqual(segment.segment_index, 1)
        self.assertEqual(segment.stamp_ns, self.stamp)
        self.assertFalse(self.stream.active)

    def test_short_pause_does_not_cut_sentence(self):
        self.feed(20)
        self.feed(20, False)
        self.feed(20)
        self.feed(40, False)
        self.assertEqual(len(self.stream.pending), 1)
        self.assertEqual(self.stream.take().data.count(b"\x00\x40"), 40 * 320)

    def test_clicks_do_not_trigger_transcription(self):
        for _ in range(20):
            self.feed(1)
            self.feed(10, False)
        self.assertFalse(self.stream.pending)
        self.feed(3)
        self.feed(40, False)
        self.assertFalse(self.stream.pending)

    def test_maximum_segment_and_continuation_have_no_gaps(self):
        self.feed(1000)
        self.assertEqual([len(s.data) for s in self.stream.pending], [320000, 320000])
        self.assertEqual([s.segment_index for s in self.stream.pending], [1, 2])
        self.feed(5)
        self.feed(40, False)
        self.assertEqual(self.stream.pending[-1].data.count(b"\x00\x40"), 5 * 320)

    def test_queue_overflow_wipes_oldest_pending_not_current_job(self):
        self.phrase()
        inflight = self.stream.take()
        self.phrase()
        oldest = self.stream.pending[0]
        self.phrase()
        self.phrase()
        self.assertEqual(self.stream.dropped, 1)
        self.assertEqual([s.segment_index for s in self.stream.pending], [3, 4])
        self.assertFalse(oldest.data)
        self.assertTrue(inflight.data)

    def test_fragmented_pcm_equals_complete_frames(self):
        audio = b"\x00\x40" * 320 * 10 + b"\0\0" * 320 * 40
        for offset in range(0, len(audio), 158):
            self.stamp += 1_000_000
            self.stream.feed(audio[offset:offset+158], self.stamp, self.stamp, self.stamp / 1e9)
        self.assertEqual(bytes(self.stream.take().data), audio)
        self.assertFalse(self.stream.partial)

    def test_cancel_discards_all_owned_buffers(self):
        self.phrase()
        queued = self.stream.pending[0].data
        self.feed(10)
        current = self.stream.data
        self.stream.clear()
        self.assertFalse(queued)
        self.assertFalse(current)
        self.assertFalse(self.stream.pending)
        self.assertFalse(self.stream.active)

    def test_invalid_audio_and_timestamp_discontinuity(self):
        with self.assertRaisesRegex(ValueError, "malformed_pcm"):
            self.stream.feed(b"a", self.stamp, self.stamp, 0)
        with self.assertRaisesRegex(ValueError, "stale_or_future_audio"):
            self.stream.feed(b"00", self.stamp, self.stamp + 600_000_000, 0)
        self.feed(1)
        with self.assertRaisesRegex(ValueError, "nonmonotonic_audio"):
            self.stream.feed(b"00", self.stamp, self.stamp, 0)
        stamp = self.stamp + 3_000_000_000
        with self.assertRaisesRegex(ValueError, "audio_capture_timeout"):
            self.stream.feed(b"00", stamp, stamp, stamp / 1e9)

    def test_invalid_stream_limits(self):
        for duration, silence in ((.5, .8), (11, .8), (10, float("nan")), (10, .1), (1, 1)):
            self.assertTrue(validate_stream(duration, silence))
        self.assertEqual(validate_stream(10, .8), "")


if __name__ == "__main__":
    unittest.main()
