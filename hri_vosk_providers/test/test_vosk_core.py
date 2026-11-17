import json
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from vosk_core import RecognizerStream

class StreamTest(unittest.TestCase):
    def setUp(self):
        self.rec = MagicMock()
        self.factory = MagicMock(return_value=self.rec)
        self.core = RecognizerStream(self.factory)
        self.rec.AcceptWaveform.return_value = True
        self.output = {'text': 'nao detente', 'result': [
            {'word': 'nao', 'conf': .95, 'start': .01, 'end': .05},
            {'word': 'detente', 'conf': .9, 'start': .05, 'end': .09}]}
        self.rec.Result.side_effect = lambda: json.dumps(self.output)

    def feed(self, data=bytes(3200), stamp=1_000_000_000, now=1_050_000_000):
        return self.core.feed(data, stamp, now)

    def test_final_timestamp_tracks_start_of_speech_not_publication(self):
        self.assertEqual(self.feed(), ('nao detente', 910_000_000))
        self.assertEqual(self.core.bytes_received, 0)

    def test_partial_never_emits(self):
        self.rec.AcceptWaveform.return_value = False
        self.assertIsNone(self.feed())
        self.rec.Result.assert_not_called()

    def test_unknown_or_uncertain_final_is_discarded(self):
        self.output['result'][0]['conf'] = .4
        self.assertIsNone(self.feed())
        self.output['result'][0]['conf'] = .9
        self.output['text'] = '[unk] nao detente'
        self.assertIsNone(self.feed(stamp=1_100_000_000, now=1_150_000_000))

    def test_stale_future_replayed_malformed_input_resets_decoder(self):
        for options in ({'data': b'1'}, {'data': bytes(64000)}, {'stamp': 100}, {'stamp': 2_000_000_000}):
            with self.subTest(options=options):
                self.assertIsNone(self.feed(**options))
        self.assertIsNotNone(self.feed())
        self.assertIsNone(self.feed())

    def test_gap_does_not_join_words_from_before_and_after(self):
        self.rec.AcceptWaveform.return_value = False
        self.feed()
        before = self.factory.call_count
        self.feed(stamp=2_000_000_000, now=2_050_000_000)
        self.assertGreater(self.factory.call_count, before)

    def test_audio_buffer_and_decoding_window_are_bounded(self):
        self.rec.AcceptWaveform.return_value = False
        for i in range(100):
            self.feed(stamp=1_000_000_000+i*100_000_000, now=1_050_000_000+i*100_000_000)
        self.assertGreater(self.factory.call_count, 1)
        self.assertLess(self.core.bytes_received, self.core.max_bytes)

if __name__ == '__main__': unittest.main()
