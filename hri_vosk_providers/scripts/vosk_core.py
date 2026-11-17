"""Bounded PCM stream and timestamped final results, independent of ROS."""
import json
import math


class RecognizerStream:
    def __init__(self, factory, minimum_confidence=.80, max_seconds=8.0):
        self.factory = factory
        self.minimum_confidence = minimum_confidence
        self.max_bytes = int(max_seconds * 32000)
        self.last_stamp = 0
        self.reset()

    def reset(self):
        self.recognizer = self.factory()
        self.recognizer.SetWords(True)
        self.origin = 0
        self.bytes_received = 0

    def feed(self, payload, stamp, now_ns):
        if (not payload or len(payload) % 2 or len(payload) > 16000
                or stamp <= 0 or not -100_000_000 <= now_ns - stamp <= 500_000_000
                or stamp <= self.last_stamp):
            self.reset()
            return None
        # Discontinuous or dropped input must not join two unrelated utterances.
        if self.last_stamp and stamp - self.last_stamp > 500_000_000:
            self.reset()
        self.last_stamp = stamp
        if not self.origin:
            self.origin = stamp - int(len(payload) / 32000 * 1e9)
        self.bytes_received += len(payload)
        final = self.recognizer.AcceptWaveform(bytes(payload))
        if not final:
            if self.bytes_received >= self.max_bytes:
                self.reset()  # Discard an overlong partial; never execute partial commands.
            return None
        try:
            result = json.loads(self.recognizer.Result())
            words = result.get('result', [])
            text = result.get('text', '').strip()
            if (not text or '[unk]' in text or not words or len(text) > 256
                    or any(not math.isfinite(w['conf']) or w['conf'] < self.minimum_confidence for w in words)):
                return None
            start = self.origin + int(words[0]['start'] * 1e9)
            if start <= 0 or start > stamp:
                return None
            return text, start
        finally:
            self.reset()
