"""VAD segmentation and bounded pending audio; all mutation on the ROS thread."""

from collections import deque
from dataclasses import dataclass
import math


def validate_stream(duration, silence):
    if not math.isfinite(duration) or not 1.0 <= duration <= 10.0:
        return "duration_out_of_range"
    if not math.isfinite(silence) or not .2 <= silence <= 2.0 or silence >= duration:
        return "silence_out_of_range"
    return ""


def make_vad():
    import webrtcvad
    return webrtcvad.Vad(2)


@dataclass
class Segment:
    request_id: str
    language: str
    data: bytearray
    stamp_ns: int
    segment_index: int


class SpeechStream:
    """16 kHz S16LE mono, 20 ms VAD frames, 200 ms pre-roll within this request.

    One current segment (<=10 s) and two waiting segments. Inference owns a
    separate segment. On overflow discard/wipe the oldest waiting segment so
    latency cannot grow indefinitely. No samples precede explicit admission.
    """
    FRAME_BYTES = 640
    MAX_PENDING = 2

    def __init__(self, request_id, language, duration, silence, vad, now):
        error = validate_stream(duration, silence)
        if error:
            raise ValueError(error)
        self.request_id, self.language, self.vad = request_id, language, vad
        self.max_frames = int(duration / .02)
        self.end_frames = math.ceil(silence / .02)
        self.partial = bytearray()
        self.preroll = deque()
        self.window = deque(maxlen=5)
        self.data = bytearray()
        self.pending = deque()
        self.active = False
        self.voiced = self.silence = self.index = self.dropped = 0
        self.last_stamp = 0
        self.last_audio_at = now

    def clear(self):
        self.partial.clear()
        self.data.clear()
        for frame in self.preroll:
            frame.clear()
        self.preroll.clear()
        self.window.clear()
        for item in self.pending:
            item.data.clear()
        self.pending.clear()
        self.active = False
        self.voiced = self.silence = 0

    def feed(self, payload, stamp_ns, now_ros_ns, now):
        if not payload or len(payload) % 2 or len(payload) > 64000:
            raise ValueError("malformed_pcm")
        if stamp_ns <= 0 or not -100_000_000 <= now_ros_ns - stamp_ns <= 500_000_000:
            raise ValueError("stale_or_future_audio")
        if self.last_stamp and stamp_ns <= self.last_stamp:
            raise ValueError("nonmonotonic_audio")
        # Never join disconnected pieces into one utterance.
        if self.last_stamp and (stamp_ns - self.last_stamp > 2_000_000_000 or now - self.last_audio_at > 2.0):
            raise ValueError("audio_capture_timeout")
        self.last_stamp, self.last_audio_at = stamp_ns, now
        self.partial.extend(payload)
        while len(self.partial) >= self.FRAME_BYTES:
            frame = self.partial[:self.FRAME_BYTES]
            del self.partial[:self.FRAME_BYTES]
            try:
                voiced = self.vad.is_speech(bytes(frame), 16000)
            except Exception as error:
                raise ValueError("vad_failed") from error
            if not self.active:
                if len(self.preroll) == 10:
                    self.preroll.popleft().clear()
                self.preroll.append(frame)
                self.window.append(voiced)
                if sum(self.window) < 3:
                    continue
                for buffered in self.preroll:
                    self.data.extend(buffered)
                    buffered.clear()
                self.preroll.clear()
                self.active, self.voiced, self.silence = True, sum(self.window), 0
                self.window.clear()
            else:
                self.data.extend(frame)
                frame.clear()
                self.voiced += int(voiced)
                self.silence = 0 if voiced else self.silence + 1
            at_limit = len(self.data) >= self.max_frames * self.FRAME_BYTES
            if self.silence >= self.end_frames or at_limit:
                self._finish(stamp_ns)
                # Continue long speech without losing the next syllable to a new trigger.
                self.active = at_limit and voiced

    def _finish(self, stamp_ns):
        if self.voiced >= 5:  # Reject clicks/transients shorter than 100 ms of voice.
            self.index += 1
            item = Segment(self.request_id, self.language, self.data, stamp_ns, self.index)
            if len(self.pending) == self.MAX_PENDING:
                self.pending.popleft().data.clear()
                self.dropped += 1
            self.pending.append(item)
            self.data = bytearray()
        else:
            self.data.clear()
        self.active = False
        self.voiced = self.silence = 0

    def take(self):
        return self.pending.popleft() if self.pending else None
