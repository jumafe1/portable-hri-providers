"""Bounded, in-memory admission/capture state; no ROS or model dependency."""

from dataclasses import dataclass
import math


def validate_request(duration, language):
    if not math.isfinite(duration) or not 0.5 <= duration <= 10.0:
        return "duration_out_of_range"
    if language not in ("", "es", "en"):
        return "unsupported_language"
    return ""


def validate_format(channels, sample_rate, sample_format, coding_format):
    # The inspected NAOqi adapter labels raw PCM as `wave`; it has no WAV header.
    if (channels, sample_rate, sample_format, coding_format) != (1, 16000, "S16LE", "wave"):
        return "unsupported_audio_format"
    return ""


@dataclass
class Capture:
    request_id: str
    language: str
    expected_bytes: int
    deadline: float
    data: bytearray
    stamp_ns: int = 0


class CaptureSession:
    """One current owner and request; a lost lease irreversibly drops its audio."""

    def __init__(self):
        self.owner = ""
        self.lease_at = -math.inf
        self.capture = None
        self.inflight = ""

    def lease(self, owner, now):
        if owner != self.owner or now - self.lease_at > 0.5:
            self.clear()
        self.owner = owner
        self.lease_at = now

    def alive(self, now):
        return bool(self.owner) and 0.0 <= now - self.lease_at <= 0.5

    def clear(self):
        if self.capture is not None:
            self.capture.data.clear()
        self.capture = None
        self.inflight = ""

    def admit(self, request_id, duration, language, now):
        error = validate_request(duration, language)
        if error:
            return error
        if not self.alive(now) or not request_id.startswith(self.owner + ":"):
            return "lease_unavailable"
        if self.capture is not None or self.inflight:
            return "worker_busy"
        self.capture = Capture(request_id, language, int(duration * 16000) * 2,
                               now + duration + 1.0, bytearray())
        return ""

    def audio(self, payload, stamp_ns, now_ros_ns, now):
        if not self.alive(now):
            self.clear()
            return None
        capture = self.capture
        if capture is None:
            return None
        if now > capture.deadline:
            raise ValueError("audio_capture_timeout")
        if not payload or len(payload) % 2 or len(payload) > 64000:
            raise ValueError("malformed_pcm")
        age_ns = now_ros_ns - stamp_ns
        if stamp_ns <= 0 or not -100_000_000 <= age_ns <= 500_000_000:
            raise ValueError("stale_or_future_audio")
        if capture.stamp_ns and stamp_ns <= capture.stamp_ns:
            raise ValueError("nonmonotonic_audio")
        capture.stamp_ns = stamp_ns
        remaining = capture.expected_bytes - len(capture.data)
        capture.data.extend(payload[:remaining])
        if len(capture.data) < capture.expected_bytes:
            return None
        self.capture = None
        self.inflight = capture.request_id
        return capture

    def accepts_result(self, request_id, now):
        return self.alive(now) and request_id == self.inflight
