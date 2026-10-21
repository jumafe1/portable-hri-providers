import pathlib
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
from transcription_core import CaptureSession, validate_format, validate_request
from whisper_worker import preflight


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.session = CaptureSession()
        self.session.lease("owner", 0.0)

    def test_request_validation(self):
        for value in (float("nan"), float("inf"), 0, -1, 10.1):
            self.assertTrue(validate_request(value, "es"))
        self.assertFalse(validate_request(0.5, ""))
        self.assertTrue(validate_request(1, "not_a_language"))

    def test_no_implicit_model_download(self):
        self.assertEqual(preflight("tiny"), "local_model_unavailable")
        with patch("pathlib.Path.is_file", return_value=True):
            self.assertEqual(preflight("model.bin"), "local_model_must_be_pt")
            with patch("importlib.util.find_spec", return_value=None):
                self.assertEqual(preflight("model.pt"), "dependency_unavailable:whisper")
            with patch("importlib.util.find_spec", return_value=object()):
                self.assertEqual(preflight("model.pt"), "")

    def test_format_is_explicit_mono_pcm(self):
        self.assertFalse(validate_format(1, 16000, "S16LE", "wave"))
        for args in ((4, 16000, "S16LE", "wave"), (1, 48000, "S16LE", "wave"),
                     (1, 16000, "F32LE", "wave"), (1, 16000, "S16LE", "mp3")):
            self.assertTrue(validate_format(*args))

    def test_requires_lease_and_owner(self):
        self.assertEqual(self.session.admit("other:1", 1, "es", 0), "lease_unavailable")
        self.assertEqual(self.session.admit("owner:1", 1, "es", 1), "lease_unavailable")

    def test_capture_is_bounded_and_busy(self):
        self.assertFalse(self.session.admit("owner:1", .5, "es", 0))
        self.assertEqual(self.session.admit("owner:2", .5, "es", 0), "worker_busy")
        result = self.session.audio(b"\x01\x00" * 16000, 1_000_000_000, 1_000_000_000, .1)
        self.assertEqual(len(result.data), 16000)
        self.assertTrue(self.session.accepts_result("owner:1", .1))
        self.assertEqual(self.session.admit("owner:2", .5, "es", .1), "worker_busy")

    def test_reacquire_discards_old_output(self):
        self.session.admit("owner:1", .5, "es", 0)
        self.session.audio(b"\x01\x00" * 8000, 1_000_000_000, 1_000_000_000, .1)
        self.session.lease("new", .2)
        self.assertFalse(self.session.accepts_result("owner:1", .2))

    def test_lease_expiry_discards_buffer(self):
        self.session.admit("owner:1", .5, "es", 0)
        self.session.audio(b"\x01\x00" * 10, 1_000_000_000, 1_000_000_000, .1)
        capture = self.session.capture
        self.session.audio(b"\x01\x00", 2_000_000_000, 2_000_000_000, .6)
        self.assertIsNone(self.session.capture)
        self.assertEqual(capture.data, bytearray())

    def test_bad_audio_and_stamp(self):
        self.session.admit("owner:1", .5, "es", 0)
        for data, stamp, now in ((b"1", 1, 1), (bytes(64002), 1, 1),
                                 (b"12", 0, 1), (b"12", 1, 600_000_001),
                                 (b"12", 200_000_001, 1)):
            with self.assertRaises(ValueError):
                self.session.audio(data, stamp, now, .1)

    def test_duplicate_audio_and_capture_timeout(self):
        self.session.admit("owner:1", .5, "es", 0)
        self.session.audio(b"12", 1_000_000_000, 1_000_000_000, .1)
        with self.assertRaisesRegex(ValueError, "nonmonotonic"):
            self.session.audio(b"12", 1_000_000_000, 1_000_000_000, .2)
        self.session.lease("owner", .4)
        self.session.lease("owner", .8)
        self.session.lease("owner", 1.2)
        self.session.lease("owner", 1.6)
        with self.assertRaisesRegex(ValueError, "timeout"):
            self.session.audio(b"12", 2_000_000_000, 2_000_000_000, 1.6)


if __name__ == "__main__":
    unittest.main()
