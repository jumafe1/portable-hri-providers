#include <hri_speech_providers/transcribe_runner.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace hri_vosk_providers {
class VoskTranscribeSpeechRunner : public hri_speech_providers::TranscribeSpeechRunner {
public:
  VoskTranscribeSpeechRunner() : TranscribeSpeechRunner("/vosk") {}
};
}
PLUGINLIB_EXPORT_CLASS(hri_vosk_providers::VoskTranscribeSpeechRunner, capabilities2_runner::RunnerBase)
