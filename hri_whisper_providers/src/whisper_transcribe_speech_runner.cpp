#include <hri_speech_providers/transcribe_runner.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace hri_whisper_providers {
class WhisperTranscribeSpeechRunner : public hri_speech_providers::TranscribeSpeechRunner {
public:
  WhisperTranscribeSpeechRunner() : TranscribeSpeechRunner("/whisper") {}
};
}
PLUGINLIB_EXPORT_CLASS(hri_whisper_providers::WhisperTranscribeSpeechRunner, capabilities2_runner::RunnerBase)
