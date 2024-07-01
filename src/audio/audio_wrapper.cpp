//
// Created by janne on 2024-07-01.
//

#include "audio_wrapper.h"

#include <cmath>
#include <vector>


const int channels = 1;
constexpr int device = 0;                     // 0 indicates the default or first available device
constexpr double frequency = 3 * 440.0;

static int audioCallback(const void *_, void *outputBuffer,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo *timeInfo,
                         PaStreamCallbackFlags statusFlags,
                         void *userData) {
    auto in = (Streams *) userData;
    auto out = (float *) outputBuffer;

    (void) _;// Ignore unused variable warning for input buffer

    in->read_stream(124, out, 0);

    return 0;
}

AudioWrapper::AudioWrapper(Streams &streams) : _streams(streams) {
    Pa_Initialize();
    Pa_OpenDefaultStream(&audio_stream_, 0, channels, paFloat32,
                         SAMPLE_RATE, 256, audioCallback, &streams);
}


void AudioWrapper::start_audio_playback() {
    float data[256];
    Pa_StartStream(audio_stream_);
    is_on_ = true;
}

void AudioWrapper::stop_audio_playback() { is_on_ = false; }

AudioWrapper::~AudioWrapper() {
    if (is_on_) {
        this->stop_audio_playback();
    }
    Pa_StopStream(audio_stream_);
    Pa_Terminate();
};
