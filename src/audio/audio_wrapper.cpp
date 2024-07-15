//
// Created by janne on 2024-07-01.
//

#include "audio_wrapper.h"

#include <cmath>
#include <vector>

constexpr int device = 0;

static int audioCallback(const void *_, void *outputBuffer,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo *timeInfo,
                         PaStreamCallbackFlags statusFlags,
                         void *userData) {
    auto in = (Streams *) userData;
    auto out = (float *) outputBuffer;

    (void) _;// Ignore unused variable warning for input buffer

    for (int i = 0; i < framesPerBuffer; ++i) {
        out[i] = in->buffers[0][i];
    }

    return 0;
}

AudioWrapper::AudioWrapper(Streams &streams) : AudioWrapper(streams, false){};

AudioWrapper::AudioWrapper(Streams &streams, bool debug) : _streams(streams), debug_(debug) {
    Pa_Initialize();

    if (debug_) {
        const int deviceAmt = Pa_GetDeviceCount();
        for (int i = 0; i < deviceAmt; ++i) {
            const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(i);
            const std::string deviceName = Pa_GetHostApiInfo(deviceInfo->hostApi)->name;
            std::cout << "Device: " << deviceInfo->name << " Api: " << deviceName << std::endl;
            std::cout << deviceInfo->defaultSampleRate << std::endl;
            std::cout << deviceInfo->maxOutputChannels << std::endl;
            std::cout << deviceInfo->defaultSampleRate << std::endl;
        }
    }

    PaError err = paNoError;

    PaStreamParameters out_param;
    out_param.device = Pa_GetDefaultOutputDevice();
    out_param.channelCount = 2;
    out_param.hostApiSpecificStreamInfo = nullptr;
    out_param.sampleFormat = paFloat32;
    out_param.suggestedLatency = 0;

    err = Pa_OpenStream(&audio_stream_,
                        nullptr,
                        &out_param,
                        SAMPLE_RATE,
                        256,
                        paNoFlag,
                        audioCallback,
                        &streams);

    if (err != paNoError) {
        std::cerr << "Unable to open stream, error code " << err << std::endl;
    }
}


void AudioWrapper::start_audio_playback() {
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
