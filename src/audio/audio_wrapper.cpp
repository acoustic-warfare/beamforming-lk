//
// Created by janne on 2024-07-01.
//

#include "audio_wrapper.h"

#include <cmath>
#include <iostream>
#include <vector>

constexpr int device = 0;

static int audioCallback(const void *inputBuffer, void *outputBuffer,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo *timeInfo,
                         PaStreamCallbackFlags statusFlags,
                         void *userData) {
    auto self = static_cast<AudioWrapper *>(userData);
    auto out = static_cast<float *>(outputBuffer);

    const float *in = self->_streams.buffers[4];
    for (unsigned long i = 0; i < framesPerBuffer; ++i) {
        self->audioData.push_back(in[i]);
        out[i] = in[i];
    }

    if (AUDIO_FILE) {
        // Encode data to MP3
        std::vector<short> pcmData;
        self->convertFloatToPcm16(self->audioData, pcmData);
        self->encoder_->encode_inter(pcmData.data(), pcmData.size());
    }
    return paContinue;
}

static void checkErr(PaError err) {
    if (err != paNoError) {
        std::cerr << "\nPortAudio error: " << err << std::endl;
    }
}

AudioWrapper::AudioWrapper(Streams &streams) : AudioWrapper(streams, true) {}

AudioWrapper::AudioWrapper(Streams &streams, bool debug) : _streams(streams), debug_(debug) {
    PaError err = paNoError;
    err = Pa_Initialize();
    checkErr(err);

    if (debug_) {
        const int deviceAmt = Pa_GetDeviceCount();

        if (deviceAmt <= 0) {
            std::cerr << "\nError getting device count: " << deviceAmt << ", error code: " << err << std::endl;
        }

        std::cout << "\nDeviceAmt: " << deviceAmt << std::endl;

        for (int i = 0; i < deviceAmt; ++i) {
            const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(i);
            const std::string deviceName = Pa_GetHostApiInfo(deviceInfo->hostApi)->name;
            std::cout << "\nDevice nr: " << i << " Name: " << deviceInfo->name << " Api: " << deviceName << std::endl;
            std::cout << "maxInputChannels: " << deviceInfo->defaultSampleRate << std::endl;
            std::cout << "maxOutputChannels: " << deviceInfo->maxOutputChannels << std::endl;
            std::cout << "defaultSampleRate: " << deviceInfo->defaultSampleRate << std::endl;
        }
    }

    PaStreamParameters out_param;
    out_param.device = Pa_GetDefaultOutputDevice();
    out_param.channelCount = 2;
    out_param.hostApiSpecificStreamInfo = nullptr;
    out_param.sampleFormat = paFloat32;
    out_param.suggestedLatency = Pa_GetDeviceInfo(out_param.device)->defaultLowOutputLatency;

    err = Pa_OpenStream(&audio_stream_,
                        nullptr,
                        &out_param,
                        SAMPLE_RATE,
                        256,
                        paNoFlag,
                        audioCallback,
                        this);

    if (err != paNoError) {
        std::cerr << "Unable to open stream, error code " << err << std::endl;
    }

    if (AUDIO_FILE) {
        // Initialize MP3 encoder
        encoder_ = new Mp3encoder(SAMPLE_RATE, 2, 128000, "output.mp3");
    }
}

void AudioWrapper::start_audio_playback() {

    Pa_StartStream(audio_stream_);
    std::cout << "start_audio_playback: " << std::endl;

    is_on_ = true;
}

void AudioWrapper::stop_audio_playback() { is_on_ = false; }

void AudioWrapper::convertFloatToPcm16(const std::vector<float> &floatData, std::vector<short> &pcmData) {
    pcmData.resize(floatData.size());
    for (size_t i = 0; i < floatData.size(); ++i) {
        pcmData[i] = static_cast<short>(floatData[i] * 32767.0f);
    }
}

AudioWrapper::~AudioWrapper() {
    if (is_on_) {
        this->stop_audio_playback();
    }
    Pa_StopStream(audio_stream_);
    Pa_Terminate();

    delete encoder_;
}
