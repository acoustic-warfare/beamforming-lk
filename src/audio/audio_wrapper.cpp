//
// Created by janne on 2024-07-01.
//

#include "audio_wrapper.h"

#include <cmath>
#include <iostream>
#include <vector>


#define AUDIO_FILE true
#define BUFFER_THRESHOLD SAMPLE_RATE * 10

constexpr int device = 0;

static int audioCallback(const void *_, void *outputBuffer,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo *timeInfo,
                         PaStreamCallbackFlags statusFlags,
                         void *userData) {

    auto self = static_cast<AudioWrapper *>(userData);
    auto out = static_cast<float *>(outputBuffer);

    const float *in = self->_streams.buffers[0];

    for (unsigned long i = 0; i < framesPerBuffer; ++i) {
        self->audioData.push_back(in[i]);
        out[i] = in[i];
    }

    if (AUDIO_FILE) {
        if (self->audioData.size() >= BUFFER_THRESHOLD) {
            self->flushBuffer();
        }
    }

    return paContinue;
}

static void checkErr(PaError err) {
    if (err != paNoError) {
        std::cerr << "\nPortAudio error: " << err << std::endl;
    }
}

AudioWrapper::AudioWrapper(Streams &streams) : AudioWrapper(streams, debug_) {}

AudioWrapper::AudioWrapper(Streams &streams, bool debug) : _streams(streams), debug_(debug) {
    PaError err = paNoError;
    err = Pa_Initialize();
    checkErr(err);

    if (true) {
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
    std::cout << "out_param.device: " << out_param.device << std::endl;
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
}

void AudioWrapper::start_audio_playback() {
    Pa_StartStream(audio_stream_);
    std::cout << "start_audio_playback: " << std::endl;

    is_on_ = true;
}

void AudioWrapper::saveToMp3(const std::string &filename) {
    flushBuffer();
    std::cout << "Buffer flushed in saveToMp3 " << std::endl;

    if (lame_) {
        const int bufferSize = 2 * audioData.size() + 7200;
        unsigned char mp3Buffer[bufferSize];

        FILE *mp3File = fopen(filename.c_str(), "ab");
        if (!mp3File) {
            std::cerr << "Unable to open file: " << filename << std::endl;
            return;
        }

        int bytesWritten = lame_encode_flush(lame_, mp3Buffer, bufferSize);
        if (bytesWritten < 0) {
            std::cerr << "Error encoding MP3:: " << bytesWritten << std::endl;
        }

        fwrite(mp3Buffer, 1, bytesWritten, mp3File);

        fclose(mp3File);

        lame_close(lame_);
        lame_ = nullptr;
    }
}


void AudioWrapper::flushBuffer() {
    if (!audioData.empty()) {
        std::cout << "\nFlushing buffer, audioData size: " << audioData.size() << std::endl;

        if (!lame_) {
            lame_ = lame_init();
            if (!lame_) {
                std::cerr << "Failed to initialize LAME encoder" << std::endl;
                return;
            }

            lame_set_in_samplerate(lame_, SAMPLE_RATE);
            lame_set_num_channels(lame_, 2);
            lame_set_quality(lame_, 2);

            if (lame_init_params(lame_) < 0) {
                std::cerr << "lame_init_params failed" << std::endl;
                lame_close(lame_);
                lame_ = nullptr;
                return;
            }
        }

        const int mp3BufferSize = 2 * audioData.size() + 7200;
        unsigned char mp3Buffer[mp3BufferSize];
        int numSamples = audioData.size() / 2;

        int bytesWritten = lame_encode_buffer_float(lame_, audioData.data(), audioData.data(), numSamples, mp3Buffer, mp3BufferSize);
        if (bytesWritten < 0) {
            std::cerr << "Error encoding MP3: " << bytesWritten << std::endl;
        }

        FILE *mp3File = fopen("output.mp3", "ab");
        if (!mp3File) {
            std::cerr << "Unable to open file: output.mp3" << std::endl;
            return;
        }

        fwrite(mp3Buffer, 1, bytesWritten, mp3File);

        fclose(mp3File);

        audioData.clear();
    }
}


void AudioWrapper::stop_audio_playback() {
    is_on_ = false;
}

AudioWrapper::~AudioWrapper() {
    if (is_on_) {
        this->stop_audio_playback();
    }

    if (AUDIO_FILE) {
        saveToMp3("output.mp3");
        std::cout << "Mp3 Saved" << std::endl;
    }

    Pa_StopStream(audio_stream_);
    Pa_Terminate();
}
