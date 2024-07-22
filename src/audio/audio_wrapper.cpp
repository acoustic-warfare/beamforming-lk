//
// Created by janne on 2024-07-01.
//

#include "audio_wrapper.h"

#include <cmath>
#include <iostream>
#include <vector>

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

    if (AUDIO_FILE && self->audioData.size() >= BUFFER_THRESHOLD) {
        if (MP3) {
            self->flushBufferMp3();
        }

        if (WAV) {
            self->flushBufferWav();
        }

        self->audioData.clear();
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
    flushBufferMp3();

    lame_close(lame_);
    lame_ = nullptr;
}

void AudioWrapper::flushBufferMp3() {
    if (!audioData.empty()) {
        std::cout << "\nFlushing buffer, audioData size: " << audioData.size() << std::endl;

        if (!lame_) {
            lame_ = lame_init();
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
    }
}

void AudioWrapper::saveToWav(const std::string &filename) {
    flushBufferWav();

    sf_close(sndfile);
    sndfile = nullptr;
}

void AudioWrapper::flushBufferWav() {
    if (!audioData.empty()) {
        std::cout << "\nFlushing buffer, audioData size: " << audioData.size() << std::endl;
        if (!sndfile) {
            sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
            sfinfo.channels = 2;
            sfinfo.samplerate = SAMPLE_RATE;

            sndfile = sf_open("output.wav", SFM_WRITE, &sfinfo);
            if (!sndfile) {
                std::cerr << "Unable to open file: output.wav" << std::endl;
                return;
            }
        }

        sf_write_float(sndfile, audioData.data(), audioData.size());
    }
}

void AudioWrapper::stop_audio_playback() {
    is_on_ = false;
}

AudioWrapper::~AudioWrapper() {
    if (is_on_) {
        this->stop_audio_playback();
    }

    Pa_StopStream(audio_stream_);
    Pa_Terminate();

    if (AUDIO_FILE) {
        if (MP3) {
            saveToMp3("output.mp3");
            std::cout << "Mp3 Saved" << std::endl;
        }

        if (WAV) {
            saveToWav("output.wav");
            std::cout << "WAV file saved" << std::endl;
        }
    }
}
