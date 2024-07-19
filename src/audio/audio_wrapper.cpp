//
// Created by janne on 2024-07-01.
//

#include "audio_wrapper.h"

#include <cmath>
#include <iostream>
#include <vector>
#include "../../../../../../usr/include/c++/11/bits/algorithmfwd.h"

#define AUDIO_FILE true
#define BUFFER_THRESHOLD 48000 * 10

constexpr int device = 0;

static int audioCallback(const void *_, void *outputBuffer,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo *timeInfo,
                         PaStreamCallbackFlags statusFlags,
                         void *userData) {
    //auto self = static_cast<AudioWrapper *>(userData);
    //auto in = (Streams *) userData;
    //auto out = (float *) outputBuffer;
    //
    //(void) _;// Ignore unused variable warning for input buffer
    //
    //
    //for (int i = 0; i < framesPerBuffer; ++i) {
    //    //std::cout << "i: " << i << "buf: " << in->buffers[4][i] << std::endl;
    //    out[i] = in->buffers[4][i];
    //    //out[i] = self->_streams.buffers[4][i];
    //}


    //auto self = static_cast<AudioWrapper *>(userData);
    //auto out = static_cast<float *>(outputBuffer);
    //
    //const float *in = self->_streams.buffers[4];
    //for (unsigned long i = 0; i < framesPerBuffer; ++i) {
    //    self->audioData.push_back(in[i]);
    //    out[i] = in[i];
    //}

    //if (AUDIO_FILE) {
    //    // Encode data to MP3
    //    std::vector<short> pcmData;
    //    self->convertFloatToPcm16(self->audioData, pcmData);
    //    self->encoder_->encode_inter(pcmData.data(), pcmData.size());
    //}


    // Cast userData to AudioWrapper*
    auto self = static_cast<AudioWrapper *>(userData);
    auto out = static_cast<float *>(outputBuffer);

    // Assuming buffers[0] contains the audio data we want to process
    const float *inBuffer = self->_streams.buffers[0];// Access the appropriate buffer

    // Process the audio data
    for (unsigned long i = 0; i < framesPerBuffer; ++i) {
        self->audioData.push_back(inBuffer[i]);
        out[i] = inBuffer[i]; // Copy audio data
    }

    if (self->audioData.size() >= BUFFER_THRESHOLD) {
        self->flushBuffer();
    }

    return paContinue;
}

/*
static int audioCallback(const void *inputBuffer, void *outputBuffer,
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
        // Encode data to MP3
        std::vector<short> pcmData;
        self->convertFloatToPcm16(self->audioData, pcmData);
        self->encoder_->encode_inter(pcmData.data(), pcmData.size());
    }

    return paContinue;
}
*/

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
    //paFramesPerBufferUnspecified

    PaStreamParameters out_param;
    out_param.device = Pa_GetDefaultOutputDevice();
    std::cout << "out_param.device: " << out_param.device << std::endl;
    out_param.channelCount = 2;
    out_param.hostApiSpecificStreamInfo = nullptr;
    out_param.sampleFormat = paFloat32;
    out_param.suggestedLatency = Pa_GetDeviceInfo(out_param.device)->defaultLowOutputLatency;

    //if (AUDIO_FILE) {
    //    // Initialize MP3 encoder
    //    encoder_ = new Mp3encoder(Pa_GetDeviceInfo(out_param.device)->defaultSampleRate, 2, 128000, "output.mp3");
    //}

    err = Pa_OpenStream(&audio_stream_,
                        nullptr,
                        &out_param,
                        48000,
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

void AudioWrapper::convertFloatToPcm16(const std::vector<float> &floatData, std::vector<short> &pcmData) {
    pcmData.resize(floatData.size());
    for (size_t i = 0; i < floatData.size(); ++i) {
        pcmData[i] = static_cast<short>(std::clamp(floatData[i] * 32767.0f, -32768.0f, 32767.0f));
    }
}

/*
// Save audio data to MP3 file
void AudioWrapper::saveToMp3(const std::string &filename) {
    // Convert float audio data to PCM
    std::vector<short> pcmData;
    convertFloatToPcm16(audioData, pcmData);


    // Setup LAME encoder
    lame_t lame = lame_init();
    lame_set_in_samplerate(lame, SAMPLE_RATE);
    lame_set_num_channels(lame, 2);// Assuming stereo
    lame_set_quality(lame, 2);     // 2 = high quality
    lame_init_params(lame);

    // Encode PCM data to MP3
    FILE *mp3File = fopen(filename.c_str(), "wb");
    if (!mp3File) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return;
    }

    const int bufferSize = 8192;
    unsigned char mp3Buffer[bufferSize];
    int mp3DataSize;


    std::cout << "mp3DataSize: " << mp3DataSize << std::endl;
    std::cout << "pcmData.size(): " << pcmData.size() << std::endl;

    mp3DataSize = lame_encode_buffer_interleaved(lame, pcmData.data(), pcmData.size() / 2, mp3Buffer, bufferSize);
    fwrite(mp3Buffer, 1, mp3DataSize, mp3File);

    std::cout << "mp3DataSize: " << mp3DataSize << std::endl;
    std::cout << "mp3Buffer: " << mp3Buffer << std::endl;
    std::cout << "mp3File: " << mp3File << std::endl;


    mp3DataSize = lame_encode_flush(lame, mp3Buffer, bufferSize);
    fwrite(mp3Buffer, 1, mp3DataSize, mp3File);

    fclose(mp3File);
    lame_close(lame);
}
*/

void AudioWrapper::saveToMp3(const std::string &filename) {
    // Ensure any remaining data is flushed
    flushBuffer();

    // Finalize LAME encoding
    if (lame_) {
        const int bufferSize = 8192;
        unsigned char mp3Buffer[bufferSize];
        int mp3DataSize;

        FILE *mp3File = fopen(filename.c_str(), "ab");
        if (!mp3File) {
            std::cerr << "Unable to open file: " << filename << std::endl;
            return;
        }

        mp3DataSize = lame_encode_flush(lame_, mp3Buffer, bufferSize);
        fwrite(mp3Buffer, 1, mp3DataSize, mp3File);

        fclose(mp3File);
        lame_close(lame_);
        lame_ = nullptr;
    }
}


void AudioWrapper::flushBuffer() {
    if (!audioData.empty()) {
        std::cout << "Flushing buffer, audioData size: " << audioData.size() << std::endl;
        // Convert float audio data to PCM
        std::vector<short> pcmData;
        convertFloatToPcm16(audioData, pcmData);

        std::cout << "PCM data size: " << pcmData.size() << std::endl;

        // Setup LAME encoder if not already set
        if (!lame_) {
            lame_ = lame_init();
            lame_set_in_samplerate(lame_, SAMPLE_RATE);
            lame_set_num_channels(lame_, 2);// Assuming stereo
            lame_set_quality(lame_, 2);     // 2 = high quality
            lame_init_params(lame_);
        }

        // Encode PCM data to MP3
        const int bufferSize = 8192;
        unsigned char mp3Buffer[bufferSize];
        int mp3DataSize;

        FILE *mp3File = fopen("output.mp3", "ab");// Append to the existing file
        if (!mp3File) {
            std::cerr << "Unable to open file: output.mp3" << std::endl;
            return;
        }

        mp3DataSize = lame_encode_buffer_interleaved(lame_, pcmData.data(), pcmData.size() / 2, mp3Buffer, bufferSize);
        fwrite(mp3Buffer, 1, mp3DataSize, mp3File);

        fclose(mp3File);

        // Clear the audio data buffer
        audioData.clear();
    }
}


void AudioWrapper::stop_audio_playback() { is_on_ = false; }

AudioWrapper::~AudioWrapper() {
    if (is_on_) {
        this->stop_audio_playback();
    }
    Pa_StopStream(audio_stream_);

    if (AUDIO_FILE) {
        saveToMp3("output.mp3");
    }

    Pa_Terminate();

    delete encoder_;
}
