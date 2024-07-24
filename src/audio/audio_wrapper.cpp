//
// Created by janne on 2024-07-01.
//

#include "audio_wrapper.h"

#include <SDL2/SDL.h>

#include <cmath>
#include <iostream>
#include <vector>

#define AUDIO_FILE true
#define MP3 false
#define WAV true

constexpr int device = 0;

constexpr int INPUT_SAMPLE_RATE = 48828;
constexpr int OUTPUT_SAMPLE_RATE = 44100;


void AudioWrapper::resampleAudioData() {
    if (audioData.empty()) {
        return;
    }

    SRC_DATA srcData;
    srcData.data_in = audioData.data();
    srcData.input_frames = audioData.size() / 1;// Assuming stereo input
    srcData.src_ratio = 44100.0 / 48828.0;
    srcData.output_frames = static_cast<long>(srcData.input_frames * srcData.src_ratio);

    resampledData.resize(srcData.output_frames * 2);// Resize output buffer for stereo output

    srcData.data_out = resampledData.data();
    srcData.end_of_input = 0;

    int error = src_process(src_state, &srcData);
    if (error) {
        std::cerr << "Resampling error: " << src_strerror(error) << std::endl;
    }
}

static int audioCallback(const void *_, void *outputBuffer,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo *timeInfo,
                         PaStreamCallbackFlags statusFlags,
                         void *userData) {

    AudioWrapper *wrapper = static_cast<AudioWrapper *>(userData);
    float *out = static_cast<float *>(outputBuffer);

    auto it = wrapper->_streams.buffers.find(0);
    if (it != wrapper->_streams.buffers.end()) {
        float *buffer = it->second;
        for (unsigned long i = 0; i < framesPerBuffer; ++i) {
            *out++ = buffer[i];
        }
        it->second += framesPerBuffer;
    } else {
        std::fill(out, out + framesPerBuffer, 0.0f);
    }

    //auto self = static_cast<AudioWrapper *>(userData);
    //auto out = static_cast<float *>(outputBuffer);


    // ORIGINAL
    //const float *in = self->_streams.buffers[0];
    //
    //for (unsigned long i = 0; i < framesPerBuffer; ++i) {
    //    self->audioData.push_back(in[i]);
    //    out[i] = in[i];
    //}

    // RINGBUFFER
    //const float *in = self->_streams.buffers[0];
    //PaUtil_WriteRingBuffer(&self->ringBuffer, in, framesPerBuffer);
    //
    //for (unsigned long i = 0; i < framesPerBuffer; ++i) {
    //    out[i] = in[i];
    //}

    //RESAMPLE
    //const float *in = self->_streams.buffers[0];
    //
    //for (unsigned long i = 0; i < framesPerBuffer; ++i) {
    //    self->audioData.push_back(in[i]);
    //}
    //
    //// Check if we have enough data to resample
    //if (self->audioData.size() >= framesPerBuffer * 1) {// Considering stereo input
    //    // Resample the audio data from 48828 Hz to 44100 Hz
    //    self->resampleAudioData();
    //
    //    // Output the resampled data
    //    for (unsigned long i = 0; i < framesPerBuffer * 1; ++i) {// Considering stereo output
    //        out[i] = self->resampledData[i];
    //    }
    //    self->flushBufferWav();
    //    self->resampledData.clear();
    //    self->audioData.clear();
    //}
    //
    //
    //if (AUDIO_FILE && self->audioData.size() >= BUFFER_THRESHOLD) {
    //    if (MP3) {
    //        self->flushBufferMp3();
    //    }
    //
    //    if (WAV) {
    //        self->flushBufferWav();
    //    }
    //
    //    self->audioData.clear();
    //}

    return paContinue;
}

static void checkErr(PaError err) {
    if (err != paNoError) {
        std::cerr << "\nPortAudio error: " << err << std::endl;
    }
}

void AudioWrapper::initAudioFiles() {
    if (MP3) {
        lame_ = lame_init();
        lame_set_in_samplerate(lame_, 48828);
        lame_set_num_channels(lame_, 1);
        lame_set_quality(lame_, 2);

        if (lame_init_params(lame_) < 0) {
            std::cerr << "lame_init_params failed" << std::endl;
            lame_close(lame_);
            lame_ = nullptr;
            return;
        }

        mp3File = fopen("output.mp3", "ab");
        if (!mp3File) {
            std::cerr << "Unable to open file: output.mp3" << std::endl;
            return;
        }
    }

    if (WAV) {
        sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_32;
        sfinfo.channels = 1;
        sfinfo.samplerate = 48828;

        sndfile = sf_open("output.wav", SFM_WRITE, &sfinfo);
        if (!sndfile) {
            std::cerr << "Unable to open file: output.wav" << std::endl;
            return;
        }
    }
}

AudioWrapper::AudioWrapper(Streams &streams) : AudioWrapper(streams, debug_) {}

AudioWrapper::AudioWrapper(Streams &streams, bool debug) : _streams(streams), debug_(debug) {
    PaError err = paNoError;
    err = Pa_Initialize();
    checkErr(err);

    int error;
    src_state = src_new(SRC_SINC_FASTEST, 1, &error);
    if (!src_state) {
        std::cerr << "Error initializing src_state: " << src_strerror(error) << std::endl;
    }

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
    //out_param.device = 5;
    std::cout << "out_param.device: " << out_param.device << std::endl;
    out_param.channelCount = 2;
    out_param.hostApiSpecificStreamInfo = nullptr;
    out_param.sampleFormat = paFloat32;
    out_param.suggestedLatency = Pa_GetDeviceInfo(out_param.device)->defaultLowOutputLatency;
    std::cout << "out_param.suggestedLatency: " << out_param.suggestedLatency << std::endl;

    err = Pa_OpenStream(&audio_stream_,
                        nullptr,
                        &out_param,
                        48828,
                        paFramesPerBufferUnspecified,
                        paNoFlag,
                        audioCallback,
                        this);

    if (err != paNoError) {
        std::cerr << "Unable to open stream, error code " << err << std::endl;
    }

    if (AUDIO_FILE) {
        initAudioFiles();
    }

    //ringBufferData.resize(2048 * 2048);
    //PaUtil_InitializeRingBuffer(&ringBuffer, sizeof(float), ringBufferData.size(), ringBufferData.data());
    //keepRunning = true;
    //fileWriterThread = std::thread(&AudioWrapper::fileWriter, this);
}

/*
void AudioWrapper::fileWriter() {
    std::vector<float> tempBuffer(2048);
    while (keepRunning || PaUtil_GetRingBufferReadAvailable(&ringBuffer) > 0) {
        long elementsRead = PaUtil_ReadRingBuffer(&ringBuffer, tempBuffer.data(), tempBuffer.size());
        if (elementsRead > 0) {
            tempBuffer.resize(elementsRead);
            if (WAV) {
                flushBufferWav(tempBuffer);
            }
            if (MP3) {
                flushBufferMp3(tempBuffer);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
*/

void AudioWrapper::saveToMp3(const std::string &filename) {
    flushBufferMp3();

    //keepRunning = false;
    //if (fileWriterThread.joinable()) {
    //    fileWriterThread.join();
    //}
    //
    //// Flush remaining data in the ring buffer
    //std::vector<float> remainingData(ringBufferData.size());
    //long elementsRead = PaUtil_ReadRingBuffer(&ringBuffer, remainingData.data(), remainingData.size());
    //if (elementsRead > 0) {
    //    remainingData.resize(elementsRead);
    //    flushBufferMp3(remainingData);
    //}

    fclose(mp3File);

    lame_close(lame_);
    lame_ = nullptr;
}

/*
void AudioWrapper::flushBufferMp3(const std::vector<float> &buffer) {
    if (!buffer.empty()) {
        std::cout << "\nFlushing buffer Mp3, buffer size: " << buffer.size() << std::endl;

        const int mp3BufferSize = 2 * buffer.size() + 7200;
        unsigned char mp3Buffer[mp3BufferSize];
        int numSamples = buffer.size() / 2;

        int bytesWritten = lame_encode_buffer_float(lame_, buffer.data(), buffer.data(), numSamples, mp3Buffer, mp3BufferSize);
        if (bytesWritten < 0) {
            std::cerr << "Error encoding MP3: " << bytesWritten << std::endl;
        }

        fwrite(mp3Buffer, 1, bytesWritten, mp3File);
    }
}

void AudioWrapper::flushBufferWav(const std::vector<float> &buffer) {
    if (!buffer.empty()) {
        //std::cout << "\nFlushing buffer wav, buffer size: " << buffer.size() << std::endl;

        sf_write_float(sndfile, buffer.data(), buffer.size());
    }
}
*/

void AudioWrapper::flushBufferMp3() {
    if (!audioData.empty()) {
        std::cout << "\nFlushing buffer Mp3, audioData size: " << audioData.size() << std::endl;

        const int mp3BufferSize = 2 * audioData.size() + 7200;
        unsigned char mp3Buffer[mp3BufferSize];
        int numSamples = audioData.size() / 2;

        int bytesWritten = lame_encode_buffer_float(lame_, audioData.data(), audioData.data(), numSamples, mp3Buffer, mp3BufferSize);
        if (bytesWritten < 0) {
            std::cerr << "Error encoding MP3: " << bytesWritten << std::endl;
        }

        fwrite(mp3Buffer, 1, bytesWritten, mp3File);
    }
}


void AudioWrapper::saveToWav(const std::string &filename) {
    flushBufferWav();

    //keepRunning = false;
    //if (fileWriterThread.joinable()) {
    //    fileWriterThread.join();
    //}

    // Flush remaining data in the ring buffer
    //std::vector<float> remainingData(ringBufferData.size());
    //long elementsRead = PaUtil_ReadRingBuffer(&ringBuffer, remainingData.data(), remainingData.size());
    //if (elementsRead > 0) {
    //    remainingData.resize(elementsRead);
    //    flushBufferWav(remainingData);
    //}

    sf_close(sndfile);
    sndfile = nullptr;
}


void AudioWrapper::flushBufferWav() {
    if (!resampledData.empty()) {
        //std::cout << "\nFlushing buffer wav, audioData size: " << audioData.size() << std::endl;

        sf_write_float(sndfile, resampledData.data(), resampledData.size());
    }
}

void AudioWrapper::start_audio_playback() {
    Pa_StartStream(audio_stream_);
    std::cout << "start_audio_playback: " << std::endl;

    is_on_ = true;
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

    //keepRunning = false;
    //if (fileWriterThread.joinable()) {
    //    fileWriterThread.join();
    //}

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

    if (src_state) {
        src_delete(src_state);
    }
}
