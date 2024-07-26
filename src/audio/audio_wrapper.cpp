//
// Created by janne on 2024-07-01.
//

#include "audio_wrapper.h"

#include <cmath>
#include <iostream>
#include <vector>

void AudioWrapper::initAudioFiles() {
    if (MP3) {
        lame_ = lame_init();
        lame_set_in_samplerate(lame_, SAMPLE_RATE);
        lame_set_num_channels(lame_, 1);
        lame_set_quality(lame_, 2);

        if (lame_init_params(lame_) < 0) {
            std::cerr << "lame_init_params failed" << std::endl;
            lame_close(lame_);
            lame_ = nullptr;
        }

        mp3File_ = fopen("output.mp3", "wb");
        if (!mp3File_) {
            std::cerr << "Unable to open file: output.mp3" << std::endl;
        }
    }

    if (WAV) {
        sfinfo_.format = SF_FORMAT_WAV | SF_FORMAT_PCM_32;
        sfinfo_.channels = 1;
        sfinfo_.samplerate = SAMPLE_RATE;

        sndfile_ = sf_open("output.wav", SFM_WRITE, &sfinfo_);
        if (!sndfile_) {
            std::cerr << "Unable to open file: output.wav" << std::endl;
        }
    }
}

void AudioWrapper::flushBufferMP3() {
    std::vector<float> *audioData = getAudioData();

    if (!audioData->empty()) {
        const int mp3BufferSize = audioData->size();
        unsigned char mp3Buffer[mp3BufferSize];
        int numSamples = audioData->size();

        // Encodes for MP3
        int bytesWritten = lame_encode_buffer_ieee_float(lame_, audioData->data(), nullptr, numSamples, mp3Buffer, mp3BufferSize);
        if (bytesWritten < 0) {
            std::cerr << "Error encoding MP3: " << bytesWritten << std::endl;
        } else {
            // Writes to MP3 file
            fwrite(mp3Buffer, 1, bytesWritten, mp3File_);
        }
    }
}

void AudioWrapper::saveToMP3(const std::string &filename) {
    flushBufferMP3();

    fclose(mp3File_);

    lame_close(lame_);
    lame_ = nullptr;
}

void AudioWrapper::flushBufferWav() {
    std::vector<float> *audioData = getAudioData();

    if (!audioData->empty()) {
        // Writes to Wav file
        sf_write_float(sndfile_, audioData->data(), audioData->size());
    }
}

void AudioWrapper::saveToWav(const std::string &filename) {
    flushBufferWav();

    sf_close(sndfile_);
    sndfile_ = nullptr;
}

/**
 * The callback function portaudio continuously runs to process audio. 
 * Audio data from the pipeline is placed into audioData buffer to be acceses by file encoders,
 * as well as an out buffer which plays the audio real time. 
 * Only when incoming data exists it is processed (barrier).
 */
static int audioCallback(const void *_, void *outputBuffer,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo *timeInfo,
                         PaStreamCallbackFlags statusFlags,
                         void *userData) {

    auto self = static_cast<AudioWrapper *>(userData);
    auto out = static_cast<float *>(outputBuffer);

    Pipeline *pipeline = self->getPipeline();
    std::vector<float> *audioData = self->getAudioData();

    pipeline->barrier();

    float *in = pipeline->getStreams()->get_signal(0, 0);

    for (unsigned long i = 0; i < framesPerBuffer; ++i) {
        audioData->push_back(in[i]);
        out[i] = in[i];
    }

    self->processAudioData();

    return paContinue;
}

static void checkErr(PaError err) {
    if (err != paNoError) {
        std::cerr << "\nPortAudio error: " << err << std::endl;
    }
}

/**
 * For debugging and printing current audio devices
 */
static void printDevices() {
    const int deviceAmt = Pa_GetDeviceCount();

    if (deviceAmt <= 0) {
        std::cerr << "\nError getting device count: " << deviceAmt << std::endl;
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

AudioWrapper::AudioWrapper(Pipeline *pipeline) : pipeline(pipeline) {
    // Initializes port audio
    PaError err = paNoError;
    err = Pa_Initialize();
    checkErr(err);

    if (DEBUG) {
        printDevices();
    }

    // Opens port audio stream
    PaStreamParameters out_param;
    out_param.device = Pa_GetDefaultOutputDevice();
    out_param.channelCount = 1;
    out_param.hostApiSpecificStreamInfo = nullptr;
    out_param.sampleFormat = paFloat32;
    out_param.suggestedLatency = Pa_GetDeviceInfo(out_param.device)->defaultLowOutputLatency;

    err = Pa_OpenStream(&audio_stream_,
                        nullptr,
                        &out_param,
                        SAMPLE_RATE,
                        N_SAMPLES * 2,
                        paNoFlag,
                        audioCallback,
                        this);

    checkErr(err);

    // Initializes audio files
    if (AUDIO_FILE) {
        initAudioFiles();
    }
}

Pipeline *AudioWrapper::getPipeline() {
    return pipeline;
}

std::vector<float> *AudioWrapper::getAudioData() {
    return &audioData;
}

void AudioWrapper::start_audio_playback() {
    Pa_StartStream(audio_stream_);
    is_on_ = true;
}

void AudioWrapper::stop_audio_playback() {
    is_on_ = false;
}

void AudioWrapper::processAudioData() {
    if (AUDIO_FILE && audioData.size() >= BUFFER_THRESHOLD) {
        if (MP3) {
            flushBufferMP3();
        }

        if (WAV) {
            flushBufferWav();
        }

        audioData.clear();
    }
}

AudioWrapper::~AudioWrapper() {
    if (is_on_) {
        this->stop_audio_playback();
    }

    Pa_StopStream(audio_stream_);
    Pa_Terminate();

    if (AUDIO_FILE) {
        if (MP3) {
            saveToMP3("output.mp3");
            std::cout << "Mp3 Saved" << std::endl;
        }

        if (WAV) {
            saveToWav("output.wav");
            std::cout << "WAV file saved" << std::endl;
        }
    }
}
