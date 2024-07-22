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

std::vector<short> audioBuffer;

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


    auto self = static_cast<AudioWrapper *>(userData);
    auto out = static_cast<float *>(outputBuffer);

    const float *in = self->_streams.buffers[0];

    size_t numFrames = framesPerBuffer;
    audioBuffer.reserve(audioBuffer.size() + 2 * numFrames);

    for (unsigned long i = 0; i < framesPerBuffer; ++i) {
        //self->audioData.push_back(inBuffer[i]);
        audioBuffer.push_back(static_cast<short>(in[2 * i] * 32767.0f));
        audioBuffer.push_back(static_cast<short>(in[2 * i + 1] * 32767.0f));// Right channel       // Left channelfloat --> short
        out[i] = in[i];
    }

    if (AUDIO_FILE) {
        if (audioBuffer.size() >= BUFFER_THRESHOLD) {
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
    //paFramesPerBufferUnspecified

    PaStreamParameters out_param;
    out_param.device = Pa_GetDefaultOutputDevice();
    //out_param.device = 17;
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

//void AudioWrapper::convertFloatToPcm16(const std::vector<float> &floatData, std::vector<short> &pcmData) {
//    pcmData.resize(floatData.size());
//    for (size_t i = 0; i < floatData.size(); ++i) {
//        pcmData[i] = static_cast<short>(std::clamp(floatData[i] * 32767.0f, -32768.0f, 32767.0f));
//    }
//}

//void AudioWrapper::convertFloatToPcm16(const std::vector<float> &floatData, std::vector<short> &pcmData) {
//    pcmData.resize(floatData.size() * 2);
//
//    for (size_t i = 0, j = 0; i < floatData.size(); ++i) {
//        float sample = floatData[i] * 32767.0f;
//        short pcmSample = static_cast<short>(std::clamp(sample, -32768.0f, 32767.0f));
//
//        pcmData[j++] = pcmSample;
//        pcmData[j++] = pcmSample;// Assuming stereo, same sample for both channels
//    }
// }

const int PCM_MAX = 32767;
const int PCM_MIN = -32768;

void convertFloatToPcmInterleaved(const std::vector<float> &floatData,
                                  std::vector<short> &pcmBuffer) {
    // Each float sample represents one channel; 2 channels per sample
    pcmBuffer.resize(floatData.size());

    for (size_t i = 0; i < floatData.size(); ++i) {
        // Convert and clip each float sample to 16-bit PCM
        pcmBuffer[i] = static_cast<short>(std::max(PCM_MIN, std::min(PCM_MAX, static_cast<int>(floatData[i] * PCM_MAX))));
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
    flushBuffer();
    std::cout << "Buffer flushed in saveToMp3 " << std::endl;

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
    if (!audioBuffer.empty()) {
        std::cout << "\nFlushing buffer, audioData size: " << audioBuffer.size() << std::endl;
        // Convert float audio data to PCM
        //std::vector<short> pcmData;
        //convertFloatToPcm16(audioData, pcmData);

        //std::cout << "PCM data size: " << pcmData.size() << std::endl;

        //std::vector<short> pcmData;
        //size_t numSamples = audioData.size() / 2;// Assuming interleaved stereo
        //std::cout << "numSamples: " << numSamples << std::endl;
        //convertFloatToPcmInterleaved(audioData, pcmData);
        //std::cout << "pcm size: " << pcmData.size() << std::endl;


        //if (pcmData.size() != audioData.size() * 2) {
        //    std::cerr << "Mismatch between audioData and pcmData sizes!" << std::endl;
        //    std::cerr << "audioData size: " << audioData.size() << std::endl;
        //    std::cerr << "pcmData size: " << pcmData.size() << std::endl;
        //}

        //std::cout << "First few PCM values: ";
        //for (size_t i = 0; i < std::min<size_t>(10, pcmData.size()); ++i) {
        //    std::cout << pcmData[i] << " ";
        //}
        //std::cout << std::endl;
        int numSamples = audioBuffer.size() / 2;// Number of samples per channel
        int sampleRate = SAMPLE_RATE;
        int numChannels = 2;// Stereo

        if (!lame_) {
            lame_ = lame_init();
            if (!lame_) {
                std::cerr << "Failed to initialize LAME encoder" << std::endl;
                return;
            }

            lame_set_in_samplerate(lame_, 48000);// Set sample rate
            lame_set_num_channels(lame_, 2);     // Set number of channels (stereo)
            lame_set_quality(lame_, 2);          // Set quality (range 0-9, 2 is high quality)

            if (lame_init_params(lame_) < 0) {
                std::cerr << "lame_init_params failed" << std::endl;
                lame_close(lame_);
                lame_ = nullptr;
                return;
            }
        }

        int mp3BufferSize = 1.25 * audioBuffer.size() + 7200;// Estimate size
        std::vector<unsigned char> mp3Buffer(mp3BufferSize);

        std::cout << "mp3BufferSize: " << mp3BufferSize << std::endl;
        std::cout << "audioBuffer: " << audioBuffer.size() << std::endl;


        // Encode PCM to MP3
        int bytesWritten = lame_encode_buffer_interleaved(lame_, audioBuffer.data(), numSamples, mp3Buffer.data(), mp3BufferSize);
        /*
        const int bufferSize = 8192;
        unsigned char mp3Buffer[bufferSize];
        int mp3DataSize;

        FILE *mp3File = fopen("output.mp3", "ab");// Append to the existing file
        if (!mp3File) {
            std::cerr << "Unable to open file: output.mp3" << std::endl;
            return;
        }

        int numSamples = pcmData.size();// / 2;// Assuming stereo, each sample has 2 channels

        try {
            std::cout << "pcmData.data(): " << pcmData.data() << std::endl;
            std::cout << "numSamples: " << numSamples << std::endl;
            std::cout << "mp3Buffer: " << mp3Buffer << std::endl;
            std::cout << "bufferSize: " << bufferSize << std::endl;

            //mp3DataSize = lame_encode_buffer_interleaved(lame_, pcmData.data(), numSamples, mp3Buffer, bufferSize);
            int mp3DataSize = lame_encode_buffer_interleaved(lame_, pcmData.data(), numSamples, mp3Buffer, bufferSize);
            if (mp3DataSize < 0) {
                std::cerr << "Error encoding MP3: " << mp3DataSize << std::endl;
                fclose(mp3File);
                return;
            }
            std::cout << "MP3 data size: " << mp3DataSize << std::endl;
            fwrite(mp3Buffer, 1, mp3DataSize, mp3File);
        } catch (const std::exception &e) {
            std::cerr << "Exception during encoding: " << e.what() << std::endl;
            fclose(mp3File);
            return;
        }
        */
        if (bytesWritten < 0) {
            std::cerr << "Error encoding MP3:: " << bytesWritten << std::endl;
        }

        FILE *mp3File = fopen("output.mp3", "ab");// Append to the existing file
        if (!mp3File) {
            std::cerr << "Unable to open file: output.mp3" << std::endl;
            return;
        }

        fwrite(reinterpret_cast<char *>(mp3Buffer.data()), 1, bytesWritten, mp3File);


        fclose(mp3File);

        //std::ofstream outFile("output.mp3", std::ios::binary);
        //outFile.write(reinterpret_cast<char *>(mp3Buffer.data()), bytesWritten);
        //outFile.close();

        //audioData.clear();
        audioBuffer.clear();
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
    std::cout << "STOP: " << std::endl;
    Pa_Terminate();
    std::cout << "STOP: " << std::endl;
}
