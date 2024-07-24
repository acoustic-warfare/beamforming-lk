//
// Created by janne on 2024-07-01.
//

#ifndef BEAMFORMER_AUDIO_WRAPPER_H
#define BEAMFORMER_AUDIO_WRAPPER_H

#include <lame/lame.h>
#include <portaudio.h>
#include <samplerate.h>
#include <sndfile.h>

#include <atomic>
#include <fstream>
#include <string>
#include <vector>

#include "../config.h"
#include "../pipeline.h"
#include "pa_ringbuffer.h"

class AudioWrapper {
private:
    AudioWrapper(Streams &streams, bool debug);
    std::thread producer_thread_;
    std::atomic<bool> is_on_ = false;
    bool debug_ = true;
    PaStream *audio_stream_ = nullptr;

    void initAudioFiles();

    void saveToMp3(const std::string &filename);
    void saveToWav(const std::string &filename);

    SNDFILE *sndfile = nullptr;
    SF_INFO sfinfo {};

    FILE *mp3File;
    lame_t lame_ = nullptr;

    std::vector<float> ringBufferData;
    std::thread fileWriterThread;
    std::atomic<bool> keepRunning;
    void fileWriter();

public:
    explicit AudioWrapper(Streams &streams);

    AudioWrapper(const AudioWrapper &) = delete;
    AudioWrapper(AudioWrapper &&) = delete;
    AudioWrapper operator=(const AudioWrapper &) = delete;
    AudioWrapper operator=(AudioWrapper &&) = delete;

    void start_audio_playback();
    void stop_audio_playback();

    void flushBufferMp3();
    void flushBufferWav();

    //void flushBufferMp3(const std::vector<float> &buffer);
    //void flushBufferWav(const std::vector<float> &buffer);

    Streams _streams;
    std::vector<float> audioData;

    PaUtilRingBuffer ringBuffer;

    SRC_STATE *src_state;
    void resampleAudioData();
    std::vector<float> resampledData;

    ~AudioWrapper();
};


#endif//BEAMFORMER_AUDIO_WRAPPER_H
