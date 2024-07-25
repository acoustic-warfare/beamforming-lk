//
// Created by janne on 2024-07-01.
//

#ifndef BEAMFORMER_AUDIO_WRAPPER_H
#define BEAMFORMER_AUDIO_WRAPPER_H

#include <lame/lame.h>
#include <portaudio.h>
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
    AudioWrapper(Pipeline &pipeline, bool debug);
    std::thread producer_thread_;
    std::atomic<bool> is_on_ = false;
    bool debug_ = true;
    PaStream *audio_stream_ = nullptr;

    //Pipeline *pipeline;
    //std::vector<float> audioData;

    void initAudioFiles();

    void saveToMp3(const std::string &filename);
    void saveToWav(const std::string &filename);

    SNDFILE *sndfile_ = nullptr;
    SF_INFO sfinfo_{};

    FILE *mp3File_;
    lame_t lame_ = nullptr;

public:
    explicit AudioWrapper(Pipeline &pipeline);

    AudioWrapper(const AudioWrapper &) = delete;
    AudioWrapper(AudioWrapper &&) = delete;
    AudioWrapper operator=(const AudioWrapper &) = delete;
    AudioWrapper operator=(AudioWrapper &&) = delete;

    void start_audio_playback();
    void stop_audio_playback();

    void flushBufferMp3();
    void flushBufferWav();

    Pipeline &pipeline;
    std::vector<float> audioData;

    //Pipeline *getPipeline(); 
    //std::vector<float> getAudioData();

    ~AudioWrapper();
};

#endif//BEAMFORMER_AUDIO_WRAPPER_H
