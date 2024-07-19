//
// Created by janne on 2024-07-01.
//

#ifndef BEAMFORMER_AUDIO_WRAPPER_H
#define BEAMFORMER_AUDIO_WRAPPER_H

#include <lame/lame.h>
#include <portaudio.h>

#include <atomic>
#include <fstream>
#include <string>
#include <vector>

#include "../config.h"
#include "../pipeline.h"
#include "Mp3encoder.h"

class AudioWrapper {
private:
    AudioWrapper(Streams &streams, bool debug);
    std::thread producer_thread_;
    std::atomic<bool> is_on_ = false;
    bool debug_ = false;
    PaStream *audio_stream_ = nullptr;

    void convertFloatToPcm16(const std::vector<float> &floatData, std::vector<short> &pcmData);
    void saveToMp3(const std::string &filename);


public:
    explicit AudioWrapper(Streams &streams);

    AudioWrapper(const AudioWrapper &) = delete;
    AudioWrapper(AudioWrapper &&) = delete;
    AudioWrapper operator=(const AudioWrapper &) = delete;
    AudioWrapper operator=(AudioWrapper &&) = delete;

    void start_audio_playback();
    void stop_audio_playback();

    void flushBuffer();

    Streams _streams;
    std::vector<float> audioData;

    Mp3encoder *encoder_;
    Mp3encoder *mp3Encoder_;
    lame_t lame_;

    ~AudioWrapper();
};


#endif//BEAMFORMER_AUDIO_WRAPPER_H
