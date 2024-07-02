//
// Created by janne on 2024-07-01.
//

#ifndef BEAMFORMER_AUDIO_WRAPPER_H
#define BEAMFORMER_AUDIO_WRAPPER_H

#include <atomic>

#include "../config.h"
#include "../pipeline.h"
#include <portaudio.h>

class AudioWrapper {
private:
    AudioWrapper(Streams &streams, bool debug);
    Streams _streams;
    std::thread producer_thread_;
    atomic<bool> is_on_ = false;
    bool debug_ = false;
    PaStream *audio_stream_ = nullptr;

public:
    explicit AudioWrapper(Streams &streams);

    AudioWrapper(const AudioWrapper &) = delete;
    AudioWrapper(AudioWrapper &&) = delete;
    AudioWrapper operator=(const AudioWrapper &) = delete;
    AudioWrapper operator=(AudioWrapper &&) = delete;

    void start_audio_playback();
    void stop_audio_playback();

    ~AudioWrapper();
};


#endif//BEAMFORMER_AUDIO_WRAPPER_H
