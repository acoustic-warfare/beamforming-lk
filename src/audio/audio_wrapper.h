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

/**
 * @class AudioWrapper
 * @brief Manages audio streaming and processing, with support for file encoding (and real-time playback)
 */
class AudioWrapper {
private:
    //! Flag if audio is on
    std::atomic<bool> is_on_ = false;

    //! A PaStream providing acces to audio hardware
    PaStream *audio_stream_ = nullptr;

    //! Pipeline to the FPGA from the PC
    Pipeline *pipeline;

    //! Vector of audio data filled from the incomming data
    std::vector<float> audioData;

    //! Struct handling the Wav file
    SNDFILE *sndfile_ = nullptr;

    //! Information about the data in the Wav file
    SF_INFO sfinfo_{};

    //! The MP3 file
    FILE *mp3File_;

    //! Handles informaton about MP3 file, global context handle
    lame_t lame_ = nullptr;

    /**
     * @brief Initializes and opens Wav & MP3 files
     */
    void initAudioFiles();

    /**
    * @brief Encodes and writes the audio data to an MP3 file.
    */
    void flushBufferMP3();

    /**
     * @brief Adds remaining data to MP3 file and closes it
     * @param filename The name the output file will receive
     */
    void saveToMP3(const std::string &filename);

    /**
     * @brief Adds remaining data to Wav file and closes it
     * @param filename The name the output file will receive
     */
    void saveToWav(const std::string &filename);

    /**
    * @brief Writes the audio data to an Wav file.
    */
    void flushBufferWav();

public:
    /**
     * @brief Constructs an AudioWrapper instance.
     * @param pipeline Pointer to a Pipeline object for data streaming.
     */
    AudioWrapper(Pipeline *pipeline);

    // Delete copy constructor and assignment operators.
    AudioWrapper(const AudioWrapper &) = delete;
    AudioWrapper(AudioWrapper &&) = delete;
    AudioWrapper operator=(const AudioWrapper &) = delete;
    AudioWrapper operator=(AudioWrapper &&) = delete;

    /**
    * @brief Fetches the pipeline
    */
    Pipeline *getPipeline();

    /**
    * @brief Fetches the buffer of incoming audio data
    */
    std::vector<float> *getAudioData();

    /**
    * @brief Starts the stream of audio
    */
    void start_audio_playback();

    /**
    * @brief Stops the stream of audio
    */
    void stop_audio_playback();

    /**
    * @brief Flushes audio data and writes to files during callback, depending on config choices
    */
    void processAudioData();

    /**
     * @brief Destructor, cleanups of resources.
     */
    ~AudioWrapper();
};

#endif//BEAMFORMER_AUDIO_WRAPPER_H
