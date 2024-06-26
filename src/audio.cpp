
#include "RtAudio.h"
#include "config.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

// Parameters for the sine wave generation
const double PI = 3.14159265358979323846;
const double frequency =
    3 * 440.0;                     // Frequency of the sine wave (440 Hz for A4)
const double sampleRate = 48828.0; // Sample rate (44.1 kHz)
unsigned int bufferFrames = 256;   // 256;   // Buffer size
RtAudio audio;

std::thread *producer;
// Global variables
std::vector<float> sineBuffer(bufferFrames * 2,
                              0.0); // Buffer for sine wave samples
bool isGenerating =
    true; // Control variable for the sine wave generation thread

int play = 1;
// Function to generate the sine wave in a separate thread
void generateSineWave() {
  double phase = 0.0;
  const double phaseIncrement = frequency * 2.0 * PI / sampleRate;

  while (isGenerating) {

    while (!play) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    play = 0;

    for (unsigned int i = 0; i < bufferFrames; ++i) {
      double sample = sin(phase);
      phase += phaseIncrement;

      // Ensure the phase remains within range [0, 2*PI)
      if (phase >= 2.0 * PI)
        phase -= 2.0 * PI;

      // Fill both channels with the sine wave sample
      sineBuffer[i * 2] = sample;     // Left channel
      sineBuffer[i * 2 + 1] = sample; // Right channel
    }
  }
}

void producerThread() {
  while (isGenerating) {
    while (!play) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    play = 0;

    for (unsigned int i = 0; i < N_SAMPLES; i++) {
      audioBuffer[i * 2] =
    }
  }
}

// Callback function for RtAudio
int audioCallback(void *outputBuffer, void *inputBuffer,
                  unsigned int nBufferFrames, double streamTime,
                  RtAudioStreamStatus status, void *userData) {
  float *buffer = (float *)outputBuffer;

  // Copy samples from the sineBuffer to the output buffer for playback
  for (unsigned int i = 0; i < nBufferFrames * 2; ++i) {
    *buffer++ = sineBuffer[i];
  }

  play = 1;

  return 0;
}

int init_audio_playback() {
  if (audio.getDeviceCount() < 1) {
    std::cout << "No audio devices found!" << std::endl;
    return EXIT_FAILURE;
  }

  RtAudio::StreamParameters parameters;
  parameters.deviceId = audio.getDefaultOutputDevice();
  parameters.nChannels = 2; // Stereo output

  try {
    audio.openStream(&parameters, nullptr, RTAUDIO_FLOAT32, sampleRate,
                     &bufferFrames, &audioCallback);
    audio.startStream();

    producer = new std::thread(generateSineWave);

  } catch (RtAudioErrorType &e) {
    // std::cout << "Error: " << e.getMessage() << std::endl;
    return 1;
  }

  return 0;
}

void stop_audio_playback() {
  // Start the separate thread for sine wave generation

  // Keep the program running

  // Stop the sine wave generation thread
  isGenerating = false;
  play = 1;
  producer->join();

  // Stop and close the RtAudio stream
  audio.stopStream();
  audio.closeStream();
}

int main() {

  init_audio_playback();

  std::cout << "Press Enter to quit..." << std::endl;
  getchar();

  std::cout << "stop" << std::endl;

  stop_audio_playback();

  return 0;

  // if (audio.getDeviceCount() < 1) {
  //   std::cout << "No audio devices found!" << std::endl;
  //   return 1;
  // }
  //
  // RtAudio::StreamParameters parameters;
  // parameters.deviceId = audio.getDefaultOutputDevice();
  // parameters.nChannels = 2; // Stereo output
  //
  // try {
  //   audio.openStream(&parameters, nullptr, RTAUDIO_FLOAT32, sampleRate,
  //                    &bufferFrames, &audioCallback);
  //   audio.startStream();
  //
  //   // Start the separate thread for sine wave generation
  //   std::thread sineThread(generateSineWave);
  //
  //   // Keep the program running
  //   std::cout << "Press Enter to quit..." << std::endl;
  //   getchar();
  //
  //   std::cout << "stop" << std::endl;
  //
  //   // Stop the sine wave generation thread
  //   isGenerating = false;
  //   play = 1;
  //   sineThread.join();
  //
  //   // Stop and close the RtAudio stream
  //   audio.stopStream();
  //   audio.closeStream();
  // } catch (RtAudioErrorType &e) {
  //   // std::cout << "Error: " << e.getMessage() << std::endl;
  //   return 1;
  // }
  //
  // return 0;
}



// If Audio playback when streaming
#if AUDIO

RtAudio audio;
int play = 1;
std::thread *producer;
std::vector<float> audioBuffer(N_SAMPLES * 2, 0.0);

/**
 * @brief Producer for audio on pipeline
 *
 * @param pipeline Pipeline
 */
void audio_producer(Pipeline &pipeline) {

  ring_buffer &rb = pipeline.getRingBuffer();

  float out[N_SAMPLES] = {0.0};

  while (pipeline.isRunning()) {

    for (int i = 0; i < N_SAMPLES; i++) {
      out[i] /= 64.f;

      out[i] *= 100.f;
      audioBuffer[i * 2] = out[i];
      audioBuffer[i * 2 + 1] = out[i];
      out[i] = 0.0;
    }
    play = 0;

    pipeline.barrier();

    // for (int s = 0; s < N_SENSORS; s++) {
    //
    //   if (VALID_SENSOR(s)) {
    //     // cout << s << " ";
    //     naive_delay(&rb, &out[0], 0.0, s);
    //   }
    // }

    naive_delay(&rb, &out[0], 0.0, 140);

    // for (int i = 0; i < N_SAMPLES; i++) {
    //   audioBuffer[i * 2] = rb.data[140][rb.index + i];
    //   audioBuffer[i * 2 + 1] = rb.data[140][rb.index + i];
    // }

    // cout << "run" << endl;

    // memcpy(&yrb.data[140][rb.index], &audioBuffer[0],
    //        N_SAMPLES * sizeof(float));
  }
}

/**
 * @brief Callback for audio stream
 *
 * @param outputBuffer Speaker buffer
 * @param inputBuffer empty (Required by RtAudio API)
 * @param nBufferFrames number of frames to fill
 * @param streamTime duration
 * @param status status
 * @param userData the incoming data
 * @return OK
 */
int audioCallback(void *outputBuffer, void *inputBuffer,
                  unsigned int nBufferFrames, double streamTime,
                  RtAudioStreamStatus status, void *userData) {
  float *buffer = (float *)outputBuffer;

  // Copy samples from the sineBuffer to the output buffer for playback
  for (unsigned int i = 0; i < N_SAMPLES * 2; ++i) {
    if (!play) {
      *buffer++ = audioBuffer[i];
    } else {
      cout << "Underflow" << endl;
      *buffer++ = 0.0;
    }
  }

  play = 1;

  return 0;
}

/**
 * @brief Initiate Audio player for Pipeline
 *
 * @param pipeline the pipeline to follow
 * @return status
 */
int init_audio_playback(Pipeline &pipeline) {
  if (audio.getDeviceCount() < 1) {
    std::cout << "No audio devices found!" << std::endl;
    return EXIT_FAILURE;
  }

  RtAudio::StreamParameters parameters;
  parameters.deviceId = audio.getDefaultOutputDevice();
  parameters.nChannels = 2; // Stereo output

  try {
    unsigned int bufferFrames = N_SAMPLES;
    audio.openStream(&parameters, nullptr, RTAUDIO_FLOAT32, 44100.f,
                     &bufferFrames, &audioCallback);
    audio.startStream();

    producer = new std::thread(audio_producer, ref(pipeline));

  } catch (RtAudioErrorType &e) {
    // std::cout << "Error: " << e.getMessage() << std::endl;
    return 1;
  }

  return 0;
}

void stop_audio_playback() {
  // Start the separate thread for sine wave generation

  // Keep the program running

  // Stop the sine wave generation thread
  producer->join();

  // Stop and close the RtAudio stream
  audio.stopStream();
  audio.closeStream();
}

#endif
