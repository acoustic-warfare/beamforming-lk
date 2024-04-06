#include "receiver.h"
#include <cstddef>
#include <cstdio>

#define RECEIVER_DEBUG 0

#if RECEIVER_DEBUG

void printBinary(uint32_t n) {
  for (int i = 31; i >= 0; --i) {
    // Bitwise AND operation to check if the ith bit is set
    if ((n >> i) & 1)
      std::cout << "1";
    else
      std::cout << "0";

    // Print space after every 4 bits for readability
    if (i % 8 == 0)
      std::cout << " ";
  }
  std::cout << " " << n << std::endl;
}
#endif

message *msg;
int socket_desc;

int init_receiver() {
  if (msg) {

#if RECEIVER_DEBUG
    std::cout << "Message pointer already defined" << std::endl;
#endif

    return -1;
  }
  msg = new message();

  struct sockaddr_in server_addr;
  socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (socket_desc < 0) {
#if RECEIVER_DEBUG
    printf("Error creating socket\n");
#endif
    return -1;
  } else {
#if RECEIVER_DEBUG
    printf("Socket created successfully\n");
#endif
  }

  // Set port and IP:
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(UDP_PORT);
  server_addr.sin_addr.s_addr = inet_addr(UDP_ADDRESS);

  // Bind to the set port and IP:
  if (bind(socket_desc, (struct sockaddr *)&server_addr, sizeof(server_addr)) <
      0) {
#if RECEIVER_DEBUG
    printf("Couldn't bind socket to the port\n");
#endif
    return -1;
  }
#if RECEIVER_DEBUG
  printf("Binding complete\n");
#endif

  return 0;
}

void stop_receiving() {
  if (msg) {
    close(socket_desc);
    delete msg;
  }
}

float sine[2 * 48828];

uint32_t packet[HEADER_SIZE + N_SENSORS];

int i = 0;
void pack_offset(ring_buffer *rb, uint32_t *packet, std::size_t &offset) {

  int inverted = 0;

  // Array is daisy-chained, flip even columns
  for (unsigned i = 0; i < N_SENSORS; i++) {
    if (i % COLUMNS == 0) {
      inverted = !inverted;
    }

    // rb->data[i][offset] =
    //     ((float)(int32_t)packet[HEADER_SIZE + i]) / MAX_VALUE_FLOAT;

    if (inverted) {
      rb->data[i][offset] =
          ((float)(int32_t)packet[HEADER_SIZE + i]) / MAX_VALUE_FLOAT;

    } else {
      rb->data[i][offset] =
          ((float)(int32_t)packet[HEADER_SIZE + COLUMNS * (1 + i / COLUMNS) -
                                  i % COLUMNS]) /
          MAX_VALUE_FLOAT;
    }
  }

  offset = (offset + 1) & (BUFFER_LENGTH - 1);
}
int receive(ring_buffer *rb) {
  if (recv(socket_desc, &packet[0],
           sizeof(uint32_t) * (HEADER_SIZE + N_SENSORS), 0) < 0) {
    printf("Couldn't receive\n");
    return -1;
  }

#if RECEIVER_DEBUG

  uint32_t header, counter;

  header = packet[0];
  counter = packet[1];

  int32_t version, n_arrays, frequency;

  // Order as defined in FPGA
  frequency = header & 0x0000FFFF;
  n_arrays = (header & 0x00FF0000) >> 16;
  version = (header & 0xFF000000) >> 24;

  printf("Version: %d\nArrays: %d\nFrequency: %d\nCounter: %d\n", version,
         n_arrays, frequency, counter);
#endif

  // write_buffer(rb, &packet[HEADER_SIZE]);
  //
  pack_offset(rb, &packet[0], rb->index);

  return 0;
}

int receive_offset(ring_buffer *rb) {
  std::size_t index = rb->index;

  for (size_t i = 0; i < N_SAMPLES; i++) {
    if (recv(socket_desc, &packet[0],
             sizeof(uint32_t) * (HEADER_SIZE + N_SENSORS), 0) < 0) {
      printf("Couldn't receive\n");
      return -1;
    }

    pack_offset(rb, &packet[0], index);
  }

  return 0;
}

#if 0

#include "RtAudio.h"
#include "config.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
RtAudio audio;
int play = 1;
std::thread *producer;
std::vector<float> audioBuffer(N_SAMPLES * 2, 0.0);
int running = 1;

float buffers[N_FRAMES][N_SAMPLES];

int k = 0;
void audio_producer() {

  float val = 0.f;

  int ki = k + 1;

  while (running) {

    for (size_t i = 0; i < N_SAMPLES; i++) {
      if (recv(socket_desc, &packet[0],
               sizeof(uint32_t) * (HEADER_SIZE + N_SENSORS), 0) < 0) {
        printf("Couldn't receive\n");
        // return -1;
        break;
      }

      val = ((float)(int32_t)packet[HEADER_SIZE + 130]) / MAX_VALUE_FLOAT;

      val *= 100.0;

      audioBuffer[i * 2] = val;
      audioBuffer[i * 2 + 1] = val;
    }

    k++;
    k %= N_FRAMES;

    ki = (k + 1) % N_FRAMES;

    play = 0;
  }
}

int audioCallback(void *outputBuffer, void *inputBuffer,
                  unsigned int nBufferFrames, double streamTime,
                  RtAudioStreamStatus status, void *userData) {
  float *buffer = (float *)outputBuffer;

  // Copy samples from the sineBuffer to the output buffer for playback
  for (unsigned int i = 0; i < N_SAMPLES * 2; ++i) {
    // if (!play) {
    //   *buffer++ = audioBuffer[i];
    // } else {
    //   std::cout << "UNderflow" << std::endl;
    //   *buffer++ = 0.0;
    // }
    *buffer++ = audioBuffer[i];
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
    unsigned int bufferFrames = N_SAMPLES;
    audio.openStream(&parameters, nullptr, RTAUDIO_FLOAT32, SAMPLE_RATE,
                     &bufferFrames, &audioCallback);
    audio.startStream();

    producer = new std::thread(audio_producer);

  } catch (RtAudioErrorType &e) {
    // std::cout << "Error: " << e.getMessage() << std::endl;
    return 1;
  }

  return 0;
}

void stop_audio_playback() {
  // Start the separate thread for sine wave generation

  // Keep the program running
  running = 0;

  // Stop the sine wave generation thread
  producer->join();

  // Stop and close the RtAudio stream
  audio.stopStream();
  audio.closeStream();
}

int main(int argc, char *argv[]) {

  if (init_receiver()) {
    std::cout << "Error initializing receiver" << std::endl;

    return 1;
  }

  init_audio_playback();

  std::cout << "Press Enter to quit..." << std::endl;
  getchar();

  std::cout << "stop" << std::endl;

  stop_audio_playback();

  // ring_buffer *rb = new ring_buffer();
  //
  // for (int i = 0; i < BUFFER_LENGTH; i++) {
  //   receive(rb);
  // }
  //
  // FILE *file = fopen("float2_array.bin", "wb");
  // if (file == NULL) {
  //   printf("Error opening file.\n");
  //   return 1;
  // }
  //
  // // Write the float array to the file
  // fwrite(&rb->data[150], sizeof(float), BUFFER_LENGTH, file);
  // // fwrite(sine, sizeof(float), sizeof(sine) / sizeof(float), file);
  // // Close the file
  // fclose(file);
  //
  // delete rb;
  //
  // stop_receiving();
  return 0;
}

#endif
