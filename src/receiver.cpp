#include "receiver.h"

#include <arpa/inet.h>

#include <algorithm>

#define RECEIVER_DEBUG 0
#define HEADER_SIZE 2

#define NUM_IPS 4
const char *ip_addresses[NUM_IPS] = {"192.168.1.75", "192.168.1.76",
                                     "192.168.1.77", "192.168.1.78"};

#if RECEIVER_DEBUG

void printBinary(uint32_t n) {
  for (int i = 31; i >= 0; --i) {
    // Bitwise AND operation to check if the ith bit is set
    if ((n >> i) & 1)
      std::cout << "1";
    else
      std::cout << "0";

    // Print space after every 4 bits for readability
    if (i % 8 == 0) std::cout << " ";
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
  struct sockaddr_in server_addr;
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

int receive_exposure(
    std::vector<Streams *> streams_dist,
    std::vector<std::unique_ptr<BeamformingOptions>> &options) {
  // float data[N_FPGAS][N_SENSORS][N_SAMPLES];

  float ***data = new float **[N_FPGAS];
  for (int i = 0; i < N_FPGAS; ++i) {
    data[i] = new float *[options[i]->n_sensors_];
    for (int j = 0; j < options[i]->n_sensors_; ++j) {
      data[i][j] = new float[N_SAMPLES];
    }
  }

  struct sockaddr_in source_addr;
  __socklen_t source_addr_len = sizeof(source_addr);

  std::vector<bool> done_samples(N_FPGAS, false);

  int sample_counter[N_FPGAS] = {};
  int streams_id = N_FPGAS + 1;

  while (std::any_of(done_samples.begin(), done_samples.end(),
                     [](bool b) { return !b; })) {
    if (recvfrom(socket_desc, msg, sizeof(message), 0,
                 (struct sockaddr *)&source_addr, &source_addr_len) < 0) {
      printf("Couldn't receive\n");
      return -1;
    }

    char ip[INET_ADDRSTRLEN];
    // Convert IP address from binary to text form
    inet_ntop(AF_INET, &(source_addr.sin_addr), ip, INET_ADDRSTRLEN);

    for (int i = 0; i < NUM_IPS; ++i) {
      if (strcmp(ip, ip_addresses[i]) == 0) {
        streams_id = i;
        break;
      }
    }

    if (streams_id == N_FPGAS + 1) {
      std::cerr << "Unknown IP: " << ip << std::endl;
    }

    int inverted = 1;

    unsigned index;

    for (unsigned m = 0; m < options[streams_id]->n_sensors_; m++) {
      // Array is daisy-chained, flip even columns
      if (m % COLUMNS == 0) {
        inverted = !inverted;
      }

      if (inverted) {
        index = COLUMNS * (1 + m / COLUMNS) - m % COLUMNS;
      } else {
        index = m;
      }

      data[streams_id][m][sample_counter[streams_id]] =
          (float)msg->stream[index] / (float)MAX_VALUE_FLOAT;
    }

    if (sample_counter[streams_id] != N_SAMPLES) {
      sample_counter[streams_id]++;
    } else {
      done_samples[streams_id] = true;
    }
  }

  for (int i = 0; i < N_FPGAS; i++) {
    for (int s = 0; s < options[i]->n_sensors_; s++) {
      streams_dist[i]->write_stream(s, &data[i][s][0]);
    }
  }

  /*
    for (int i = 0; i < N_SAMPLES; i++) {
      if (recvfrom(socket_desc, msg, sizeof(message), 0,
                   (struct sockaddr *)&source_addr, &source_addr_len) < 0) {
        printf("Couldn't receive\n");
        return -1;
      }

      char ip[INET_ADDRSTRLEN];
      // Convert IP address from binary to text form
      inet_ntop(AF_INET, &(source_addr.sin_addr), ip, INET_ADDRSTRLEN);

      for (int i = 0; i < NUM_IPS; ++i) {
        if (strcmp(ip, ip_addresses[i]) == 0) {
          break;
        }
      }

      int inverted = 1;

      unsigned index;

      for (unsigned m = 0; m < N_SENSORS; m++) {
        // Array is daisy-chained, flip even columns
        if (m % COLUMNS == 0) {
          inverted = !inverted;
        }

        if (inverted) {
          index = COLUMNS * (1 + m / COLUMNS) - m % COLUMNS;
        } else {
          index = m;
        }

        data[m][i] = (float)msg->stream[index] / (float)MAX_VALUE_FLOAT;
      }
    }
  */

  return 0;
}

int number_of_sensors(int id, BeamformingOptions *config) {
  struct sockaddr_in server_addr;
  __socklen_t server_addr_len = sizeof(server_addr);
  char ip[INET_ADDRSTRLEN];

  while (true) {
    if (recvfrom(socket_desc, msg, sizeof(message), 0,
                 (struct sockaddr *)&server_addr, &server_addr_len) < 0) {
      printf("Couldn't receive\n");
      return -1;
    }

    // Convert IP address from binary to text form
    inet_ntop(AF_INET, &(server_addr.sin_addr), ip, INET_ADDRSTRLEN);

    std::cout << "\rn_arrays: " << static_cast<int>(msg->n_arrays) << std::endl;
    std::cout << "\rsrc_addrs: " << ip << std::endl;

    if (strcmp(ip, ip_addresses[id]) == 0) {
      config->arrays_ = msg->n_arrays;
      config->n_sensors_ = msg->n_arrays * ELEMENTS;
      std::cout << "\rn_sensors_ RECEIVER: " << config->n_sensors_ << std::endl;

      return msg->n_arrays * ELEMENTS;
    }
  }

  std::cerr << "Unknown IP: " << ip << std::endl;
}

#if 0 //TODO: N_SENSORS NEED TO BE UPDATED
float sine[2 * 48828];



void pack_buffer(float *data, message *msg2, unsigned offset) {
  int inverted = 1;

  unsigned index;

  std::cout << std::endl;
  
  for (unsigned i = 0; i < N_SENSORS; i++) {

    // Array is daisy-chained, flip even columns
    if (i % COLUMNS == 0) {
      inverted = !inverted;
    }

    if (inverted) {
      index = COLUMNS * (1 + i / COLUMNS) - i % COLUMNS;
    } else {
      index = i;
    }
    //std::cout << i * N_SAMPLES + offset<<" " <<index << " " << offset << std::endl;

    data[i * N_SAMPLES + offset] = (float)msg2->stream[index] / (float)MAX_VALUE_FLOAT;
  }
} 



int main() { //TODO: N_SENSORS NEED TO BE UPDATED
  if (init_receiver() == -1) {
    std::cerr << "Unable to establish a connection to antenna" << std::endl;
    return -1;
  }
 

  Streams *streams = new Streams();

  for (int s = 0; s < number_of_sensors(); s++) {
    streams->create_stream(s);
  }

  for (int i = 0; i < 10; i++)
    receive_exposure(streams);



  float data[N_SAMPLES];

  streams->read_stream(255, &data[0]);

  //memcpy(&msg, &buffer, sizeof(message_t));


//  uint32_t header, counter;
//
//  header = packet[0];
//  counter = packet[1];
//
//  int32_t version, n_arrays, frequency;
//
//  // Order as defined in FPGA
//  frequency = header & 0x0000FFFF;
//  n_arrays = (header & 0x00FF0000) >> 16;
//  version = (header & 0xFF000000) >> 24;

#if 1
  for (int i = 0; i < N_SAMPLES; i++) {
    std::cout << data[i] << std::endl;
  }
#endif
  printf("Version: %d\nArrays: %d\nFrequency: %d\nCounter: %d\n", msg->version,
         msg->n_arrays, msg->frequency, msg->counter);

  //std::cout << std::endl;
  //for (int i = 0; i < N_SENSORS; i++) {
  //  std::cout << (float)(std::uint32_t)msg.stream[i] / MAX_VALUE_FLOAT << std::endl;
  //}

  delete streams;


  stop_receiving();
}
#endif

#if 0 //TODO: N_SENSORS NEED TO BE UPDATED

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "RtAudio.h"
#include "config.h"
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
