#ifndef PIPELINE_H
#define PIPELINE_H

//#include "ring_buffer.h"
#include "streams.hpp"

#include "pipeline.h"
#include "receiver.h"
//#include "ring_buffer.h"
#include <fstream>
#include <iostream>
#include <string>

#include <condition_variable>
#include <mutex>
#include <thread>

#include <atomic>

#include <cstddef>
#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

/**
 * @class Pipeline
 * @brief Establish a pipeline to the FPGA from the PC
 *
 */
class Pipeline {

public:
  std::atomic_int canPlot = 0;
  cv::Mat *magnitudeHeatmap;
  Pipeline();
  ~Pipeline();
  /**
   * Connect beamformer to antenna
   */
  int connect();

  /**
   * Disconnect beamformer from antenna
   */
  int disconnect();

  /**
   * Checks if the pipeline is online
   */
  int isRunning();

  /**
   * check if no new data has been added
   */
  int mostRecent();

  /**
   * Wait until release
   */
  void barrier();
  
  Streams *getStreams();

  int save_pipeline(std::string path);

private:
  Streams *streams;
  int connected = 0;
  mutex pool_mutex;
  mutex barrier_mutex;
  condition_variable barrier_condition;
  int barrier_count = 0;

  thread connection;

  int modified = 0;

  int p = 0;

  float buffer[N_SAMPLES * N_SENSORS * 2];

  /**
   * Allow the worker threads to continue
   */
  void release_barrier();

  /**
   * The main distributer of data to the threads (this is also a thread)
   */
  void producer();
};

#endif
