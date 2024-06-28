#ifndef PSO_H
#define PSO_H

#include "../config.h"

#include "../antenna.h"
#include "../streams.hpp"

#if USE_KALMAN_FILTER
#include "../kf.h"
#endif



#include <Eigen/Dense>

#include <vector>

class Particle {
public:
  float theta, phi;
  float velocity_theta, velocity_phi;
  float best_theta, best_phi;
  float best_magnitude;

  Antenna &antenna;
  Streams *streams;

  //float (*objective_function)(float, float);

  Particle(Antenna &antenna, Streams *streams);

  void random();

  float compute(float theta, float phi);

  void update();
};



class PSO {
public:
  std::vector<Particle> particles;
  float global_best_theta, global_best_phi;
  float global_best_magnitude;
  int n_particles;
  Antenna &antenna;
  Streams *streams;

#if USE_KALMAN_FILTER
  KalmanFilter3D kf;
#endif

  //float (*objective_function)(float, float);

  PSO(int n_particles, Antenna &antenna, Streams *streams);

  void initialize_particles();

  void optimize(int iterations);

  Eigen::Vector3f sanitize();
};

#endif