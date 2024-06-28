#include "pso_seeker.h"

#define VALID_SENSOR(i) (64 <= i) && (i < 128)


float beamform(Streams *streams, int *offset_delays, float *fractional_delays) {
  float out[N_SAMPLES] = {0.0f};
  unsigned n = 0;

  for (unsigned s = 0; s < N_SENSORS; s++) {
    if (VALID_SENSOR(s)) {
      float fraction = fractional_delays[s - 64];
      int offset = offset_delays[s - 64];

      float *signal = (float *)((char *)streams->buffers[s] +
                                streams->position + offset * sizeof(float));

      for (int i = 0; i < N_SAMPLES; i++) {
        out[i] += signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
      }

      n++;
    }
  }

  float power = 0.0f;
  float norm = 1 /(float)n;

  for (int i = 0; i < N_SAMPLES; i++) {
    power += powf(out[i] * norm, 2);
  }

  return power / (float)N_SAMPLES;

}


Particle::Particle(Antenna &antenna, Streams *streams) : antenna(antenna), streams(streams) {
    theta = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0 * ANGLE_LIMIT;
    phi = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0 * ANGLE_LIMIT;
    velocity_theta = 0.0f;
    velocity_phi = 0.0f;
    best_theta = theta;
    best_phi = phi;
    best_magnitude = compute(theta, phi);
}


void Particle::random() {
  theta = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0 * ANGLE_LIMIT;
  phi = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0 * ANGLE_LIMIT;
}

float Particle::compute(float theta, float phi) {
  Eigen::VectorXf tmp_delays = steering_vector(antenna, phi, theta);
  float fractional_delays[N_SENSORS];
  int offset_delays[N_SENSORS];
  int i = 0;
  for (float del : tmp_delays) {
    double _offset;
    float fraction;
    fraction = (float)modf((double)del, &_offset);
    int offset = N_SAMPLES - (int)_offset;
    // cout << del << endl;
    fractional_delays[i] = fraction;
    offset_delays[i] = offset;
    i++;
  }
  return beamform(streams, &offset_delays[0], &fractional_delays[0]);
}

void Particle::update() {
  float magnitude = compute(theta, phi);
  best_magnitude *= 0.99;
  if (magnitude > best_magnitude) {
    best_magnitude = magnitude;
    best_theta = theta;
    best_phi = phi;
  }
}




inline float clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

inline float wrapAngle( float angle )
{
    double twoPi = 2.0 * 3.141592865358979;
    return (float)(angle - twoPi * floor( angle / twoPi ));
}


PSO::PSO(int n_particles, Antenna &antenna, Streams *streams) : antenna(antenna), streams(streams), kf(0.1) {
  this->n_particles = n_particles;
  global_best_magnitude = 0.0f;
  initialize_particles();
}

void PSO::initialize_particles() {
  global_best_magnitude *= 0.9f;
  particles.clear();
  for (int i = 0; i < n_particles; i++) {
    particles.emplace_back(antenna, streams);
    if (particles.back().best_magnitude > global_best_magnitude) {
      global_best_magnitude = particles.back().best_magnitude;
      global_best_theta = particles.back().best_theta;
      global_best_phi = particles.back().best_phi;
    }
  }
}

void PSO::optimize(int iterations) {
  float w = 0.5f, c1 = 2.0f, c2 = 2.0f;
  float delta = to_radians(FOV / iterations * 2.0);
  global_best_magnitude *= 0.9f;
  for (auto& particle : particles) {
    particle.random();
  }
  for (int i = 0; i < iterations; i++) {
    for (auto& particle : particles) {
      particle.velocity_theta = w * particle.velocity_theta \
      + c1 * static_cast<float>(rand()) / RAND_MAX * (particle.best_theta - particle.theta) \
      + c2 * static_cast<float>(rand()) / RAND_MAX * (global_best_theta - particle.theta);
      particle.velocity_phi = w * particle.velocity_phi \
      + c1 * static_cast<float>(rand()) / RAND_MAX * (particle.best_phi - particle.phi) \
      + c2 * static_cast<float>(rand()) / RAND_MAX * (global_best_phi - particle.phi);
      particle.theta += particle.velocity_theta * delta;
      particle.phi += particle.velocity_phi * delta;
      
      //particle.theta = wrapAngle(particle.theta);
      //particle.theta %= (float)(2.0*M_PI); //clip(particle.theta, -ANGLE_LIMIT, ANGLE_LIMIT);
      //particle.phi = clip(particle.phi, to_radians(5.0), to_radians(80.0)); // Should be 2pi
      particle.theta = clip(particle.theta, -ANGLE_LIMIT, ANGLE_LIMIT);
      particle.phi = clip(particle.phi, -ANGLE_LIMIT, ANGLE_LIMIT);
      particle.update();
      if (particle.best_magnitude > global_best_magnitude) {
        global_best_magnitude = particle.best_magnitude;
        global_best_theta = particle.best_theta;
        global_best_phi = particle.best_phi;
      }
    }
  }
}

Eigen::Vector3f PSO::sanitize() {
  Eigen::Vector3f sample(global_best_theta, global_best_phi, 0.0);

#if USE_KALMAN_FILTER
  kf.update(sample);
  return kf.getState();
#else
  return sample;
#endif
}