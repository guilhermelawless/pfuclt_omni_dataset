#include "particles.h"
#include "ros/ros.h"
#include <stdlib.h>
#include <boost/random.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>
#include <angles/angles.h>

namespace pfuclt_ptcls {
void ParticleFilter::assign(const pdata_t value) {
  for (int i = 0; i < nSubParticleSets_; ++i)
    assign(value, i);
}

void ParticleFilter::assign(const pdata_t value, const uint index) {
  particles_[index].assign(nParticles_, value);
}

ParticleFilter::ParticleFilter(const uint nParticles, const uint nTargets,
                               const uint statesPerRobot, const uint nRobots,
                               const uint nLandmarks, const std::vector<float> alpha)
  : nParticles_(nParticles), nTargets_(nTargets),
    nStatesPerRobot_(statesPerRobot), nRobots_(nRobots),
    nSubParticleSets_(nTargets * STATES_PER_TARGET +
                      nRobots * statesPerRobot + 1),
    nLandmarks_(nLandmarks),
    particles_(nSubParticleSets_, subparticles_t(nParticles)), seed_(time(0)),
    initialized_(false), states(nRobots), alpha_(alpha),
    bufMeasurements_(nRobots, std::vector<Measurement>(nLandmarks)) {

  // If vector alpha is not provided, use a default one
  if (alpha_.empty()) {
    for (int r = 0; r < nRobots; ++r) {
      alpha_.push_back(0.015);
      alpha_.push_back(0.1);
      alpha_.push_back(0.5);
      alpha_.push_back(0.001);
    }
  }

  // Check size of vector alpha
  if (alpha_.size() != 4 * nRobots) {
    ROS_ERROR("The provided vector alpha is not of the correct size. Returning "
              "without particle filter! (should have %d=nRobots*4 elements)",
              nRobots * 4);
    return;
  }

  ROS_INFO("Created particle filter with dimensions %d, %d",
           (int)particles_.size(), (int)particles_[0].size());

  // Initiate states
  states.assign(states.size(), Predict);
}

ParticleFilter::ParticleFilter(const ParticleFilter &other)
  : nParticles_(other.nParticles_),
    nSubParticleSets_(other.nSubParticleSets_),
    nStatesPerRobot_(other.nStatesPerRobot_), nRobots_(other.nRobots_),
    particles_(other.nSubParticleSets_, subparticles_t(other.nParticles_)),
    seed_(time(0)), initialized_(other.initialized_) {
  ROS_DEBUG("Creating copy of another pf");

  // Only thing left is to copy particle values
  particles_ = other.particles_; // std::vector assignment operator! yay C++
}

ParticleFilter &ParticleFilter::operator=(const ParticleFilter &other) {
  // The default operator won't to because we don't want to pass the same RNG
  // seed

  ROS_DEBUG("Copying a pf to another pf");

  nParticles_ = other.nParticles_;
  nSubParticleSets_ = other.nSubParticleSets_;
  nRobots_ = other.nRobots_;
  nStatesPerRobot_ = other.nStatesPerRobot_;
  initialized_ = other.initialized_;
  particles_ = other.particles_;
}

// TODO set different values for position and orientation, targets, etc
// Simple version, use default values - randomize all values between [-10,10]
void ParticleFilter::init() {
  if (initialized_)
    return;

  int lvalue = -10;
  int rvalue = 10;

  std::vector<double> def((nSubParticleSets_ - 1) * 2);

  for (int i = 0; i < def.size(); i += 2) {
    def[i] = lvalue;
    def[i + 1] = rvalue;
  }

  // Call the custom function with these values
  init(def);
}

// Overloaded function when using custom values
void ParticleFilter::init(const std::vector<double> custom) {
  if (initialized_)
    return;

  ROS_INFO("Initializing particle filter");

  ROS_DEBUG_COND(custom.size() != ((nSubParticleSets_ - 1) * 2),
                 "The provided vector for initilization does not have the "
                 "correct size (should have %d elements",
                 (nSubParticleSets_ - 1) * 2);

  // For all subparticle sets except the particle weights
  for (int i = 0; i < custom.size() / 2; ++i) {
    ROS_DEBUG("Values for distribution: %.4f %.4f", custom[2 * i],
        custom[2 * i + 1]);

    boost::random::uniform_real_distribution<> dist(custom[2 * i],
        custom[2 * i + 1]);

    // Sample a value from the uniform distribution for each particle
    BOOST_FOREACH (pdata_t &val, particles_[i]) { val = dist(seed_); }
  }

  // Particle weights init with same weight (1/nParticles)
  resetWeights();

  // Set flag
  initialized_ = true;

  ROS_INFO("Particle filter initialized");
}

void ParticleFilter::predict(const uint robotNumber, const Odometry odom) {
  if (!initialized_ || states[robotNumber] != Predict)
    return;

  using namespace boost::random;

  ROS_DEBUG("Predicting for OMNI%d", robotNumber + 1);

  // Variables concerning this robot specifically
  int robot_offset = robotNumber * nStatesPerRobot_;
  float alpha[4] = {alpha_[robotNumber * 4], alpha_[robotNumber * 4 + 1],
                    alpha_[robotNumber * 4 + 2], alpha_[robotNumber * 4 + 3]};

  // TODO find out if this model is correct
  // Determining the propagation of the robot state through odometry
  float deltaRot = atan2(odom.y, odom.x) - odom.theta;
  float deltaTrans = sqrt(odom.x * odom.x + odom.y * odom.y);
  float deltaFinalRot = odom.theta - deltaRot;

  // Create an error model based on a gaussian distribution
  normal_distribution<> deltaRotEffective(deltaRot, alpha[0] * fabs(deltaRot) +
      alpha[1] * deltaTrans);
  normal_distribution<> deltaTransEffective(
        deltaTrans,
        alpha[2] * deltaTrans + alpha[3] * fabs(deltaRot + deltaFinalRot));
  normal_distribution<> deltaFinalRotEffective(
        deltaFinalRot, alpha[0] * fabs(deltaFinalRot) + alpha[1] * deltaTrans);

  // Debugging
  ROS_DEBUG("Before prediction: p[%d] = [%f %f %f]", nParticles_ / 2,
            particles_[O_X + robot_offset][nParticles_ / 2],
      particles_[O_Y + robot_offset][nParticles_ / 2],
      particles_[O_THETA + robot_offset][nParticles_ / 2]);
  ROS_DEBUG("Odometry = [%f %f %f]", odom.x, odom.y, odom.theta);

  for (int i = 0; i < nParticles_; i++) {
    // Rotate to final position
    particles_[O_THETA + robot_offset][i] += deltaRotEffective(seed_);

    // Translate to final position
    particles_[O_X + robot_offset][i] +=
        deltaTransEffective(seed_) * cos(particles_[O_THETA + robot_offset][i]);
    particles_[O_Y + robot_offset][i] +=
        deltaTransEffective(seed_) * sin(particles_[O_THETA + robot_offset][i]);

    // Rotate to final position and normalize angle
    particles_[O_THETA + robot_offset][i] = angles::normalize_angle(
          particles_[O_THETA + robot_offset][i] + deltaFinalRotEffective(seed_));
  }

  // Debugging
  ROS_DEBUG("After prediction: p[%d] = [%f %f %f]", nParticles_ / 2,
            particles_[O_X + robot_offset][nParticles_ / 2],
      particles_[O_Y + robot_offset][nParticles_ / 2],
      particles_[O_THETA + robot_offset][nParticles_ / 2]);

  // TODO predict ball velocity

  // Change the state from to FuseRobot
  states[robotNumber] = FuseRobot;
}

// end of namespace pfuclt_ptcls
}
