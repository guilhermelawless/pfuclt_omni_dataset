#include "particles.h"
#include "ros/ros.h"
#include <stdlib.h>
#include <boost/random.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>

namespace pfuclt_ptcls
{
ParticleFilter::ParticleFilter(const uint nParticles, const uint nTargets, const uint statesPerRobot, const uint nRobots)
  : nParticles_(nParticles), nTargets_(nTargets), nStatesPerRobot_(statesPerRobot), nRobots_(nRobots), nSubParticleSets_(nTargets*STATES_PER_TARGET + nRobots*statesPerRobot + 1),
    particles_(nSubParticleSets_, subparticles_t(nParticles)), seed_(time(0)),
    initialized_(false)
{
  int size[2];
  size[0] = (int)particles_.size();
  size[1] = (int)particles_[0].size();

  ROS_INFO("Created particle filter with dimensions %d, %d", size[0], size[1]);
}

ParticleFilter::ParticleFilter(const ParticleFilter& other)
  : nParticles_(other.nParticles_), nSubParticleSets_(other.nSubParticleSets_),
    nStatesPerRobot_(other.nStatesPerRobot_), nRobots_(other.nRobots_),
    particles_(other.nSubParticleSets_, subparticles_t(other.nParticles_)), seed_(time(0)),
    initialized_(other.initialized_)
{
  ROS_DEBUG("Creating copy of another pf");

  // Only thing left is to copy particle values
  particles_ = other.particles_;  //std::vector assignment operator! yay C++
}

ParticleFilter &ParticleFilter::operator=(const ParticleFilter &other)
{
  // The default operator won't to because we don't want to pass the same RNG seed

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
void ParticleFilter::init()
{
  if (initialized_)
    return;

  int lvalue = -10;
  int rvalue = 10;

  std::vector<double> def((nSubParticleSets_ - 1) * 2);

  for (int i = 0; i < def.size(); i += 2)
  {
    def[i] = lvalue;
    def[i + 1] = rvalue;
  }

  // Call the custom function with these values
  init(def);
}

// Overloaded fucntion when using custom values
void ParticleFilter::init(const std::vector<double> custom)
{
  if (initialized_)
    return;

  ROS_INFO("Initializing particle filter");

  // For all subparticle sets except the particle weights
  for (int i = 0; i < custom.size() / 2; ++i)
  {
    ROS_DEBUG("Values for distribution: %.4f %.4f", custom[2 * i],
        custom[2 * i + 1]);

    boost::random::uniform_real_distribution<> dist(custom[2 * i],
        custom[2 * i + 1]);

    // Sample a value from the uniform distribution for each particle
    BOOST_FOREACH (pdata_t& val, particles_[i])
    {
      val = dist(seed_);
    }
  }

  // Particle weights init with same weight (1/nParticles)
  particles_[nSubParticleSets_ - 1].assign(nParticles_, 1 / nParticles_);

  // Set flag
  initialized_ = true;

  ROS_INFO("Particle filter initialized");
}

void ParticleFilter::predict(uint robotNumber,
                             const Eigen::Isometry2d& odometry)
{
  if (!initialized_)
    return;

  Eigen::Isometry2d prevParticle, curParticle;

  // use non-0-index logic (copied value)
  robotNumber--;

  // lock mutex
  boost::mutex::scoped_lock(mutex);

  for (uint i = 0; i < nParticles_; i++)
  {
    prevParticle = Eigen::Rotation2Dd(particles_[2 + robotNumber * 3][i])
        .toRotationMatrix();
    prevParticle.translation() =
        Eigen::Vector2d(particles_[0 + (robotNumber - 1) * 3][i],
        particles_[1 + (robotNumber - 1) * 3][i]);
    curParticle = prevParticle * odometry;
    particles_[0 + (robotNumber - 1) * 3][i] = curParticle.translation()[0];
    particles_[1 + (robotNumber - 1) * 3][i] = curParticle.translation()[1];
    Eigen::Matrix<double, 2, 2> r_particle = curParticle.linear();
    double angle_particle = acos(r_particle(0, 0));
    particles_[2 + (robotNumber - 1) * 3][i] = angle_particle;
  }
}

// end of namespace pfuclt_ptcls
}
