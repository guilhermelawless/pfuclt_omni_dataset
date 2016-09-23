#ifndef PARTICLES_H
#define PARTICLES_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <boost/random.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

//ideally later this will be a parameter, when it makes sense to
#define STATES_PER_TARGET 3

namespace pfuclt_ptcls
{
// Apply concept of subparticles (the particle set for each dimension)
typedef float pdata_t;
typedef std::vector<float> subparticles_t;
typedef std::vector<subparticles_t> particles_t;

// This will be the generator use for randomizing
typedef boost::random::mt19937 RNGType;

class ParticleFilter
{
  boost::mutex mutex();

private:
  uint nParticles_;
  uint nTargets_;
  uint nRobots_;
  uint nStatesPerRobot_;
  uint nSubParticleSets_;
  particles_t particles_;
  RNGType seed_;
  bool initialized_;

public:
  /**
   * @brief ParticleFilter - constructor
   * @param nParticles - the number of particles to be in the particle filter
   * @param nTargets - the number of targets to consider
   * @param statesPerRobot - the state space dimension for each robot
   * @param nRobots - number of robots
   */
  ParticleFilter(const uint nParticles, const uint nTargets, const uint statesPerRobot, const uint nRobots);

  /**
   * @brief ParticleFilter - copy constructor. Will create and return a new ParticleFilter object identical to the one provided
   * @param other - the ParticleFilter object to be copied
   */
  ParticleFilter(const ParticleFilter& other);

  /**
   * @brief operator = - copy assignment. Will copy other and return the new ParticleFilter object
   * @param other
   * @return the copied object
   */
  ParticleFilter& operator=(const ParticleFilter& other);

  /**
   * @brief operator [] - array subscripting access to the private particle set
   * @param index - the subparticle set index number to access
   * @return the subparticles_t object reference located at particles_[index]
   */
  subparticles_t& operator[](int index) { return particles_[index]; }

  /**
   * @brief operator [] - const version of the array subscripting access, when using it on const intantiations of the class
   * @param index - the subparticle set index number to access
   * @return a const subparticles_t object reference located at particles_[index]
   */
  const subparticles_t& operator[](int index) const
  {
    return particles_[index];
  }

  /**
   * @brief init - initialize the particle filter set with the default
   * randomized values
   */
  void init();

  /**
   * @brief init - initialize the particle filter set with custom values
   * @param custom - vector of doubles with the following form: <lvalue, rvalue,
   * lvalue, rvalue, ...>
   * which will be used as the limits of the uniform distribution.
   * This vector should have a size equal to twice the number of dimensions
   */
  void init(const std::vector<double> custom);

  /**
   * @brief predict - prediction step in the particle filter set with the
   * received odometry
   * @param robotNumber - the robot number [0,N]
   * @param odometry - a structure containing the latest odometry readings
   * @remark will not do anything if the particle filter has not been
   * initialized
   * @warning only for the omni dataset configuration
   */
  void predict(uint robotNumber, const Eigen::Isometry2d& odometry);

  /**
   * @brief isInitialized - simple interface to access private member initialized_
   * @return true if particle filter has been initialized, false otherwise
   */
  bool isInitialized() { return initialized_; }

  /**
   * @brief size - interface to the size of the particle filter
   * @return - the number of subparticle sets
   */
  std::size_t size() { return particles_.size(); }
};

// end of namespace pfuclt_ptcls
}
#endif // PARTICLES_H
