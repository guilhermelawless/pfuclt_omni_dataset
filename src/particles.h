#ifndef PARTICLES_H
#define PARTICLES_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <boost/random.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

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
private:
  int nParticles_;
  int nDimensions_;
  uint nRobots_;
  uint statesPerRobot_;
  particles_t particles_;
  RNGType seed_;
  bool initialized_;

public:
  ParticleFilter(int nParticles, int nDimensions, uint statesPerRobot, uint nRobots);
  /**
   * @brief init - initialize the particle filter set with the default randomized values
   */
  void init();

  /**
   * @brief init - initialize the particle filter set with custom values
   * @param custom - vector of doubles with the following form: <lvalue, rvalue, lvalue, rvalue, ...>
   * which will be used as the limits of the uniform distribution.
   * This vector should have a size equal to twice the number of dimensions
   */
  void init(const std::vector<double> custom);

  /**
   * @brief predict - prediction step in the particle filter set with the received odometry
   * @param robotNumber - the robot number [0,N]
   * @param odometry - a structure containing the latest odometry readings
   * @remark will not do anything if the particle filter has not been initialized
   * @warning only for the omni dataset configuration
   */
  void predict(const uint robotNumber, const Eigen::Isometry2d& odometry);

  //method to get private initialized_
  bool isInitialized(){ return initialized_; }

  // array subscripting operator
  subparticles_t& operator[](int index) { return particles_[index]; }

  // implement const overloaded operator for a const object
  const subparticles_t& operator[](int index) const { return particles_[index]; }

  // assignment operator
  // particles operator=(const particles& copyThis) { copyThis.subparts_.; }
};

// end of namespace pfuclt_ptcls
}
#endif // PARTICLES_H
