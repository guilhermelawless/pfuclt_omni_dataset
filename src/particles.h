#ifndef PARTICLES_H
#define PARTICLES_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <boost/random.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

// ideally later this will be a parameter, when it makes sense to
#define STATES_PER_TARGET 3
#define WEIGHT_INDEX (nSubParticleSets_ - 1)

// offsets
#define O_X (0)
#define O_Y (1)
#define O_THETA (2)
#define O_TARGET (nRobots_ * nStatesPerRobot_)
#define O_TX (0)
#define O_TY (1)
#define O_TZ (2)

// target motion model
#define TARGET_RAND_MEAN 0
#define TARGET_RAND_STDDEV 20.0

// concerning time
#define TIME_NOTAVAILABLE -1

namespace pfuclt_ptcls
{
typedef struct odometry_s
{
  float x, y, theta;
} Odometry;

typedef struct measurement_s
{
  double x, y;
  double d, phi;
  double covDD, covPP, covXX, covYY;
} Measurement;

typedef struct targetMotion_s
{

  bool started;
  double Vx, Vy, Vz;

  targetMotion_s()
  {
    started = false;
    Vx = Vy = Vz = 0.0;
  }
} TargetMotion;

// Apply concept of subparticles (the particle set for each dimension)
typedef float pdata_t;
typedef std::vector<float> subparticles_t;
typedef std::vector<subparticles_t> particles_t;

// This will be the generator use for randomizing
typedef boost::random::mt19937 RNGType;

class ParticleFilter
{
  /**
   * @brief The State enum - auxiliary enumeration to allow robots to know the
   * state of the particle filter concerning each robot
   */
  enum State
  {
    Predict,
    FuseRobot,
    FuseTarget,
    Resample,
    CalcVel
  };

  // TODO find if mutex is necessary while using the simple implemented state
  // machine
  // boost::mutex mutex();

private:
  uint nParticles_;
  uint nTargets_;
  uint nRobots_;
  uint nStatesPerRobot_;
  uint nSubParticleSets_;
  uint nLandmarks_;
  particles_t particles_;
  RNGType seed_;
  std::vector<float> alpha_;
  bool initialized_;
  std::vector<std::vector<Measurement> > bufMeasurements_;
  TargetMotion targetMotionState;

public:
  double iterationTimeS;
  std::vector<State> states;

  /**
   * @brief assign - assign a value to every particle in all subsets
   * @param value - the value to assign
   */
  void assign(const pdata_t value);

  /**
   * @brief assign - assign a value to every particle in one subset
   * @param value - the value to assign
   * @param index - the subset index [0,N]
   */
  void assign(const pdata_t value, const uint index);

  /**
   * @brief ParticleFilter - constructor
   * @param nParticles - the number of particles to be in the particle filter
   * @param nTargets - the number of targets to consider
   * @param statesPerRobot - the state space dimension for each robot
   * @param nRobots - number of robots
   * @param alpha - vector with values to be used in the RNG for the model
   * sampling
   */
  ParticleFilter(const uint nParticles, const uint nTargets,
                 const uint statesPerRobot, const uint nRobots,
                 const uint nLandmarks,
                 const std::vector<float> alpha = std::vector<float>());

  /**
   * @brief ParticleFilter - copy constructor. Will create and return a new
   * ParticleFilter object identical to the one provided
   * @param other - the ParticleFilter object to be copied
   */
  ParticleFilter(const ParticleFilter& other);

  /**
   * @brief operator = - copy assignment. Will copy other and return the new
   * ParticleFilter object
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
   * @brief operator [] - const version of the array subscripting access, when
   * using it on const intantiations of the class
   * @param index - the subparticle set index number to access
   * @return a const subparticles_t object reference located at
   * particles_[index]
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
   * @remark will not do anything if the current state of the robot robotNumber
   * is not Predict
   * @warning only for the omni dataset configuration
   */
  void predict(const uint robotNumber, const Odometry odom);

  /**
   * @brief isInitialized - simple interface to access private member
   * initialized_
   * @return true if particle filter has been initialized, false otherwise
   */
  bool isInitialized() { return initialized_; }

  /**
   * @brief size - interface to the size of the particle filter
   * @return - the number of subparticle sets
   */
  std::size_t size() { return nSubParticleSets_; }

  /**
   * @brief resetWeights - assign the value 1.0 to all particle weights
   */
  void resetWeights() { assign((pdata_t)1.0, WEIGHT_INDEX); }

  /**
   * @brief saveLandmarkObservation - saves the Measurement obs to a buffer of
   * observations
   * @param robotNumber - the robot number in the team
   * @param landmarkNumber - the landmark serial id
   * @param obs - the observation data as a structure defined in this file
   */
  void saveLandmarkObservation(const uint robotNumber,
                               const uint landmarkNumber, const Measurement obs)
  {
    bufMeasurements_[robotNumber][landmarkNumber] = obs;
  }

  /**
   * @brief saveTargetMotionState - saves the new state of the target
   * @param vel - array with the 3 velocities (x,y,z)
   */
  void saveTargetMotionState(const double vel[3]);
};

// end of namespace pfuclt_ptcls
}
#endif // PARTICLES_H
