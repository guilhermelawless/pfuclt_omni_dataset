#ifndef PARTICLES_H
#define PARTICLES_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <boost/random.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sstream>
#include "pfuclt_aux.h"

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
#define ROS_TIME_SEC (ros::Time::now().toSec())

namespace pfuclt_ptcls
{
using namespace pfuclt_aux;

typedef struct odometry_s
{
  float x, y, theta;
} Odometry;

typedef struct landmarkObs_s
{
  bool found;
  double x, y;
  double d, phi;
  double covDD, covPP, covXX, covYY;
} LandmarkObservation;

typedef struct targetObs_s
{
  bool found;
  double x, y, z;
  double d, phi;
  double covDD, covPP, covXX, covYY;
} TargetObservation;

// Apply concept of subparticles (the particle set for each dimension)
typedef float pdata_t;
typedef std::vector<pdata_t> subparticles_t;
typedef std::vector<subparticles_t> particles_t;

// This will be the generator use for randomizing
typedef boost::random::mt19937 RNGType;

class ParticleFilter
{
  /**
   * @brief The state_s struct - defines a structure to hold state information
   * for the particle filter class
   */
  struct State
  {
    uint nRobots;
    const std::vector<bool>& robotsUsed;
    std::vector<bool> predicted;
    std::vector<bool> landmarkMeasurementsDone;
    std::vector<bool> targetMeasurementsDone;
    bool targetPredicted;
    bool robotsFused;
    bool targetFused;
    bool resampled;

    /**
     * @brief The robotState_s struct - saves information on the belief of a
     * robot's state
     */
    struct robotState_s
    {
      float x, y, theta;
      float conf;
    };
    std::vector<struct robotState_s> robots;

    /**
     * @brief The targetState_s struct - saves information on the belief of the
     * target's state
     */
    struct targetState_s
    {
      float x, y, z;
      float vx, vy, vz;
    } target;

    /**
     * @brief State - constructor
     * @param numberRobots
     */
    State(const uint numberRobots, const std::vector<bool>& robotsBeingUsed)
      : nRobots(numberRobots), predicted(nRobots, false),
        landmarkMeasurementsDone(nRobots, false),
        targetMeasurementsDone(nRobots, false), robots(nRobots),
        robotsUsed(robotsBeingUsed)
    {
      reset();
    }

  private:
    void print(std::ostringstream& oss, std::vector<bool>& vec)
    {
      oss << "[";

      for (std::vector<bool>::iterator it = vec.begin(); it != vec.end(); ++it)
        oss << *it;

      oss << "]";
      return;
    }

  public:
    // TODO add state print
    void print()
    {
      std::ostringstream oss;
      oss << "PF State:";
      oss << std::endl
          << "-predicted = ";
      print(oss, predicted);
      oss << std::endl
          << "-landmarkMeasurementsDone = ";
      print(oss, landmarkMeasurementsDone);
      oss << std::endl
          << "-targetMeasurementsDone = ";
      print(oss, targetMeasurementsDone);
      oss << std::endl
          << "-targetPredicted = " << targetPredicted;
      oss << std::endl
          << "-robotsFused = " << robotsFused;
      oss << std::endl
          << "-targetFused = " << targetFused;
      oss << std::endl
          << "-resampled = " << resampled;

      ROS_DEBUG("%s", oss.str().c_str());
    }

    /**
     * @brief reset - set the state's bools to false after the PF's last step.
     * If any robot is not used, sets those variables to true
     */
    void reset()
    {
      predicted.assign(nRobots, false);
      landmarkMeasurementsDone.assign(nRobots, false);
      targetMeasurementsDone.assign(nRobots, false);
      robotsFused = targetFused = resampled = targetPredicted = false;

      for (uint r = 0; r < nRobots; ++r)
      {
        if (!robotsUsed[r])
        {
          predicted[r] = true;
          landmarkMeasurementsDone[r] = true;
          targetMeasurementsDone[r] = true;
        }
      }
    }

    /**
     * @brief allPredicted - call to find out if the PF has predicted each
     * robot's state
     * @return true if the PF has predicted all robots in this iteration, false
     * otherwise
     */
    bool allPredicted()
    {
      for (std::vector<bool>::iterator it = predicted.begin();
           it != predicted.end(); ++it)
      {
        if (false == *it)
          return false;
      }

      return true;
    }

    /**
     * @brief allLandmarkMeasurementsDone - call to find out if all the robots
     * have sent their landmark measurements to the PF in this iteration
     * @return true if all measurements are present, false otherwise
     */
    bool allLandmarkMeasurementsDone()
    {
      for (std::vector<bool>::iterator it = landmarkMeasurementsDone.begin();
           it != landmarkMeasurementsDone.end(); ++it)
      {
        if (false == *it)
          return false;
      }

      return true;
    }

    /**
     * @brief allTargetMeasurementsDone - call to find out if all the robots
     * have sent their target measurements to the PF in this iteration
     * @return true if all measurements are present, false otherwise
     */
    bool allTargetMeasurementsDone()
    {
      for (std::vector<bool>::iterator it = targetMeasurementsDone.begin();
           it != targetMeasurementsDone.end(); ++it)
      {
        if (false == *it)
          return false;
      }

      return true;
    }
  };

  // TODO find if mutex is necessary while using the simple implemented state
  // machine
  // boost::mutex mutex();

private:
  const std::vector<Landmark>& landmarksMap_;
  const std::vector<bool>& robotsUsed_;
  const uint nParticles_;
  const uint nTargets_;
  const uint nRobots_;
  const uint nStatesPerRobot_;
  const uint nSubParticleSets_;
  const uint nLandmarks_;
  particles_t particles_;
  particles_t weightComponents_;
  RNGType seed_;
  std::vector<float> alpha_;
  bool initialized_;
  std::vector<std::vector<LandmarkObservation> > bufLandmarkObservations_;
  std::vector<TargetObservation> bufTargetObservations_;

  /**
   * @brief resetWeights - assign the value 1.0 to all particle weights
   */
  void resetWeights()
  {
    assign((pdata_t)1.0, WEIGHT_INDEX);
    /*
    for(uint s=0; s < weightComponents_.size(); ++s)
      weightComponents_[s].assign(nParticles_, 1.0);
    */
  }

  /**
   * @brief predictTarget - predict target state step
   * @param robotNumber - the robot performing, for debugging purposes
   */
  void predictTarget(uint robotNumber);

  /**
   * @brief fuseRobots - fuse robot states step
   */
  void fuseRobots();

  /**
   * @brief fuseTarget - fuse target state step
   */
  void fuseTarget();

  /**
   * @brief low_variance_resampler - implementation of the resampler by Thrun and Burgard
   */
  void low_variance_resampler(const float weightSum);

  /**
   * @brief myResampler - a simpler resampler
   */
  void myResampler(const float weightSum);

  /**
   * @brief resample - the resampling step
   */
  void resample();

public:
  double prevTime, iterationTime;
  struct State state;
  boost::shared_ptr<std::ostringstream> iteration_oss;

  /**
   * @brief printWeights
   */
  void printWeights(std::string pre);

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
                 const uint nLandmarks, const std::vector<bool>& robotsUsed,
                 const std::vector<Landmark>& landmarksMap,
                 const std::vector<float> alpha = std::vector<float>());

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
   * @brief saveLandmarkObservation - saves the landmark observation to a buffer
   * of
   * observations
   * @param robotNumber - the robot number in the team
   * @param landmarkNumber - the landmark serial id
   * @param obs - the observation data as a structure defined in this file
   */
  void saveLandmarkObservation(const uint robotNumber,
                               const uint landmarkNumber,
                               const LandmarkObservation obs)
  {
    bufLandmarkObservations_[robotNumber][landmarkNumber] = obs;
  }

  /**
   * @brief saveLandmarkObservation - change the measurement buffer state
   * @param robotNumber - the robot number in the team
   * @param _found - whether this landmark has been found
   */
  void saveLandmarkObservation(const uint robotNumber,
                               const uint landmarkNumber, const bool _found)
  {
    bufLandmarkObservations_[robotNumber][landmarkNumber].found = _found;
  }

  /**
   * @brief saveAllLandmarkMeasurementsDone - call this function when all
   * landmark measurements have
   * been performed by a certain robot
   * @param robotNumber - the robot number performing the measurements
   */
  void saveAllLandmarkMeasurementsDone(const uint robotNumber);

  /**
   * @brief saveTargetObservation - saves the target observation to a buffer of
   * observations
   * @param robotNumber - the robot number in the team
   * @param obs - the observation data as a structure defined in this file
   */
  void saveTargetObservation(const uint robotNumber,
                             const TargetObservation obs)
  {
    bufTargetObservations_[robotNumber] = obs;
  }

  /**
   * @brief saveTargetObservation - change the measurement buffer state
   * @param robotNumber - the robot number in the team
   * @param _found - whether the target has been found
   */
  void saveTargetObservation(const uint robotNumber, const bool _found)
  {
    bufTargetObservations_[robotNumber].found = _found;
  }

  /**
   * @brief saveAllTargetMeasurementsDone - call this function when all target
   * measurements have
   * been performed by a certain robot
   * @param robotNumber - the robot number performing the measurements
   */
  void saveAllTargetMeasurementsDone(const uint robotNumber);
};

// end of namespace pfuclt_ptcls
}
#endif // PARTICLES_H
