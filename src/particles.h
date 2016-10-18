#ifndef PARTICLES_H
#define PARTICLES_H

#include "pfuclt_aux.h"

#include <vector>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <boost/random.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>

#include <read_omni_dataset/RobotState.h>
#include <read_omni_dataset/LRMGTData.h>
#include <pfuclt_omni_dataset/particle.h>
#include <pfuclt_omni_dataset/particles.h>

// ideally later this will be a parameter, when it makes sense to
#define STATES_PER_TARGET 3

// offsets
#define O_X (0)
#define O_Y (1)
#define O_THETA (2)
#define O_TARGET (nRobots_ * nStatesPerRobot_)
#define O_TX (0)
#define O_TY (1)
#define O_TZ (2)
#define O_WEIGHT (nSubParticleSets_ - 1)

// target motion model and estimator
#define MAX_ESTIMATOR_STACK_SIZE 25
#define TARGET_RAND_MEAN 0
#define TARGET_RAND_STDDEV 5.0

// concerning time
#define ITERATION_TIME_DEFAULT 0.0333
#define ITERATION_TIME_NA (-1)
#define ITERATION_TIME_MAX (1)

// others
#define MIN_WEIGHTSUM 1e-7

namespace pfuclt_ptcls
{

typedef float pdata_t;

using namespace pfuclt_aux;

typedef double (*estimatorFunc)(const std::vector<double>&,
                                const std::vector<double>&);

typedef struct odometry_s
{
  pdata_t x, y, theta;
} Odometry;

typedef struct landmarkObs_s
{
  bool found;
  double x, y;
  double d, phi;
  double covDD, covPP, covXX, covYY;
  landmarkObs_s() { found = false; }
} LandmarkObservation;

typedef struct targetObs_s
{
  bool found;
  double x, y, z;
  double d, phi;
  double covDD, covPP, covXX, covYY;

  targetObs_s() { found = false; }
} TargetObservation;

// Apply concept of subparticles (the particle set for each dimension)
typedef std::vector<pdata_t> subparticles_t;
typedef std::vector<subparticles_t> particles_t;

// This will be the generator use for randomizing
typedef boost::random::mt19937 RNGType;

class ParticleFilter
{
protected:
  /**
   * @brief The state_s struct - defines a structure to hold state information
   * for the particle filter class
   */
  struct State
  {
    uint nRobots;
    uint nStatesPerRobot;
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
      std::vector<pdata_t> pose;
      pdata_t conf;

      robotState_s(uint poseSize) : pose(poseSize, 0.0), conf(0.0) {}
    };
    std::vector<struct robotState_s> robots;

    /**
     * @brief The targetState_s struct - saves information on the belief of the
     * target's state
     */
    struct targetState_s
    {
      std::vector<pdata_t> pos;
      std::vector<pdata_t> vel;

      targetState_s() : pos(STATES_PER_TARGET, 0.0), vel(STATES_PER_TARGET, 0.0)
      {
      }
    } target;

    /**
     * @brief The targetVelocityEstimator_s struct - will estimate the velocity
     * of a target using a custom function. The struct keeps vectors with the
     * latest avilable information, up until a max data amount is reached
     */
    struct targetVelocityEstimator_s
    {
      std::vector<double> timeVec;
      std::vector<std::vector<double> > posVec;
      estimatorFunc estimateVelocity;

      uint maxDataSize;
      uint timeInit;
      uint numberVels;

      targetVelocityEstimator_s(const uint _numberVels, const uint _maxDataSize,
                                estimatorFunc ptrFunc)
        : numberVels(_numberVels), posVec(_numberVels, std::vector<double>()),
          maxDataSize(_maxDataSize)
      {
        estimateVelocity = ptrFunc;
      }

      void insert(const double timeData, const std::vector<double>& posData)
      {
        if (timeVec.empty())
          timeInit = ros::Time::now().toNSec() * 1e-9;

        timeVec.push_back(timeData - timeInit);
        for (uint velType = 0; velType < posVec.size(); ++velType)
          posVec[velType].push_back(posData[velType]);

        if (timeVec.size() > maxDataSize)
        {
          timeVec.erase(timeVec.begin());
          for (uint velType = 0; velType < posVec.size(); ++velType)
            posVec[velType].erase(posVec[velType].begin());
        }
      }

      bool isReadyToEstimate() { return (timeVec.size() == maxDataSize); }

      double estimate(uint velType)
      {
        double velEst = estimateVelocity(timeVec, posVec[velType]);
        ROS_DEBUG("Estimated velocity type %d = %f", velType, velEst);

        return velEst;
      }
    } targetVelocityEstimator;

    /**
     * @brief State - constructor
     */
    State(const uint nStatesPerRobot_, const uint numberRobots,
          const std::vector<bool>& robotsBeingUsed)
      : nStatesPerRobot(nStatesPerRobot_), nRobots(numberRobots),
        predicted(nRobots, false), landmarkMeasurementsDone(nRobots, false),
        targetMeasurementsDone(nRobots, false), robotsUsed(robotsBeingUsed),
        targetVelocityEstimator(STATES_PER_TARGET, MAX_ESTIMATOR_STACK_SIZE,
                                pfuclt_aux::linearRegressionSlope)
    {
      reset();

      // Create and initialize the robots vector
      for (uint r = 0; r < nRobots; ++r)
        robots.push_back(robotState_s(nStatesPerRobot));
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
    void print()
    {
      std::ostringstream oss;
      oss << "PF State:" << std::endl;
      for (uint r = 0; r < nRobots; ++r)
      {
        oss << "OMNI " << r + 1 << "[ ";
        for (uint k = 0; k < nStatesPerRobot; ++k)
          oss << robots[r].pose[k] << " ";
        oss << "]" << std::endl;
      }

      oss << "Target [ ";
      for (uint k = 0; k < STATES_PER_TARGET; ++k)
        oss << target.pos[k] << " ";
      oss << "]" << std::endl;

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

public:
  /**
   * @brief The PFinitData struct - provides encapsulation to the initial data
   * necessary to construct a ParticleFilter instance
   */
  struct PFinitData
  {
    const uint nParticles, nTargets, statesPerRobot, nRobots, nLandmarks;
    const std::vector<bool>& robotsUsed;
    const std::vector<Landmark>& landmarksMap;
    std::vector<float> alpha;

    /**
     * @brief _PFinitData
     * @param _nParticles - the number of particles to be in the particle filter
     * @param _nTargets - the number of targets to consider
     * @param _statesPerRobot - the state space dimension for each robot
     * @param _nRobots - number of robots
     * @param _nLandmarks - number of landmarks
     * @param _robotsUsed - vector of bools mentioning if robots are being used,
     * according to the standard robot ordering
     * @param _landmarksMap - vector of Landmark structs containing information
     * on the landmark locations
     * @param _vector with values to be used in the RNG for the model sampling
     */
    PFinitData(const uint _nParticles, const uint _nTargets,
               const uint _statesPerRobot, const uint _nRobots,
               const uint _nLandmarks, const std::vector<bool>& _robotsUsed,
               const std::vector<Landmark>& _landmarksMap,
               const std::vector<float> _alpha = std::vector<float>())
      : nParticles(_nParticles), nTargets(_nTargets),
        statesPerRobot(_statesPerRobot), nRobots(_nRobots),
        nLandmarks(_nLandmarks), alpha(_alpha), robotsUsed(_robotsUsed),
        landmarksMap(_landmarksMap)
    {
      // If vector alpha is not provided, use a default one
      if (alpha.empty())
      {
        for (int r = 0; r < nRobots; ++r)
        {
          alpha.push_back(0.015);
          alpha.push_back(0.1);
          alpha.push_back(0.5);
          alpha.push_back(0.001);
        }
      }

      // Check size of vector alpha
      if (alpha.size() != 4 * nRobots)
      {
        ROS_ERROR(
              "The provided vector alpha is not of the correct size. Returning "
              "without particle filter! (should have %d=nRobots*4 elements)",
              nRobots * 4);
        return;
      }
    }
  };

protected:
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
  double iterationTime_;
  ros::Time prevTime_, newTime_;
  struct State state_;

  /**
   * @brief copyParticle - copies a whole particle from one particle set to
   * another
   * @param p_To - the destination particle set
   * @param p_From - the origin particle set
   * @param i_To - the index of the particle to copy to
   * @param i_From - the index of the particle to copy from
   * @remark Make sure the sizes of p_To and p_From are the same
   */
  inline void copyParticle(particles_t& p_To, particles_t& p_From, uint i_To,
                           uint i_From)
  {
    copyParticle(p_To, p_From, i_To, i_From, 0, p_To.size()-1);
  }

  /**
   * @brief copyParticle - copies some subparticle sets of a particle from one
   * particle set to another
   * @param p_To - the destination particle set
   * @param p_From - the origin particle set
   * @param i_To - the index of the particle to copy to
   * @param i_From - the index of the particle to copy from
   * @param subFirst - the first subparticle set index
   * @param subLast - the last subparticle set index
   * @remark Make sure the sizes of p_To and p_From are the same
   */
  inline void copyParticle(particles_t& p_To, particles_t& p_From, uint i_To,
                           uint i_From, uint subFirst, uint subLast)
  {
    for (uint k = subFirst; k <= subLast; ++k)
      p_To[k][i_To] = p_From[k][i_From];
  }

  /**
   * @brief resetWeights - assign the value val to all particle weights
   */
  inline void resetWeights(pdata_t val) { assign(val, O_WEIGHT); }

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
   * @brief modifiedMultinomialResampler - a costly resampler that keeps 50% of
   * the particles and implements the multinomial resampler on the rest
   */
  void modifiedMultinomialResampler(uint startAt);

  /**
   * @brief resample - the resampling step
   */
  void resample();

  /**
   * @brief resample - state estimation through weighted means, and linear
   * regression for the target velocity
   */
  void estimate();

  /**
   * @brief nextIteration - perform final steps and reset the PF state
   */
  virtual void nextIteration() { state_.reset(); }

public:
  boost::shared_ptr<std::ostringstream> iteration_oss;

  /**
   * @brief ParticleFilter - constructor
   * @param data - reference to a struct of PFinitData containing the necessary
   * information to construct the Particle Filter
   */
  ParticleFilter(struct PFinitData& data);

  void updateIterationTime(ros::Time tRos)
  {
    prevTime_ = newTime_;
    newTime_ = tRos;
    iterationTime_ = (newTime_ - prevTime_).toNSec() * 1e-9;
    if (fabs(iterationTime_) > 10)
    {
      // Something is wrong, set to default iteration time
      iterationTime_ = ITERATION_TIME_DEFAULT;
    }
    ROS_DEBUG("Target tracking iteration time: %f", iterationTime_);
  }

  ParticleFilter* getPFReference() { return this; }

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
   * @brief operator [] - array subscripting access to the private particle set
   * @param index - the subparticle set index number to access
   * @return the subparticles_t object reference located at particles_[index]
   */
  inline subparticles_t& operator[](int index) { return particles_[index]; }

  /**
   * @brief operator [] - const version of the array subscripting access, when
   * using it on const intantiations of the class
   * @param index - the subparticle set index number to access
   * @return a const subparticles_t object reference located at
   * particles_[index]
   */
  inline const subparticles_t& operator[](int index) const
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
  inline void saveLandmarkObservation(const uint robotNumber,
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
  inline void saveLandmarkObservation(const uint robotNumber,
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
  inline void saveTargetObservation(const uint robotNumber,
                             const TargetObservation obs)
  {
    bufTargetObservations_[robotNumber] = obs;
  }

  /**
   * @brief saveTargetObservation - change the measurement buffer state
   * @param robotNumber - the robot number in the team
   * @param _found - whether the target has been found
   */
  inline void saveTargetObservation(const uint robotNumber, const bool _found)
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

/**
 * @brief The PFPublisher class - implements publishing for the ParticleFilter
 * class using ROS
 */
class PFPublisher : public ParticleFilter
{
public:
  struct PublishData
  {
    ros::NodeHandle& nh;
    float robotHeight;

    /**
     * @brief PublishData - contains information necessary for the PFPublisher
     * class
     * @param _nh - the node handle object
     * @param _robotHeight - the fixed robot height
     */
    PublishData(ros::NodeHandle& _nh, float _robotHeight)
      : nh(_nh), robotHeight(_robotHeight)
    {
    }
  };

private:
  ros::Subscriber GT_sub_;
  ros::Publisher robotStatePublisher_, targetStatePublisher_,
  particlePublisher_, syncedGTPublisher_, targetEstimatePublisher_,
  targetGTPublisher_, targetParticlePublisher_;
  std::vector<ros::Publisher> particleStdPublishers_;
  std::vector<ros::Publisher> robotGTPublishers_;
  std::vector<ros::Publisher> robotEstimatePublishers_;

  read_omni_dataset::LRMGTData msg_GT_;
  pfuclt_omni_dataset::particles msg_particles_;
  read_omni_dataset::RobotState msg_state_;
  read_omni_dataset::BallData msg_target_;

  std::vector<tf2_ros::TransformBroadcaster> robotBroadcasters;

  struct PublishData pubData;

  void publishParticles();
  void publishRobotStates();
  void publishTargetState();
  void publishGTData();

public:
  /**
   * @brief PFPublisher - constructor
   * @param data - a structure with the necessary initializing data for the
   * ParticleFilter class
   * @param publishData - a structure with some more data for this class
   */
  PFPublisher(struct ParticleFilter::PFinitData& data,
              struct PublishData publishData);

  /**
   * @brief getPFReference - retrieve a reference to the base class's members
   * @remark C++ surely is awesome
   * @return returns a reference to the base ParticleFilter for this object
   */
  ParticleFilter* getPFReference() { return (ParticleFilter*)this; }

  /**
   * @brief gtDataCallback - callback of ground truth data
   */
  void gtDataCallback(const read_omni_dataset::LRMGTData::ConstPtr&);

  /**
   * @brief nextIteration - extends the base class method to add the ROS
   * publishing
   */
  void nextIteration();
};

// end of namespace pfuclt_ptcls
}
#endif // PARTICLES_H
