#ifndef PARTICLES_H
#define PARTICLES_H

#include <pfuclt_omni_dataset/pfuclt_aux.h>

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

#include <dynamic_reconfigure/server.h>
#include <pfuclt_omni_dataset/DynamicConfig.h>

#define NUM_ALPHAS 4

// ideally later this will be a parameter, when it makes sense to
#define STATES_PER_TARGET 3

// offsets
#define O_X (0)
#define O_Y (1)
#define O_THETA (2)
//#define O_TARGET (nRobots_ * nStatesPerRobot_)
#define O_TX (0)
#define O_TY (1)
#define O_TZ (2)
//#define O_WEIGHT (nSubParticleSets_ - 1)

// target motion model and estimator
#define TARGET_RAND_MEAN 0

// concerning time
#define TARGET_ITERATION_TIME_DEFAULT 0.0333
#define TARGET_ITERATION_TIME_MAX (1)

// others
#define MIN_WEIGHTSUM 1e-10

//#define MORE_DEBUG true

namespace pfuclt_ptcls
{

using namespace pfuclt_aux;

typedef float pdata_t;

typedef double (*estimatorFunc)(const std::vector<double>&,
                                const std::vector<double>&);

typedef struct odometry_s
{
  double x, y, theta;
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
private:
  boost::mutex mutex_;
  dynamic_reconfigure::Server<pfuclt_omni_dataset::DynamicConfig>
      dynamicServer_;

protected:
  struct dynamicVariables_s
  {
    bool firstCallback;
    int nParticles;
    int velocityEstimatorStackSize;
    double resamplingPercentageToKeep;
    double targetRandStddev;
    std::vector<std::vector<float> > alpha;

    dynamicVariables_s(ros::NodeHandle& nh, const uint nRobots);

    void fill_alpha(const uint robot, const std::string& str);

  } dynamicVariables_;

  /**
   * @brief The state_s struct - defines a structure to hold state information
   * for the particle filter class
   */
  struct State
  {
    uint nRobots;
    uint nStatesPerRobot;

    /**
     * @brief The robotState_s struct - saves information on the belief of a
     * robot's state
     */
    typedef struct robotState_s
    {
      std::vector<pdata_t> pose;
      pdata_t conf;

      robotState_s(uint poseSize) : pose(poseSize, 0.0), conf(0.0) {}
    } RobotState;
    std::vector<RobotState> robots;

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
      std::vector<std::vector<double> > timeVecs;
      std::vector<std::vector<double> > posVecs;
      std::vector<double> lastEstimates;
      estimatorFunc estimateVelocity;

      uint maxDataSize;
      uint timeInit;
      uint numberVels;

      targetVelocityEstimator_s(const uint numberVels, const int maxDataSize,
                                estimatorFunc ptrFunc)
          : numberVels(numberVels), posVecs(numberVels, std::vector<double>()),
            timeVecs(numberVels, std::vector<double>()),
            maxDataSize(maxDataSize), lastEstimates(numberVels)
      {
        // Pointer to the estimator function
        estimateVelocity = ptrFunc;

        // Reserve the data size
        for (uint velType = 0; velType < numberVels; ++velType)
        {
          timeVecs[velType].reserve(maxDataSize);
          posVecs[velType].reserve(maxDataSize);
        }

        ROS_INFO("Created a target velocity estimator");
      }

      void insert(const double timeData,
                  const std::vector<TargetObservation>& obsData,
                  const std::vector<RobotState>& robotStates);

      bool isReadyToEstimate()
      {
        for (uint velType = 0; velType < numberVels; ++velType)
        {
          if (timeVecs[velType].size() != timeVecs[velType].capacity())
            return false;
        }

        return true;
      }

      double estimate(const uint velType)
      {
        lastEstimates[velType] =
            estimateVelocity(timeVecs[velType], posVecs[velType]);
        return lastEstimates[velType];
      }

      void resize(const uint newStackSize);

      void reset()
      {
        for (uint velType = 0; velType < numberVels; ++velType)
        {
          timeVecs[velType].clear();
          posVecs[velType].clear();
        }
      }

    } targetVelocityEstimator;

    /**
     * @brief State - constructor
     */
    State(const uint nStatesPerRobot, const uint nRobots,
          const uint velocityEstimatorStackSize)
        : nStatesPerRobot(nStatesPerRobot), nRobots(nRobots),
          targetVelocityEstimator(STATES_PER_TARGET, velocityEstimatorStackSize,
                                  pfuclt_aux::linearRegressionSlope)
    {
      ROS_INFO("Target Velocity Estimator initialiazed with a stack size of %d",
               velocityEstimatorStackSize);

      // Create and initialize the robots vector
      for (uint r = 0; r < nRobots; ++r)
        robots.push_back(robotState_s(nStatesPerRobot));
    }
  };

public:
  /**
   * @brief The PFinitData struct - provides encapsulation to the initial data
   * necessary to construct a ParticleFilter instance
   */
  struct PFinitData
  {
    const uint mainRobotID, nTargets, statesPerRobot, nRobots, nLandmarks;
    const std::vector<bool>& robotsUsed;
    const std::vector<Landmark>& landmarksMap;
    ros::NodeHandle& nh;

    /**
     * @brief PFinitData
     * @param nh - the node handle
     * @param mainRobotID - the robot number where this algorithm will run on -
     * affects the timings of iteration and estimation updates - consider that
     * OMNI1 is ID1
     * @param nTargets - the number of targets to consider
     * @param statesPerRobot - the state space dimension for each robot
     * @param nRobots - number of robots
     * @param nLandmarks - number of landmarks
     * @param robotsUsed - vector of bools mentioning if robots are being used,
     * according to the standard robot ordering
     * @param landmarksMap - vector of Landmark structs containing information
     * on the landmark locations
     * @param vector with values to be used in the RNG for the model sampling
     */
    PFinitData(ros::NodeHandle& nh, const uint mainRobotID, const uint nTargets,
               const uint statesPerRobot, const uint nRobots,
               const uint nLandmarks, const std::vector<bool>& robotsUsed,
               const std::vector<Landmark>& landmarksMap)
        : nh(nh), mainRobotID(mainRobotID), nTargets(nTargets),
          statesPerRobot(statesPerRobot), nRobots(nRobots),
          nLandmarks(nLandmarks), robotsUsed(robotsUsed),
          landmarksMap(landmarksMap)
    {
    }
  };

protected:
  ros::NodeHandle& nh_;
  uint nParticles_;
  const uint mainRobotID_;
  const std::vector<Landmark>& landmarksMap_;
  const std::vector<bool>& robotsUsed_;
  const uint nTargets_;
  const uint nRobots_;
  const uint nStatesPerRobot_;
  const uint nSubParticleSets_;
  const uint nLandmarks_;
  particles_t particles_;
  particles_t weightComponents_;
  RNGType seed_;
  bool initialized_;
  std::vector<std::vector<LandmarkObservation> > bufLandmarkObservations_;
  std::vector<TargetObservation> bufTargetObservations_;
  TimeEval targetIterationTime_, odometryTime_, iterationTime_;
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
    copyParticle(p_To, p_From, i_To, i_From, 0, p_To.size() - 1);
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
  inline void resetWeights(pdata_t val)
  {
    particles_[O_WEIGHT].assign(particles_[O_WEIGHT].size(), val);
  }

  /**
   * @brief predictTarget - predict target state step
   * @param robotNumber - the robot performing, for debugging purposes
   */
  void predictTarget();

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
   * @brief nextIteration - perform final steps before next iteration
   */
  virtual void nextIteration() {}

  /**
   * @brief resize_particles - change to a different number of particles
   * @param n - the desired number of particles
   */
  virtual void resize_particles(const uint n)
  {
    size_t old_size = particles_[0].size();

    // Resize weightComponents
    for (uint r = 0; r < weightComponents_.size(); ++r)
      weightComponents_[r].resize(n);

    // Resize particles
    for (uint s = 0; s < particles_.size(); ++s)
      particles_[s].resize(n);

    // If n is lower than old_size, the last particles are removed - the ones
    // with the most weight are kept
    // But if n is higher, it's better to resample
    if (n > old_size)
      resample();
  }

public:
  boost::shared_ptr<std::ostringstream> iteration_oss;
  uint O_TARGET, O_WEIGHT;

  /**
   * @brief dynamicReconfigureCallback - Dynamic reconfigure callback for
   * dynamically setting variables during runtime
   */
  void dynamicReconfigureCallback(pfuclt_omni_dataset::DynamicConfig&);

  /**
   * @brief ParticleFilter - constructor
   * @param data - reference to a struct of PFinitData containing the necessary
   * information to construct the Particle Filter
   */
  ParticleFilter(struct PFinitData& data);

  /**
   * @brief updateTargetIterationTime - the main robot should call this method
   * after the target callback
   * @param tRos - time variable to be used in calculating the target's
   * iteration time
   */
  void updateTargetIterationTime(ros::Time tRos)
  {
    targetIterationTime_.updateTime(tRos);

    if (fabs(targetIterationTime_.diff) > TARGET_ITERATION_TIME_MAX)
    {
      // Something is wrong, set to default iteration time
      targetIterationTime_.diff = TARGET_ITERATION_TIME_DEFAULT;
    }
    ROS_DEBUG("Target tracking iteration time: %f", targetIterationTime_.diff);
  }

  /**
   * @brief getPFReference - retrieve a reference to this object - to be
   * overloaded by deriving classes so that the base class can be returned
   * @return reference to this object
   */
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
   * @param customRandInit - vector of doubles with the following form: <lvalue,
   * rvalue,
   * lvalue, rvalue, ...>
   * which will be used as the limits of the uniform distribution.
   * This vector should have a size equal to twice the number of dimensions
   * @param customPosInit - vector of doubles with the following form
   * <x,y,theta,x,y,theta,...> with size equal to nStatesPerRobot*nRobots
   */
  void init(const std::vector<double>& customRandInit,
            const std::vector<double>& customPosInit);

  /**
   * @brief predict - prediction step in the particle filter set with the
   * received odometry
   * @param robotNumber - the robot number [0,N]
   * @param odometry - a structure containing the latest odometry readings
   * @param time - a ros::Time structure with the timestamp for this data
   * @warning only for the omni dataset configuration
   */
  void predict(const uint robotNumber, const Odometry odom,
               const ros::Time stamp);

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
   * @param found - whether this landmark has been found
   */
  inline void saveLandmarkObservation(const uint robotNumber,
                                      const uint landmarkNumber,
                                      const bool found)
  {
    bufLandmarkObservations_[robotNumber][landmarkNumber].found = found;
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
   * @param found - whether the target has been found
   */
  inline void saveTargetObservation(const uint robotNumber, const bool found)
  {
    bufTargetObservations_[robotNumber].found = found;
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
    float robotHeight;

    /**
     * @brief PublishData - contains information necessary for the PFPublisher
     * class
     * @param nh - the node handle object
     * @param robotHeight - the fixed robot height
     */
    PublishData(float robotHeight) : robotHeight(robotHeight) {}
  };

  /**
   * @brief resize_particles - change to a different number of particles and
   * resize the publishing message
   * @param n - the desired number of particles
   */
  void resize_particles(const uint n)
  {
    // Call base class method
    ParticleFilter::resize_particles(n);

    ROS_INFO("Resizing particle message");

    // Resize particle message
    msg_particles_.particles.resize(n);
    for (uint p = 0; p < n; ++p)
    {
      msg_particles_.particles[p].particle.resize(nSubParticleSets_);
    }
  }

private:
  ros::Subscriber GT_sub_;
  ros::Publisher robotStatePublisher_, targetStatePublisher_,
      particlePublisher_, syncedGTPublisher_, targetEstimatePublisher_,
      targetGTPublisher_, targetParticlePublisher_;
  std::vector<ros::Publisher> particleStdPublishers_;
  std::vector<ros::Publisher> robotGTPublishers_;
  std::vector<ros::Publisher> robotEstimatePublishers_;
  ros::Publisher targetObservationsPublisher_;

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
  void publishTargetObservations();

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
