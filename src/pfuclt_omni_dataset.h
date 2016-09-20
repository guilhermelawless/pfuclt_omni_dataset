// STL libraries
#include <vector>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>

// ROS message definitions
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <read_omni_dataset/BallData.h>
#include <read_omni_dataset/LRMLandmarksData.h>
#include <read_omni_dataset/RobotState.h>
#include <read_omni_dataset/LRMGTData.h>
#include <pfuclt_omni_dataset/particle.h>
#include <pfuclt_omni_dataset/particles.h>

// Eigen libraries
#include <Eigen/Core>
#include <Eigen/Geometry>

// Boost libraries
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/random.hpp>
#include <boost/foreach.hpp>

// Auxiliary libraries
#include "pfuclt_aux.h"

namespace pfuclt
{
#define PI 3.14159

int MAX_ROBOTS;
int NUM_ROBOTS; // total number of playing robots in the team including self
int NUM_SENSORS_PER_ROBOT; // SENSORS include odometry, each feature sensor like
// a ball detector, each landmark-set detector and so
// on. In this case for example the number of sensors
// are 3, 1-odometry, 1-orange ball, 1-landmarkset.
// Usually this must co-incide with the number of
// topics to which each robot is publishing its
// sensed information.
int NUM_TARGETS; // Number of targets being tracked. In omni dataset, only one
// target exists for now: the orange ball. This may be improved
// in future by adding the blue ball which can be seen the raw
// footage of the dataset experiment
std::vector<bool> PLAYING_ROBOTS; // indicate which robot(s) is(are) playing

// Empirically obtained coefficients in the covariance expression. See (add
// publications here)

// coefficients for landmark observation covariance
float K1, K2;

// coefficients for target observation covariance
float K3, K4, K5;

float ROB_HT; // Fixed height of the robots above ground in meters
int MY_ID; // Use this flag to set the ID of the robot expected to run a certain
// decentralized algorithm. Robot with MY_ID will be trated as the
// self robot running the algorithm while the rest will be considered
// teammates. Note that in the dataset there are 4 robots with IDs
// 1,3,4 and 5. Robot with ID=2 is not present.

// Initial 2D positons of the robot as obtained from the overhead ground truth
// system. The order is OMNI1 OMNI2 OMNI3 OMNI4 and OMNI5. Notice OMNI2 is
// initialized as 0,0 because the robot is absent from the dataset.
// This initialization will work only if the node startes befoe the rosbag of
// the dataset. Obviously, the initialization below is for the initial positions
// at the begginning of the dataset. Otherwise, the initialization makes the
// odometry-only trajecory frame transformed to the origin.
std::vector<double> POS_INIT;

int nParticles_;

// for ease of access
std::vector<pfuclt_aux::Landmark> landmarks;

// This will be the generator use for randomizing
typedef boost::random::mt19937 RNGType;

// ReadRobotMessages needs a forward declaration of the robot class
class Robot;

typedef std::vector<std::vector<float> > particles_t;

/**
 * @brief The ReadRobotMessages class - Creates and keeps information on the
 * robot running the algorithm and its teammates. Is used as a middle-man
 * between all robots
 */
class ReadRobotMessages
{
private:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;
  std::vector<Robot*> robots_;

public:
  particles_t pfParticles;

  ReadRobotMessages();
  ~ReadRobotMessages();

  /**
   * @brief initializeFixedLandmarks - will get a filename from the parameter
   * server, and use its information to store landmark positions in the
   * landmarks vector
   */
  void initializeFixedLandmarks();

  /**
   * @brief areAllTeammatesActive - uses each robot's public methods to check if
   * they have started yet
   * @return true if every robot is active, false otherwise
   */
  bool areAllRobotsActive();
};

/**
 * @brief The Robot class - Has the common variables and methods of selfRobot
 * and TeammateRobot's
 */
class Robot
{
protected:
  ros::NodeHandle& nh_;
  ReadRobotMessages* parent_;
  bool started_;
  ros::Subscriber sOdom_, sBall_, sLandmark_;
  uint robotNumber_;
  Eigen::Isometry2d initPose_; // x y theta;
  Eigen::Isometry2d prevPose_;
  Eigen::Isometry2d curPose_;
  ros::Time curTime_;
  ros::Time prevTime_;
  particles_t& pfParticles_;

public:
  Robot(ros::NodeHandle& nh, ReadRobotMessages* parent,
        Eigen::Isometry2d initPose, particles_t& pfParticles, uint robotNumber)
      : nh_(nh), parent_(parent), initPose_(initPose), curPose_(initPose),
        pfParticles_(pfParticles), started_(false), robotNumber_(robotNumber)
  {
    // nothing, only initialize members above
  }
  bool isStarted() { return started_; }
  void setStarted(bool value) { started_ = value; }
};

/**
 * @brief The SelfRobot class - This is the class that performs the PF-UCLT
 * algorithm
 */
class SelfRobot : public Robot
{
private:
  ros::Subscriber GT_sub_;
  read_omni_dataset::LRMGTData receivedGTdata;
  pfuclt_omni_dataset::particles msg_particles;

  ros::Publisher State_publisher, targetStatePublisher, virtualGTPublisher,
      particlePublisher;
  read_omni_dataset::RobotState msg_state;

  RNGType seed_;
  std::vector<float> particleSet_[19];

  std::vector<float> normalizedWeights;

  bool particlesInitialized;

public:
  SelfRobot(ros::NodeHandle& nh, Eigen::Isometry2d initPose, particles_t& ptcls,
            ReadRobotMessages* caller, uint robotNumber);

  /// Use this method to implement perception algorithms
  void selfOdometryCallback(const nav_msgs::Odometry::ConstPtr&,
                            uint robotNumber);

  /// Use this method to implement perception algorithms
  void selfTargetDataCallback(const read_omni_dataset::BallData::ConstPtr&,
                              uint robotNumber);

  /// Use this method to implement perception algorithms
  void
  selfLandmarkDataCallback(const read_omni_dataset::LRMLandmarksData::ConstPtr&,
                           uint robotNumber);

  void gtDataCallback(const read_omni_dataset::LRMGTData::ConstPtr&);

  void initPFset();

  void PFpredict();

  void PFfuseRobotInfo();

  void PFfuseTargetInfo();

  void PFresample();

  // publish the estimated state of all the teammate robot
  void publishState(float, float, float);
};

/**
 * @brief The TeammateRobot class - This class doesn't perform the PF-UCLT
 * algorithm. Instead, it is used for callbacks and storing information to be
 * used later by the SelfRobot class
 */
class TeammateRobot : public Robot
{
public:
  TeammateRobot(ros::NodeHandle& nh, Eigen::Isometry2d initPose,
                particles_t& ptcls, ReadRobotMessages* caller,
                uint robotNumber);

  /// Use this method to implement perception algorithms
  void teammateOdometryCallback(const nav_msgs::Odometry::ConstPtr&,
                                uint robotNumber);

  /// Use this method to implement perception algorithms
  void teammateTargetDataCallback(const read_omni_dataset::BallData::ConstPtr&,
                                  uint robotNumber);

  /// Use this method to implement perception algorithms
  void teammateLandmarkDataCallback(
      const read_omni_dataset::LRMLandmarksData::ConstPtr&, uint robotNumber);
};

// end of namespace
}
