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

// Auxiliary libraries
#include "pfuclt_aux.h"
#include "particles.h"

namespace pfuclt
{
using namespace pfuclt_ptcls;
#define PI 3.14159
#define NUM_WEIGHT 1
#define STATES_PER_ROBOT 3

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

int N_PARTICLES;
int N_DIMENSIONS;

bool USE_CUSTOM_VALUES = false; // If set to true via the parameter server, the custom values will be used
std::vector<double> CUSTOM_PARTICLE_INIT; // Used to set custom values when initiating the particle filter set (will still be a uniform distribution)

// for ease of access
std::vector<pfuclt_aux::Landmark> landmarks;
ros::Time timeInit;

// RobotFactory needs a forward declaration of the robot class
class Robot;

/**
 * @brief The RobotFactory class - Creates and keeps information on the
 * robot running the algorithm and its teammates. Is used as a middle-man
 * between all robots
 */
class RobotFactory
{
private:
  ros::NodeHandle& nh_;
  std::vector<Robot*> robots_;

public:
  particle_filter pf;

  RobotFactory(ros::NodeHandle& nh);
  ~RobotFactory();

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
  RobotFactory* parent_;
  particle_filter& pf_;
  bool started_;
  ros::Time timeStarted_;
  ros::Subscriber sOdom_, sBall_, sLandmark_;
  uint robotNumber_;
  Eigen::Isometry2d initPose_; // x y theta;
  Eigen::Isometry2d prevPose_;
  Eigen::Isometry2d curPose_;
  ros::Time curTime_;
  ros::Time prevTime_;

  void startNow();

public:
  Robot(ros::NodeHandle& nh, RobotFactory* parent,
        Eigen::Isometry2d initPose, particle_filter& pf, uint robotNumber)
    : nh_(nh), parent_(parent), initPose_(initPose), curPose_(initPose),
      pf_(pf), started_(false), robotNumber_(robotNumber)
  {
    // nothing, only initialize members above
  }

  // This has to be public since the SelfRobot will call it
  void odometryCallback(const nav_msgs::Odometry::ConstPtr&);

  bool hasStarted() { return started_; }

  struct stateBuffer_s{
    Eigen::Isometry2d pose;
    Eigen::Isometry2d odometry;
  } stateBuffer;
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

  std::vector<float> particleSet_[19];

  std::vector<float> normalizedWeights;

  void tryInitializeParticles();

public:
  SelfRobot(ros::NodeHandle& nh, Eigen::Isometry2d initPose, particle_filter& ptcls,
            RobotFactory* caller, uint robotNumber);

  /// Use this method to implement perception algorithms
  void odometryCallback(const nav_msgs::Odometry::ConstPtr&);

  /// Use this method to implement perception algorithms
  void targetDataCallback(const read_omni_dataset::BallData::ConstPtr&);

  /// Use this method to implement perception algorithms
  void landmarkDataCallback(const read_omni_dataset::LRMLandmarksData::ConstPtr&);

  void gtDataCallback(const read_omni_dataset::LRMGTData::ConstPtr&);

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
                particle_filter& ptcls, RobotFactory* caller,
                uint robotNumber);

  /// Use this method to implement perception algorithms
  void OdometryCallback(const nav_msgs::Odometry::ConstPtr&);

  /// Use this method to implement perception algorithms
  void TargetDataCallback(const read_omni_dataset::BallData::ConstPtr&);

  /// Use this method to implement perception algorithms
  void LandmarkDataCallback(
      const read_omni_dataset::LRMLandmarksData::ConstPtr&);
};

// end of namespace
}
