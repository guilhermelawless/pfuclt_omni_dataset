// STL libraries
#include <vector>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>

// ROS message definitions
#include <ros/ros.h>
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
#include <boost/shared_ptr.hpp>

// Auxiliary libraries
#include "pfuclt_aux.h"
#include "particles.h"

namespace pfuclt
{
using namespace pfuclt_ptcls;
#define PI 3.14159
#define NUM_WEIGHT 1
#define STATES_PER_ROBOT 3
#define HEURISTICS_THRESH_DEFAULT                                              \
  {                                                                            \
    2.5, 2.5, 2.5, 2.5, FLT_MAX, FLT_MAX, 3.5, 3.5, FLT_MAX, FLT_MAX           \
  }

// workaround for a scoped enum
struct RobotType
{
  enum RobotType_e
  {
    Self,
    Teammate
  };
};

// Forward declaration of classes
class Robot;
class SelfRobot;

// useful typedefs
typedef boost::shared_ptr<Robot> Robot_ptr;
typedef boost::shared_ptr<SelfRobot> SelfRobot_ptr;

/**
 * @brief The RobotFactory class - Creates and keeps information on the
 * robot running the algorithm and its teammates. Is used as a middle-man
 * between all robots
 */
class RobotFactory
{

private:
  ros::NodeHandle& nh_;
  std::vector<Robot_ptr> robots_;

  /**
   * @brief areAllTeammatesActive - uses each robot's public methods to check if
   * they have started yet
   * @return true if every robot is active, false otherwise
   */
  bool areAllRobotsActive();

public:
  ParticleFilter pf;

  RobotFactory(ros::NodeHandle& nh);

  /**
   * @brief tryInitializeParticles - checks if every robot is started, and if
   * so, will initiate the particle filter
   */
  void tryInitializeParticles();

  /**
   * @brief initializeFixedLandmarks - will get a filename from the parameter
   * server, and use its information to store landmark positions in the
   * landmarks vector
   */
  void initializeFixedLandmarks();
};

/**
 * @brief The Robot class - Has the common variables and methods of all robots,
 * and is the base class of any specialized robots who may derive from it.
 * Teammates should be instances of this class
 */
class Robot
{
protected:
  ros::NodeHandle& nh_;
  RobotFactory* parent_;
  ParticleFilter& pf_;
  bool started_;
  ros::Time timeStarted_;
  ros::Subscriber sOdom_, sBall_, sLandmark_;
  uint robotNumber_;
  Eigen::Isometry2d initPose_; // x y theta;
  Eigen::Isometry2d prevPose_;
  Eigen::Isometry2d curPose_;
  ros::Time curTime_;
  ros::Time prevTime_;

  /**
   * @brief startNow - starts the robot
   */
  void startNow();

public:
  /**
   * @brief Robot - constructor, creates a new Robot instance
   * @param nh - reference to the node handler object
   * @param parent - reference to the robot factory creating this object
   * @param initPose - initial (x,y) pose of the robot
   * @param pf - reference to the particle filter to be used for this robot
   * @param robotNumber - the assigned number in the team
   * @param robotType - enumeration: is the robot a teammate or the self robot?
   */
  Robot(ros::NodeHandle& nh, RobotFactory* parent, Eigen::Isometry2d initPose,
        ParticleFilter& pf, uint robotNumber,
        RobotType::RobotType_e robotType = RobotType::Teammate);

  /**
   * @brief odometryCallback - event-driven function which should be called when
   * new odometry data is received
   * @param odometry - the odometry data received, using the standard ROS
   * odometry message type
   */
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry);

  /**
   * @brief targetCallBack - event-driven function which should be called when
   * new target data is received
   * @param target - the target data received, using custom msg types defined in
   * the read_omni_dataset package
   */
  void targetCallback(const read_omni_dataset::BallData::ConstPtr& target);

  /**
   * @brief landmarkDataCallback - event-driven function which should be called
   * when new landmark data is received
   * @param landmarkData - the landmark data received, using custom msg types
   * defined in the read_omni_dataset package
   */
  void landmarkDataCallback(
      const read_omni_dataset::LRMLandmarksData::ConstPtr& landmarkData);

  /**
   * @brief hasStarted
   * @return
   */
  bool hasStarted() { return started_; }
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

public:
  SelfRobot(ros::NodeHandle& nh, RobotFactory* caller,
            Eigen::Isometry2d initPose, ParticleFilter& ptcls,
            uint robotNumber);

  /**
   * @brief odometryCallback - event-driven function which should be called when
   * new odometry data is received
   * @param odometry - the odometry data received, using the standard ROS
   * odometry message type
   * @remark calls the Robot::odometryCallback method and extends it
   */
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry);

  void gtDataCallback(const read_omni_dataset::LRMGTData::ConstPtr&);

  void PFfuseRobotInfo();

  void PFfuseTargetInfo();

  void PFresample();

  // publish the estimated state of all the teammate robot
  void publishState(float, float, float);
};

// end of namespace
}
