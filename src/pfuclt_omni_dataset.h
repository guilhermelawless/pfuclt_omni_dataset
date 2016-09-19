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

// #define M_PI        3.141592653589793238462643383280    /* pi */
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
std::vector<bool> playingRobots; // indicate which robot(s) is(are) playing

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
std::vector<double> initArray;

int nParticles_;

using namespace ros;
using namespace std;

vector<vector<float> > map_1;

typedef boost::random::mt19937 RNGType;

class SelfRobot
{
  NodeHandle* nh;
  // One subscriber per sensor in the robot

  Subscriber sOdom_;
  Subscriber sBall_;
  Subscriber sLandmark_;
  Subscriber GT_sub_;
  read_omni_dataset::LRMGTData receivedGTdata;
  pfuclt_omni_dataset::particles msg_particles;

  Eigen::Isometry2d initPose; // x y theta;
  Eigen::Isometry2d prevPose;

  Publisher State_publisher, targetStatePublisher, virtualGTPublisher,
      particlePublisher;
  read_omni_dataset::RobotState msg_state;

  RNGType seed_;
  vector<float> particleSet_[19];

  vector<float> normalizedWeights;
  /*
  Ipp32f normalizedWeightsSorted[nParticles_];
  Ipp32f normalizedCumWeightsSorted[nParticles_];
  int normalizedCumWeightsSortedIndex[nParticles_];
  Ipp32f ballWeights[nParticles_];
  */

  vector<vector<float> >& pfParticlesSelf;

  bool particlesInitialized;

private:
  vector<bool>* ifRobotIsStarted_;

public:
  SelfRobot(NodeHandle* nh, int robotNumber, Eigen::Isometry2d _initPose,
            vector<bool>* _robotStarted, vector<vector<float> >& _ptcls)
      : initPose(_initPose), curPose(_initPose),
        ifRobotIsStarted_(_robotStarted), pfParticlesSelf(_ptcls),
        seed_(time(0))
  {

    // Subscribe to topics
    sOdom_ = nh->subscribe<nav_msgs::Odometry>(
        "/omni" + boost::lexical_cast<string>(robotNumber + 1) + "/odometry",
        10, boost::bind(&SelfRobot::selfOdometryCallback, this, _1,
                        robotNumber + 1));

    sBall_ = nh->subscribe<read_omni_dataset::BallData>(
        "/omni" + boost::lexical_cast<string>(robotNumber + 1) +
            "/orangeball3Dposition",
        10, boost::bind(&SelfRobot::selfTargetDataCallback, this, _1,
                        robotNumber + 1));

    sLandmark_ = nh->subscribe<read_omni_dataset::LRMLandmarksData>(
        "/omni" + boost::lexical_cast<string>(robotNumber + 1) +
            "/landmarkspositions",
        10, boost::bind(&SelfRobot::selfLandmarkDataCallback, this, _1,
                        robotNumber + 1));

    // The Graph Generator ans solver should also subscribe to the GT data and
    // publish it... This is important for time synchronization
    GT_sub_ = nh->subscribe<read_omni_dataset::LRMGTData>(
        "gtData_4robotExp", 10,
        boost::bind(&SelfRobot::gtDataCallback, this, _1));
    // GT_sub_ = nh->subscribe("gtData_4robotExp", 1000, gtDataCallback);

    ROS_INFO(" constructing SelfRobot <<object>> and called sensor subscribers "
             "for this robot %d",
             robotNumber + 1);

    // Advertise some topics; don't change the names, preferably remap in a
    // launch file
    State_publisher = nh->advertise<read_omni_dataset::RobotState>(
        "/pfuclt_omni_poses", 1000);

    targetStatePublisher = nh->advertise<read_omni_dataset::BallData>(
        "/pfuclt_orangeBallState", 1000);

    virtualGTPublisher = nh->advertise<read_omni_dataset::LRMGTData>(
        "/gtData_synced_pfuclt_estimate", 1000);

    particlePublisher =
        nh->advertise<pfuclt_omni_dataset::particles>("/pfuclt_particles", 10);

    // Mark initialized flags as false
    ifRobotIsStarted_->at(robotNumber) = false;
    particlesInitialized = false;

    // Resize particle set to allow all subparticles
    for (int i = 0; i < 19; i++)
    {
      particleSet_[i].resize(nParticles_);
    }
  }

  /// Use this method to implement perception algorithms
  void selfOdometryCallback(const nav_msgs::Odometry::ConstPtr&, int);

  /// Use this method to implement perception algorithms
  void selfTargetDataCallback(const read_omni_dataset::BallData::ConstPtr&,
                              int);

  /// Use this method to implement perception algorithms
  void
  selfLandmarkDataCallback(const read_omni_dataset::LRMLandmarksData::ConstPtr&,
                           int);

  void gtDataCallback(const read_omni_dataset::LRMGTData::ConstPtr&);

  void initPFset();

  void PFpredict();

  void PFfuseRobotInfo();

  void PFfuseTargetInfo();

  void PFresample();

  Eigen::Isometry2d curPose;
  Time curTime;
  Time prevTime;

  // publish the estimated state of all the teammate robot
  void publishState(float, float, float);
};

class TeammateRobot
{
  NodeHandle* nh;
  // One subscriber per sensor in the robot
  Subscriber sOdom_;
  Subscriber sBall_;
  Subscriber sLandmark_;

  Eigen::Isometry2d initPose; // x y theta;
  Eigen::Isometry2d prevPose;

  vector<vector<float> >& pfParticlesMate;

private:
  vector<bool>* ifRobotIsStarted;

public:
  TeammateRobot(NodeHandle* nh, int robotNumber, Eigen::Isometry2d _initPose,
                vector<bool>* _robotStarted, vector<vector<float> >& _ptcls)
      : initPose(_initPose), curPose(_initPose),
        ifRobotIsStarted(_robotStarted), pfParticlesMate(_ptcls)
  {

    sOdom_ = nh->subscribe<nav_msgs::Odometry>(
        "/omni" + boost::lexical_cast<string>(robotNumber + 1) + "/odometry",
        10, boost::bind(&TeammateRobot::teammateOdometryCallback, this, _1,
                        robotNumber + 1));

    sBall_ = nh->subscribe<read_omni_dataset::BallData>(
        "/omni" + boost::lexical_cast<string>(robotNumber + 1) +
            "/orangeball3Dposition",
        10, boost::bind(&TeammateRobot::teammateTargetDataCallback, this, _1,
                        robotNumber + 1));

    sLandmark_ = nh->subscribe<read_omni_dataset::LRMLandmarksData>(
        "/omni" + boost::lexical_cast<string>(robotNumber + 1) +
            "/landmarkspositions",
        10, boost::bind(&TeammateRobot::teammateLandmarkDataCallback, this, _1,
                        robotNumber + 1));

    ROS_INFO(" constructing TeammateRobot object and called sensor subscribers "
             "for robot %d",
             robotNumber + 1);

    ifRobotIsStarted->at(robotNumber) = false;
  }

  /// Use this method to implement perception algorithms
  void teammateOdometryCallback(const nav_msgs::Odometry::ConstPtr&, int);

  /// Use this method to implement perception algorithms
  void teammateTargetDataCallback(const read_omni_dataset::BallData::ConstPtr&,
                                  int);

  /// Use this method to implement perception algorithms
  void teammateLandmarkDataCallback(
      const read_omni_dataset::LRMLandmarksData::ConstPtr&, int);

  Eigen::Isometry2d curPose;
  Time curTime;
  Time prevTime;
};

class ReadRobotMessages
{
  NodeHandle nh_;
  Rate loop_rate_;

  SelfRobot* robot_;
  vector<TeammateRobot*> teammateRobots_;

  vector<bool> robotStarted; // to indicate whether a robot has started or not..

  vector<vector<float> > pfParticles;

public:
  ReadRobotMessages() : loop_rate_(30)
  {
    // Read parameters from param server
    using pfuclt_aux::readParam;
    readParam<int>(&nh_, "/MAX_ROBOTS", &MAX_ROBOTS);
    readParam<int>(&nh_, "/NUM_ROBOTS", &NUM_ROBOTS);
    readParam<float>(&nh_, "/ROB_HT", &ROB_HT);
    readParam<int>(&nh_, "/MY_ID", &MY_ID);
    readParam<int>(&nh_, "/N_PARTICLES", &nParticles_);
    readParam<int>(&nh_, "/NUM_SENSORS_PER_ROBOT", &NUM_SENSORS_PER_ROBOT);
    readParam<int>(&nh_, "/NUM_TARGETS", &NUM_TARGETS);
    readParam<float>(&nh_, "/LANDMARK_COV/K1", &K1);
    readParam<float>(&nh_, "/LANDMARK_COV/K2", &K2);
    readParam<float>(&nh_, "/LANDMARK_COV/K3", &K3);
    readParam<float>(&nh_, "/LANDMARK_COV/K4", &K4);
    readParam<float>(&nh_, "/LANDMARK_COV/K5", &K5);

    if (nh_.getParam("/playingRobots", playingRobots))
    {
      ostringstream oss;
      oss << "Received parameter /playingRobots=[ ";
      for (std::vector<bool>::iterator it = playingRobots.begin();
           it != playingRobots.end(); ++it)
        oss << std::boolalpha << *it << " ";
      oss << "]";

      ROS_INFO("%s", oss.str().c_str());
    }
    else
      ROS_ERROR("Failed to receive parameter /playingRobots");

    if (nh_.getParam("/POS_INIT", initArray))
    {
      ostringstream oss;
      oss << "Received parameter /POS_INIT=[ ";
      for (std::vector<double>::iterator it = initArray.begin();
           it != initArray.end(); ++it)
        oss << *it << " ";
      oss << "]";

      ROS_INFO("%s", oss.str().c_str());
    }
    else
      ROS_ERROR("Failed to receive parameter /POS_INIT");

    pfParticles.resize(nParticles_);
    for (int i = 0; i < nParticles_; i++)
      pfParticles[i].resize((MAX_ROBOTS + 1) * 3 + 1);

    Eigen::Isometry2d initialRobotPose;

    robotStarted.resize(MAX_ROBOTS, false);

    for (int i = 0; i < MAX_ROBOTS; i++)
    {
      if (playingRobots[i])
      {
        initialRobotPose = Eigen::Rotation2Dd(-M_PI).toRotationMatrix();
        initialRobotPose.translation() =
            Eigen::Vector2d(initArray[2 * i + 0], initArray[2 * i + 1]);

        if (i + 1 == MY_ID)
        {
          robot_ = new SelfRobot(&nh_, i, initialRobotPose, &robotStarted,
                                 pfParticles);
        }
        else
        {
          TeammateRobot* tempRobot = new TeammateRobot(
              &nh_, i, initialRobotPose, &robotStarted, pfParticles);
          teammateRobots_.push_back(tempRobot);
        }
      }
    }
  }

  void initializeFixedLandmarks(vector<vector<float> >&);
};
