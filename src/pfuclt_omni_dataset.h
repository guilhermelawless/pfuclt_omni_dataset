#include <cstdio>
#include <iostream>
#include <string>


#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <read_omni_dataset/BallData.h>
#include <read_omni_dataset/LRMLandmarksData.h>
#include <read_omni_dataset/RobotState.h>
#include <read_omni_dataset/LRMGTData.h>
#include <pfuclt_omni_dataset/particle.h>
#include <pfuclt_omni_dataset/particles.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "std_msgs/String.h"

//#include </opt/intel/ipp/include/ipp.h>

//GNU Scientific Library
/*
#include <gsl/gsl_rng.h>        //Random Number Generation
#include <gsl/gsl_randist.h>    //Random Number Distributions
#include <gsl/gsl_vector.h>     //GSL vectors
*/

//Boost libraries
#include <boost/random.hpp>
#include <boost/foreach.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

// #define M_PI        3.141592653589793238462643383280    /* pi */
#define PI 3.14159

///TODO These hard coded values must go out of this place. Preferably as the arguments of the main function and therefore in the launch file.

const std::size_t MAX_ROBOTS = 5;
const std::size_t NUM_ROBOTS = 1;// total number of playing robots in the team including self
const bool playingRobots[5] = {0,0,0,1,0}; // indicate which robot(s) is(are) playing
const std::size_t NUM_SENSORS_PER_ROBOs = 3;// SENSORS include odometry, each feature sensor like a ball detector, each landmark-set detector and so on. In this case for example the number of sensors are 3, 1-odometry, 1-orange ball, 1-landmarkset. Usually this must co-incide with the number of topics to which each robot is publishing its sensed information.

const std::size_t NUM_TARGETS = 1; // Number of targets being tracked. In omni dataset, only one target exists for now: the orange ball. This may be improved in future by adding the blue ball which can be seen the raw footage of the dataset experiment

//Below are empirically obtained coefficients in the covariance expression. See (add publications here)


//coefficients for landmark observation covariance
const std::size_t K1 = 2.0;
const std::size_t K2 = 0.5;

//coefficients for target observation covariance
const std::size_t K3 = 0.2;
const std::size_t K4 = 0.5;
const std::size_t K5 = 0.5; 
//const float K3 = 0.2, K4 = 0.5, K5 = 0.5; 

const std::size_t ROB_HT = 0.81; //(In this dataset) fixed height of the robots above ground in meter
const std::size_t MY_ID = 4; // Use this flag to set the ID of the robot expected to run a certain decentralized algorithm. Robot with MY_ID will be trated as the self robot running the algorithm while the rest will be considered teammates. Note that in the dataset there are 4 robots with IDs 1,3,4 and 5. Robot with ID=2 is not present.

//Initial 2D positons of the robot as obtained from the overhead ground truth system. The order is OMNI1 OMNI2 OMNI3 OMNI4 and OMNI5. Notice OMNI2 is initialized as 0,0 because the robot is absent from the dataset.
//This initialization will work only if the read_omni_dataset node startes befoe the rosbag of the dataset. Obviously, the initialization below is for the initial positions at the begginning of the dataset. Otherwise, the initialization makes the odometry-only trajecory frame transformed to the origin.
const double initArray[10] = {5.086676,-2.648978,0.0,0.0,1.688772,-2.095153,3.26839,-3.574936,4.058235,-0.127530};


static const int nParticles_ = 200;

using namespace ros;
using namespace std;
using namespace boost::accumulators;

vector< vector<float> > map_1;

typedef boost::random::mt19937 RNGType;

//Auxiliary functions
template <typename T>
T calc_stdDev(T vec);

template <typename T>
std::vector<size_t> order_index(std::vector<T> const& values);

class SelfRobot
{
  NodeHandle *nh;
  //One subscriber per sensor in the robot
  
  Subscriber sOdom_;
  Subscriber sBall_;
  Subscriber sLandmark_;
  Subscriber GT_sub_;
  read_omni_dataset::LRMGTData receivedGTdata;
  pfuclt_omni_dataset::particles pfucltPtcls;
  
  Eigen::Isometry2d initPose; // x y theta;
  Eigen::Isometry2d prevPose;  
  
  Publisher State_publisher, targetStatePublisher, virtualGTPublisher, particlePublisher;
  read_omni_dataset::RobotState msg;

  RNGType seed_;
  vector<float> particleSet_[19];
  //Ipp32f particleSet_[19][nParticles_]; // fields in rows are : OMNI4: X,Y,Theta,OMNI1: X,Y,Theta,OMNI3: X,Y,Theta,OMNI5: X,Y,Theta,Ball: X,Y,Z,ParticleWeight

  vector<float> normalizedWeights;
  /*
  Ipp32f normalizedWeightsSorted[nParticles_];
  Ipp32f normalizedCumWeightsSorted[nParticles_];
  int normalizedCumWeightsSortedIndex[nParticles_];
  Ipp32f ballWeights[nParticles_];  
  */
  
  vector< vector<float> >& pfParticlesSelf;
  
  
  
  bool *ifRobotIsStarted; 
  bool particlesInitialized;
  
  public:
    SelfRobot(NodeHandle *nh, int robotNumber, Eigen::Isometry2d _initPose, bool *_ifRobotIsStarted, vector< vector<float> >& _ptcls): initPose(_initPose), curPose(_initPose), ifRobotIsStarted(_ifRobotIsStarted), pfParticlesSelf(_ptcls), seed_(time(0))
    {
    
      sOdom_ = nh->subscribe<nav_msgs::Odometry>("/omni"+boost::lexical_cast<string>(robotNumber+1)+"/odometry", 10, boost::bind(&SelfRobot::selfOdometryCallback,this, _1,robotNumber+1));
      
      sBall_ = nh->subscribe<read_omni_dataset::BallData>("/omni"+boost::lexical_cast<string>(robotNumber+1)+"/orangeball3Dposition", 10, boost::bind(&SelfRobot::selfTargetDataCallback,this, _1,robotNumber+1));
      
      sLandmark_ = nh->subscribe<read_omni_dataset::LRMLandmarksData>("/omni"+boost::lexical_cast<string>(robotNumber+1)+"/landmarkspositions", 10, boost::bind(&SelfRobot::selfLandmarkDataCallback,this, _1,robotNumber+1));
      
      
      //The Graph Generator ans solver should also subscribe to the GT data and publish it... This is important for time synchronization
      GT_sub_ = nh->subscribe<read_omni_dataset::LRMGTData>("gtData_4robotExp", 10, boost::bind(&SelfRobot::gtDataCallback,this, _1));      
      //GT_sub_ = nh->subscribe("gtData_4robotExp", 1000, gtDataCallback); 
      
      ROS_INFO(" constructing SelfRobot <<object>> and called sensor subscribers for this robot %d",robotNumber+1);
      
      State_publisher = nh->advertise<read_omni_dataset::RobotState>("/pfuclt_omni_poses", 1000);
      
      targetStatePublisher = nh->advertise<read_omni_dataset::BallData>("/pfuclt_orangeBallState", 1000);
      
      virtualGTPublisher = nh->advertise<read_omni_dataset::LRMGTData>("/gtData_synced_pfuclt_estimate", 1000);
      
      particlePublisher = nh->advertise<pfuclt_omni_dataset::particles>("/pfuclt_particles",10);
      
      ifRobotIsStarted[robotNumber] = false;      
      particlesInitialized = false;

      for(int i=0; i<19; i++){
          particleSet_[i].resize(nParticles_);
      }
 
    }

    /// Use this method to implement perception algorithms
    void selfOdometryCallback(const nav_msgs::Odometry::ConstPtr&, int);
    
    /// Use this method to implement perception algorithms
    void selfTargetDataCallback(const read_omni_dataset::BallData::ConstPtr&, int);
     
    /// Use this method to implement perception algorithms
    void selfLandmarkDataCallback(const read_omni_dataset::LRMLandmarksData::ConstPtr&, int);
    
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
// private: 
    void publishState(float, float, float);
    
};


class TeammateRobot
{
  NodeHandle *nh;
  //One subscriber per sensor in the robot
  Subscriber sOdom_;
  Subscriber sBall_;
  Subscriber sLandmark_;

  Eigen::Isometry2d initPose; // x y theta;
  Eigen::Isometry2d prevPose;
  
  bool *ifRobotIsStarted;
  
  vector< vector<float> >& pfParticlesMate;
  
  public:
    TeammateRobot(NodeHandle *nh, int robotNumber, Eigen::Isometry2d _initPose, bool *_ifRobotIsStarted, vector< vector<float> >& _ptcls): initPose(_initPose), curPose(_initPose), ifRobotIsStarted(_ifRobotIsStarted), pfParticlesMate(_ptcls)
    {
    
      sOdom_ = nh->subscribe<nav_msgs::Odometry>("/omni"+boost::lexical_cast<string>(robotNumber+1)+"/odometry", 10, boost::bind(&TeammateRobot::teammateOdometryCallback,this, _1,robotNumber+1));
      
      sBall_ = nh->subscribe<read_omni_dataset::BallData>("/omni"+boost::lexical_cast<string>(robotNumber+1)+"/orangeball3Dposition", 10, boost::bind(&TeammateRobot::teammateTargetDataCallback,this, _1,robotNumber+1));
      
      sLandmark_ = nh->subscribe<read_omni_dataset::LRMLandmarksData>("/omni"+boost::lexical_cast<string>(robotNumber+1)+"/landmarkspositions", 10, boost::bind(&TeammateRobot::teammateLandmarkDataCallback,this, _1,robotNumber+1));
      
      ROS_INFO(" constructing TeammateRobot object and called sensor subscribers for robot %d",robotNumber+1);
      
      ifRobotIsStarted[robotNumber] = false;
      
    }

    /// Use this method to implement perception algorithms
    void teammateOdometryCallback(const nav_msgs::Odometry::ConstPtr&, int);
    
    /// Use this method to implement perception algorithms
    void teammateTargetDataCallback(const read_omni_dataset::BallData::ConstPtr&, int);
    
    /// Use this method to implement perception algorithms
    void teammateLandmarkDataCallback(const read_omni_dataset::LRMLandmarksData::ConstPtr&, int);
  

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
  
  bool robotStarted[MAX_ROBOTS]; // to indicaate whether a robot has started or not..

  vector< vector<float> > pfParticles;
 
  public:
    ReadRobotMessages(): loop_rate_(30)
    { 
      pfParticles.resize(nParticles_);
      for ( int i = 0 ; i < nParticles_ ; i++ )
	pfParticles[i].resize((MAX_ROBOTS+1)*3+1);
      
      Eigen::Isometry2d initialRobotPose;
      
      teammateRobots_.reserve(MAX_ROBOTS);
	
      for(int j=0;j<MAX_ROBOTS;j++)
      {
	robotStarted[j]=false;
      }
      
      for(int i=0;i<MAX_ROBOTS;i++)
      {
	if(playingRobots[i])
	{
	  initialRobotPose = Eigen::Rotation2Dd(-M_PI).toRotationMatrix();
	  initialRobotPose.translation() = Eigen::Vector2d(initArray[2*i+0],initArray[2*i+1]); 
	  
	  if(i+1 == MY_ID)
	  {
	    robot_ = new SelfRobot(&nh_,i, initialRobotPose,&robotStarted[0],pfParticles);
	  }
	  else
	  {	  
	    TeammateRobot *tempRobot = new TeammateRobot(&nh_,i, initialRobotPose,&robotStarted[0],pfParticles);
	    teammateRobots_.push_back(tempRobot);
	  }
	}
      }
      
      
    }
    
    void initializeFixedLandmarks(vector< vector<float> >&);
    
    
    
};






































