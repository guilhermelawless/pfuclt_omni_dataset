#include "pfuclt_aux.h"
#include "particles.h"
#include "pfuclt_omni_dataset.h"

#define ROS_TDIFF(t) (t.toSec() - timeInit.toSec())

namespace pfuclt
{

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
int NUM_LANDMARKS = 10;
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
std::vector<double> POS_INIT;

int N_PARTICLES;
int N_DIMENSIONS;

bool USE_CUSTOM_VALUES = false; // If set to true via the parameter server, the
// custom values will be used
std::vector<double> CUSTOM_PARTICLE_INIT; // Used to set custom values when
// initiating the particle filter set (will still be a uniform distribution)

std::vector<float> CUSTOM_RANDOM_ALPHA; // Used to set custom values for the
// sampling models in the particle
// filter

// for ease of access
std::vector<pfuclt_aux::Landmark> landmarks;
ros::Time timeInit;

// Method definitions

RobotFactory::RobotFactory(ros::NodeHandle& nh)
    : nh_(nh),
      pf(pfuclt_ptcls::ParticleFilter(
          N_PARTICLES, NUM_TARGETS, STATES_PER_ROBOT, MAX_ROBOTS, NUM_LANDMARKS,
          PLAYING_ROBOTS, landmarks, CUSTOM_RANDOM_ALPHA))
{
  for (uint rn = 0; rn < MAX_ROBOTS; rn++)
  {
    if (PLAYING_ROBOTS[rn])
    {
      Eigen::Isometry2d initialRobotPose(
          Eigen::Rotation2Dd(-M_PI).toRotationMatrix());
      initialRobotPose.translation() =
          Eigen::Vector2d(POS_INIT[2 * rn + 0], POS_INIT[2 * rn + 1]);

      timeInit = ros::Time::now();
      ROS_INFO("Init time set to %f", timeInit.toSec());

      if (rn + 1 == MY_ID)
      {
        robots_.push_back(
            SelfRobot_ptr(new SelfRobot(nh_, this, initialRobotPose, pf, rn)));
      }
      else
      {
        robots_.push_back(
            Robot_ptr(new Robot(nh_, this, initialRobotPose, pf, rn)));
      }
    }
  }
}

void RobotFactory::tryInitializeParticles()
{
  if (!areAllRobotsActive())
    return;

  if (USE_CUSTOM_VALUES)
    pf.init(CUSTOM_PARTICLE_INIT);
  else
    pf.init();
}

void RobotFactory::initializeFixedLandmarks()
{
  std::string filename;
  using namespace pfuclt_aux;

  // get the filename from parameter server
  if (!readParam<std::string>(nh_, "/LANDMARKS_CONFIG", filename))
    return;

  // parse the file and copy to vector of Landmarks
  landmarks = getLandmarks(filename.c_str());
  ROS_ERROR_COND(landmarks.empty(), "Couldn't open file \"%s\"",
                 filename.c_str());

  ROS_ERROR_COND(landmarks.size() != NUM_LANDMARKS,
                 "Read a number of landmarks different from the specified in "
                 "NUM_LANDMARKS");

  // iterate over the vector and print information
  for (std::vector<Landmark>::iterator it = landmarks.begin();
       it != landmarks.end(); ++it)
  {
    ROS_INFO("A fixed landmark with ID %d at position {x=%.2f, y=%.2f} \twas "
             "created",
             it->serial, it->x, it->y);
  }
}

bool RobotFactory::areAllRobotsActive()
{
  for (std::vector<Robot_ptr>::iterator it = robots_.begin();
       it != robots_.end(); ++it)
  {
    if (false == (*it)->hasStarted())
      return false;
  }
  return true;
}

void Robot::startNow()
{
  timeStarted_ = ros::Time::now();
  started_ = true;
  ROS_INFO("OMNI%d has started %.2fs after the initial time", robotNumber_ + 1,
           ROS_TDIFF(timeStarted_));
}

Robot::Robot(ros::NodeHandle& nh, RobotFactory* parent,
             Eigen::Isometry2d initPose, ParticleFilter& pf, uint robotNumber,
             RobotType::RobotType_e robotType)
    : nh_(nh), parent_(parent), initPose_(initPose), curPose_(initPose),
      pf_(pf), started_(false), robotNumber_(robotNumber)
{
  std::string robotNamespace("/omni" +
                             boost::lexical_cast<std::string>(robotNumber + 1));

  // Subscribe to topics
  sOdom_ = nh.subscribe<nav_msgs::Odometry>(
      robotNamespace + "/odometry", 10,
      boost::bind(&Robot::odometryCallback, this, _1));

  sBall_ = nh.subscribe<read_omni_dataset::BallData>(
      robotNamespace + "/orangeball3Dposition", 10,
      boost::bind(&Robot::targetCallback, this, _1));

  sLandmark_ = nh.subscribe<read_omni_dataset::LRMLandmarksData>(
      robotNamespace + "/landmarkspositions", 10,
      boost::bind(&Robot::landmarkDataCallback, this, _1));

  if (robotType == RobotType::Teammate)
    ROS_INFO("Created teammate robot OMNI%d", robotNumber + 1);
}

void Robot::odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry)
{
  if (!started_)
    startNow();

  if (!pf_.isInitialized())
    parent_->tryInitializeParticles();

  pfuclt_ptcls::Odometry odomStruct = {
    odometry->pose.pose.position.x, odometry->pose.pose.position.y,
    tf::getYaw(odometry->pose.pose.orientation)
  };

  ROS_DEBUG("OMNI%d odometry at time %d", robotNumber_ + 1,
            odometry->header.stamp.sec);

  // Call the particle filter predict step for this robot
  pf_.predict(robotNumber_, odomStruct);
}

void Robot::targetCallback(const read_omni_dataset::BallData::ConstPtr& target)
{
  if (!started_)
    startNow();

  if (target->found)
  {
    ROS_DEBUG("OMNI%d ball data at time %d", robotNumber_ + 1,
              target->header.stamp.sec);

    Eigen::Vector2d ballObsVec = Eigen::Vector2d(target->x, target->y);
    pfuclt_ptcls::TargetObservation obs;

    obs.found = true;
    obs.x = target->x;
    obs.y = target->y;
    obs.z = target->z;
    obs.d = ballObsVec.norm();
    obs.phi = atan2(target->y, target->x);

    obs.covDD = (double)(1 / target->mismatchFactor) *
                (K3 * obs.d + K4 * (obs.d * obs.d));

    obs.covPP = K5 * (1 / (obs.d + 1));

    obs.covXX = pow(cos(obs.phi), 2) * obs.covDD +
                pow(sin(obs.phi), 2) *
                    (pow(obs.d, 2) * obs.covPP + obs.covDD * obs.covPP);
    obs.covYY = pow(sin(obs.phi), 2) * obs.covDD +
                pow(cos(obs.phi), 2) *
                    (pow(obs.d, 2) * obs.covPP + obs.covDD * obs.covPP);

    // Save this observation
    pf_.saveTargetObservation(robotNumber_, obs);
  }
  else
  {
    ROS_DEBUG("OMNI%d didn't find the ball at time %d", robotNumber_ + 1,
              target->header.stamp.sec);

    pf_.saveTargetObservation(robotNumber_, false);
  }

  pf_.saveAllTargetMeasurementsDone(robotNumber_);
}

void Robot::landmarkDataCallback(
    const read_omni_dataset::LRMLandmarksData::ConstPtr& landmarkData)
{
  ROS_DEBUG("OMNI%d landmark data at time %d", robotNumber_ + 1,
            landmarkData->header.stamp.sec);

  bool heuristicsFound[NUM_LANDMARKS];
  for (int i = 0; i < NUM_LANDMARKS; i++)
    heuristicsFound[i] = landmarkData->found[i];

  float distances[NUM_LANDMARKS];

  // d = sqrt(x^2+y^2)
  for (int i = 0; i < NUM_LANDMARKS; i++)
  {
    distances[i] =
        pow((pow(landmarkData->x[i], 2) + pow(landmarkData->y[i], 2)), 0.5);
  }

  // Define heuristics if using custom values
  if (USE_CUSTOM_VALUES)
  {
    // Huristic 1. If I see only 8 and not 9.... then I cannot see 7
    if (landmarkData->found[8] && !landmarkData->found[9])
      heuristicsFound[7] = false;

    // Huristic 2. If I see only 9 and not 8.... then I cannot see 6
    if (!landmarkData->found[8] && landmarkData->found[9])
      heuristicsFound[6] = false;

    // Huristic 3. If I see both 8 and 9.... then there are 2 subcases
    if (landmarkData->found[8] && landmarkData->found[9])
    {
      // Huristic 3.1. If I am closer to 9.... then I cannot see 6
      if (distances[9] < distances[8])
        heuristicsFound[6] = false;

      // Huristic 3.2 If I am closer to 8.... then I cannot see 7
      if (distances[8] < distances[9])
        heuristicsFound[7] = false;
    }

    float heuristicsThresh[] = HEURISTICS_THRESH_DEFAULT;

    if (robotNumber_ == 4)
    {
      heuristicsThresh[4] = 3.0;
      heuristicsThresh[5] = 3.0;
      heuristicsThresh[8] = 3.0;
      heuristicsThresh[9] = 3.0;
    }

    if (robotNumber_ == 3)
    {
      heuristicsThresh[4] = 6.5;
      heuristicsThresh[5] = 6.5;
      heuristicsThresh[8] = 6.5;
      heuristicsThresh[9] = 6.5;
    }

    if (robotNumber_ == 1)
    {
      heuristicsThresh[4] = 6.5;
      heuristicsThresh[5] = 6.5;
      heuristicsThresh[8] = 6.5;
      heuristicsThresh[9] = 6.5;
    }

    if (robotNumber_ == 5)
    {
      heuristicsThresh[4] = 3.5;
      heuristicsThresh[5] = 3.5;
      heuristicsThresh[8] = 3.5;
      heuristicsThresh[9] = 3.5;
    }

    // Set landmark as not found if distance to it is above a certain threshold
    for (int i = 0; i < NUM_LANDMARKS; i++)
    {
      if (distances[i] > heuristicsThresh[i])
        heuristicsFound[i] = false;
    }
  }

  // End of heuristics, below uses the array but just for convenience

  for (int i = 0; i < NUM_LANDMARKS; i++)
  {

    if (false == heuristicsFound[i])
      pf_.saveLandmarkObservation(robotNumber_, i, false);

    else
    {
      // TODO in the no-ROS version the y frame is inverted. Not here? Check
      // later

      pfuclt_ptcls::LandmarkObservation obs;
      obs.found = true;
      obs.x = landmarkData->x[i];
      obs.y = landmarkData->y[i];
      obs.d = sqrt(obs.x * obs.x + obs.y * obs.y);
      obs.phi = atan2(obs.y, obs.x);
      obs.covDD =
          (K1 * fabs(1.0 - (landmarkData->AreaLandMarkActualinPixels[i] /
                            landmarkData->AreaLandMarkExpectedinPixels[i]))) *
          (obs.d * obs.d);
      obs.covPP = NUM_LANDMARKS * K2 * (1 / (obs.d + 1));
      obs.covXX = pow(cos(obs.phi), 2) * obs.covDD +
                  pow(sin(obs.phi), 2) *
                      (pow(obs.d, 2) * obs.covPP + obs.covDD * obs.covPP);
      obs.covYY = pow(sin(obs.phi), 2) * obs.covDD +
                  pow(cos(obs.phi), 2) *
                      (pow(obs.d, 2) * obs.covPP + obs.covDD * obs.covPP);

      pf_.saveLandmarkObservation(robotNumber_, i, obs);
    }
  }

  pf_.saveAllLandmarkMeasurementsDone(robotNumber_);
}

SelfRobot::SelfRobot(ros::NodeHandle& nh, RobotFactory* caller,
                     Eigen::Isometry2d initPose, ParticleFilter& ptcls,
                     uint robotNumber)
    : Robot(nh, caller, initPose, ptcls, robotNumber, RobotType::Self)
{
  // Prepare particle message
  msg_particles.particles.resize(N_PARTICLES);
  for (uint part = 0; part < N_PARTICLES; ++part)
  {
    msg_particles.particles[part].particle.resize(ptcls.size());
  }

  // The Graph Generator ans solver should also subscribe to the GT data and
  // publish it... This is important for time synchronization
  GT_sub_ = nh.subscribe<read_omni_dataset::LRMGTData>(
      "gtData_4robotExp", 10,
      boost::bind(&SelfRobot::gtDataCallback, this, _1));
  // GT_sub_ = nh->subscribe("gtData_4robotExp", 1000, gtDataCallback);

  // Advertise some topics; don't change the names, preferably remap in a
  // launch file
  State_publisher =
      nh.advertise<read_omni_dataset::RobotState>("/pfuclt_omni_poses", 1000);

  targetStatePublisher = nh.advertise<read_omni_dataset::BallData>(
      "/pfuclt_orangeBallState", 1000);

  virtualGTPublisher = nh.advertise<read_omni_dataset::LRMGTData>(
      "/gtData_synced_pfuclt_estimate", 1000);

  particlePublisher =
      nh.advertise<pfuclt_omni_dataset::particles>("/pfuclt_particles", 10);

  ROS_INFO("Created main robot OMNI%d", robotNumber + 1);
}

void SelfRobot::gtDataCallback(
    const read_omni_dataset::LRMGTData::ConstPtr& gtMsgReceived)
{
  receivedGTdata = *gtMsgReceived;
}

void SelfRobot::publishState(float x, float y, float theta)
{
  // using the first robot for the globaltime stamp of this message
  // printf("we are here \n\n\n");

  // first publish the particles!!!!
  particlePublisher.publish(msg_particles);

  msg_state.header.stamp =
      ros::Time::now(); // time of the self-robot must be in the full state
  receivedGTdata.header.stamp = ros::Time::now();

  msg_state.robotPose[MY_ID - 1].pose.pose.position.x = x;
  msg_state.robotPose[MY_ID - 1].pose.pose.position.y = y;
  msg_state.robotPose[MY_ID - 1].pose.pose.position.z =
      ROB_HT; // fixed height aboveground

  msg_state.robotPose[MY_ID - 1].pose.pose.orientation.x = 0;
  msg_state.robotPose[MY_ID - 1].pose.pose.orientation.y = 0;
  msg_state.robotPose[MY_ID - 1].pose.pose.orientation.z = sin(theta / 2);
  msg_state.robotPose[MY_ID - 1].pose.pose.orientation.w = cos(theta / 2);

  State_publisher.publish(msg_state);
  virtualGTPublisher.publish(receivedGTdata);
}

// end of namespace
}

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "pfuclt_omni_dataset");
  ros::NodeHandle nh;

  // Set debug if asked
  if (argc > 2)
  {
    if (!strcmp(argv[2], "true"))
    {
      if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                         ros::console::levels::Debug))
      {
        ros::console::notifyLoggerLevelsChanged();
      }
    }

    ROS_DEBUG("DEBUG mode set");
  }

  // read parameters from param server
  using pfuclt_aux::readParam;
  using namespace pfuclt;

  readParam<int>(nh, "/MAX_ROBOTS", MAX_ROBOTS);
  readParam<float>(nh, "/ROB_HT", ROB_HT);
  readParam<int>(nh, "/MY_ID", MY_ID);
  readParam<int>(nh, "/N_PARTICLES", N_PARTICLES);
  readParam<int>(nh, "/NUM_SENSORS_PER_ROBOT", NUM_SENSORS_PER_ROBOT);
  readParam<int>(nh, "/NUM_TARGETS", NUM_TARGETS);
  readParam<int>(nh, "/NUM_LANDMARKS", NUM_LANDMARKS);
  readParam<float>(nh, "/LANDMARK_COV/K1", K1);
  readParam<float>(nh, "/LANDMARK_COV/K2", K2);
  readParam<float>(nh, "/LANDMARK_COV/K3", K3);
  readParam<float>(nh, "/LANDMARK_COV/K4", K4);
  readParam<float>(nh, "/LANDMARK_COV/K5", K5);
  readParam<bool>(nh, "/PLAYING_ROBOTS", PLAYING_ROBOTS);
  readParam<double>(nh, "/POS_INIT", POS_INIT);
  readParam<float>(nh, "/CUSTOM_RANDOM_ALPHA", CUSTOM_RANDOM_ALPHA);
  readParam<bool>(nh, "/USE_CUSTOM_VALUES", USE_CUSTOM_VALUES);

  uint total_size = (MAX_ROBOTS + NUM_TARGETS) * STATES_PER_ROBOT;

  if (USE_CUSTOM_VALUES)
  {
    readParam<double>(nh, "/CUSTOM_PARTICLE_INIT", CUSTOM_PARTICLE_INIT);
    if (CUSTOM_PARTICLE_INIT.size() != (total_size * 2))
    {
      ROS_ERROR("/CUSTOM_PARTICLE_INIT given but not of correct size - should "
                "have %d numbers and has %d",
                total_size * 2, (int)CUSTOM_PARTICLE_INIT.size());
    }
  }

  if (N_PARTICLES < 0 || total_size < 0)
  {
    ROS_ERROR("Unacceptable configuration for particles class");
    nh.shutdown();
    return 0;
  }

  pfuclt::RobotFactory Factory(nh);

  if (pfuclt::MY_ID == 2)
  {
    // ROS_WARN("OMNI2 not present in dataset. Please try with another Robot ID
    // for self robot");
    return 0;
  }

  Factory.initializeFixedLandmarks();

  ros::spin();
  return 0;
}
