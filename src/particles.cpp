#include "particles.h"
#include "ros/ros.h"
#include <stdlib.h>
#include <boost/random.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>
#include <angles/angles.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>

#define DONT_FUSE_TARGET true
#define BROADCAST_TF_AND_POSES true
#define PUBLISH_PTCLS true

namespace pfuclt_ptcls
{

ParticleFilter::ParticleFilter(struct PFinitData& data)
    : nParticles_(data.nParticles), nTargets_(data.nTargets),
      nStatesPerRobot_(data.statesPerRobot), nRobots_(data.nRobots),
      nSubParticleSets_(data.nTargets * STATES_PER_TARGET +
                        data.nRobots * data.statesPerRobot + 1),
      nLandmarks_(data.nLandmarks), alpha_(data.alpha),
      robotsUsed_(data.robotsUsed), landmarksMap_(data.landmarksMap),
      iteration_oss(new std::ostringstream("")),
      particles_(nSubParticleSets_, subparticles_t(data.nParticles)),
      seed_(time(0)), initialized_(false),
      bufLandmarkObservations_(
          data.nRobots, std::vector<LandmarkObservation>(data.nLandmarks)),
      bufTargetObservations_(data.nRobots), prevTime_(ros::Time::now()),
      newTime_(ros::Time::now()), iterationTime_(ITERATION_TIME_DEFAULT),
      weightComponents_(data.nRobots, subparticles_t(data.nParticles, 0.0)),
      state_(data.statesPerRobot, data.nRobots, data.robotsUsed)
{

  ROS_INFO("Created particle filter with dimensions %d, %d",
           (int)particles_.size(), (int)particles_[0].size());
}

void ParticleFilter::predictTarget(uint robotNumber)
{
  if (!initialized_)
    return;

  state_.targetPredicted = true;

  *iteration_oss << "predictTarget(OMNI" << robotNumber + 1 << ") -> ";

  using namespace boost::random;

  ROS_DEBUG("OMNI%d is predicting the target state.", robotNumber + 1);

  // Random acceleration model
  normal_distribution<> targetAcceleration(TARGET_RAND_MEAN,
                                           TARGET_RAND_STDDEV);

  // If the iterationTime is too high, consider the default value
  if (iterationTime_ > ITERATION_TIME_MAX)
    iterationTime_ = ITERATION_TIME_DEFAULT;

  for (int i = 0; i < nParticles_; i++)
  {
    // TODO what exactly should happen to ball velocity?

    // Get random accelerations
    pdata_t accel[STATES_PER_TARGET] = { targetAcceleration(seed_),
                                         targetAcceleration(seed_),
                                         targetAcceleration(seed_) };

    for (uint s = 0; s < STATES_PER_TARGET; ++s)
    {
      particles_[O_TARGET + s][i] += state_.target.vel[s] * iterationTime_ +
                                     0.5 * accel[s] * pow(iterationTime_, 2);
    }
  }
}

void ParticleFilter::fuseRobots()
{
  if (!initialized_ || state_.robotsFused)
    return;

  *iteration_oss << "fuseRobots() -> ";

  ROS_DEBUG("Fusing Robots");

  state_.robotsFused = true;

  for (uint p = 0; p < nParticles_; ++p)
  {
    std::vector<pdata_t> probabilities(nRobots_, 1.0);
    std::vector<uint> landmarksUsed(nRobots_, 0);

    for (uint l = 0; l < nLandmarks_; ++l)
    {
      for (uint r = 0; r < nRobots_; ++r)
      {
        if (false == robotsUsed_[r] ||
            false == bufLandmarkObservations_[r][l].found)
          continue;

        uint o_robot = r * nStatesPerRobot_;

        LandmarkObservation& m = bufLandmarkObservations_[r][l];

        // Observation in robot frame
        Eigen::Vector2d Zrobot(m.x, m.y);

        // Transformation to global frame
        Eigen::Rotation2Dd Rrobot(-particles_[o_robot + O_THETA][p]);
        Eigen::Vector2d Srobot(particles_[o_robot + O_X][p],
                               particles_[o_robot + O_Y][p]);
        Eigen::Vector2d Zglobal = Srobot + Rrobot * Zrobot;

        // Error in observation
        Eigen::Vector2d LM(landmarksMap_[l].x, landmarksMap_[l].y);

        Eigen::Vector2d Zglobal_err = LM - Zglobal;

        // The values of interest to the particle weights
        // Note: using Eigen wasn't of particular interest here since it does
        // not allow for transposing a non-dynamic matrix
        float expArg = -0.5 * (Zglobal_err(0) * Zglobal_err(0) / m.covXX +
                               Zglobal_err(1) * Zglobal_err(1) / m.covYY);
        float detValue = 1.0; // pow( (2*M_PI*m.covXX*m.covYY),-0.5);

        probabilities[r] *= detValue * exp(expArg);
        landmarksUsed[r]++;

        ROS_DEBUG_COND(
            p == 0,
            "OMNI%d's particle 0 is at {%f;%f;%f}, sees landmark %d with "
            "certainty %f%%, at {%f;%f} "
            "although it is at {%f;%f}",
            r + 1, particles_[o_robot + O_X][p], particles_[o_robot + O_Y][p],
            particles_[o_robot + O_THETA][p], l, 100 * (detValue * exp(expArg)),
            Zglobal(0), Zglobal(1), LM(0), LM(1));
      }
    }

    // Update mean error vector
    for (uint r = 0; r < nRobots_; ++r)
    {
      if (false == robotsUsed_[r])
        continue;

      if (landmarksUsed[r] > 0)
        weightComponents_[r][p] = probabilities[r];
    }
  }

  // Reset weights to 1.0 - later we will multiply by the various weight
  // components
  resetWeights(1.0);

  // Duplicate particles
  particles_t dupParticles(particles_);

  for (uint r = 0; r < nRobots_; ++r)
  {
    if (false == robotsUsed_[r])
      continue;

    uint o_robot = r * nStatesPerRobot_;

    // Create a vector of indexes according to a descending order of the weights
    // components of robot r
    std::vector<uint> sorted = order_index<pdata_t>(weightComponents_[r], DESC);

    // For every particle
    for (uint p = 0; p < nParticles_; ++p)
    {
      // Re-order the particle subsets of this robot
      uint sort_index = sorted[p];

      for (uint s = 0; s < nStatesPerRobot_; ++s)
        particles_[o_robot + s][p] = dupParticles[o_robot + s][sort_index];

      // Update the particle weight (will get multiplied nRobots times and get a
      // lower value)
      particles_[O_WEIGHT][p] *= weightComponents_[r][sort_index];
    }
  }

  // If the target measurements were performed prior to this function ending,
  // then we should call the target fusion step here
  if (!state_.targetFused && state_.allTargetMeasurementsDone())
    fuseTarget();
}

void ParticleFilter::fuseTarget()
{
  if (!initialized_ || state_.targetFused)
    return;

  *iteration_oss << "fuseTarget() -> ";

  ROS_DEBUG("Fusing Target");

  state_.targetFused = true;

#ifndef DONT_FUSE_TARGET
  // If ball not seen by any robot, just skip all of this
  bool ballSeen = false;
  for (std::vector<TargetObservation>::iterator it =
           bufTargetObservations_.begin();
       it != bufTargetObservations_.end(); ++it)
    if (it->found)
    {
      ballSeen = true;
      break;
    }

  // resample and exit if ball not seen by any robot
  if (!ballSeen)
  {
    *iteration_oss << "Ball not seen ";
    return resample();
  }

  // If program is here, at least one robot saw the ball

  // For every particle m in the particle set [1:M]
  for (uint m = 0; m < nParticles_; ++m)
  {
    // Keep track of the maximum contributed weight and that particle's index
    pdata_t maxTargetSubParticleWeight = 0.0;
    uint mStar = 0;

    // Find the particle m* in the set [m:M] for which the weight contribution
    // by the target subparticle to the full weight is maximum
    for (uint p = 0; p < nParticles_; ++p)
    {
      std::vector<pdata_t> probabilities(nRobots_, 1.0);

      // Observations of the target by all robots
      for (uint r = 0; r < nRobots_; ++r)
      {
        if (false == robotsUsed_[r] || false == bufTargetObservations_[r].found)
          continue;

        uint o_robot = r * nStatesPerRobot_;

        TargetObservation& obs = bufTargetObservations_[r];

        // Observation in robot frame
        Eigen::Vector3d Zrobot(obs.x, obs.y, obs.z);

        // Transformation to global frame
        // Affine creates a (Dim+1) * (Dim+1) matrix and sets
        // last row to [0 0 ... 1]
        Eigen::Transform<pdata_t, 2, Eigen::Affine> toGlobal(
            Eigen::Rotation2Dd(-particles_[o_robot + O_THETA][m]));
        Eigen::Vector3d Srobot(particles_[o_robot + O_X][m],
                               particles_[o_robot + O_Y][m], 0.0);
        Eigen::Vector3d Zglobal = Srobot + toGlobal * Zrobot;

        // Error in observation
        Eigen::Vector3d Target(particles_[O_TARGET + O_TX][p],
                               particles_[O_TARGET + O_TY][p],
                               particles_[O_TARGET + O_TZ][p]);

        // TODO should the Y frame be inverted?
        Eigen::Vector3d Z_Zcap = Zrobot - (Target - Zglobal);

        // The values of interest to the particle weights
        // Note: using Eigen wasn't of particular interest here since it does
        // not allow for transposing a non-dynamic matrix

        // Q - diagonal matrix where the diagonal is ( covXX, covYY, 0.1 )
        // Qinv - inverse of the diagonal matrix, consists of inverting each
        // value in the diagonal

        float expArg = -0.5 * (Z_Zcap[0] * Z_Zcap[0] / obs.covXX +
                               Z_Zcap[1] * Z_Zcap[1] / obs.covYY +
                               Z_Zcap[2] * Z_Zcap[2] / 0.1);

        float detValue = 1.0;

        probabilities[r] *= detValue * exp(expArg);
      }

      // Calculate total weight contributed by this particle
      pdata_t totalWeight =
          std::accumulate(probabilities.begin(), probabilities.end(), 0.0);

      // If the weight is the maximum as of now, update the maximum and set
      // particle p as mStar
      if (totalWeight > maxTargetSubParticleWeight)
      {
        // Swap particle m with m* so that the most relevant (in terms of
        // weight)
        // target subparticle is at the lowest indexes
        maxTargetSubParticleWeight = totalWeight;
        mStar = p;
      }
    }

    // Particle m* has been found, let's swap the subparticles
    for (uint i = 0; i < STATES_PER_TARGET; ++i)
      std::swap(particles_[O_TARGET + i][m], particles_[O_TARGET + i][mStar]);

    // Update the weight of this particle
    particles_[O_WEIGHT][m] *= maxTargetSubParticleWeight;
  }

// The target subparticles are now reordered according to their weight
// contribution
#endif

  // Start resampling
  resample();
}

void ParticleFilter::modifiedMultinomialResampler()
{
  // Implementing a very basic resampler... a particle gets selected
  // proportional to its weight and 50% of the top particles are kept

  particles_t duplicate(particles_);

  std::vector<pdata_t> cumulativeWeights(nParticles_);
  cumulativeWeights[0] = duplicate[O_WEIGHT][0];

  for (int par = 1; par < nParticles_; par++)
  {
    cumulativeWeights[par] =
        cumulativeWeights[par - 1] + duplicate[O_WEIGHT][par];
  }

  int halfParticles = nParticles_ * 0.5;

  // Resample the rest of the set
  for (int par = halfParticles; par < nParticles_; par++)
  {
    boost::random::uniform_real_distribution<> dist(0, 1);
    float randNo = dist(seed_);

    int m = 0;
    while (randNo > cumulativeWeights[m])
      m++;

    for (int k = 0; k < nSubParticleSets_; k++)
      particles_[k][par] = duplicate[k][m];
  }

  // Every particle with the same weight
  resetWeights(1.0 / nParticles_);

  ROS_DEBUG("End of modifiedMultinomialResampler()");
}

void ParticleFilter::resample()
{
  if (!initialized_ || state_.resampled)
    return;

  state_.resampled = true;

  *iteration_oss << "resample() -> ";
  ROS_DEBUG("Resampling");

  for (uint r = 0; r < nRobots_; ++r)
  {
    if (false == robotsUsed_[r])
      continue;

    uint o_robot = r * nStatesPerRobot_;

    pdata_t stdX = pfuclt_aux::calc_stdDev<pdata_t>(particles_[o_robot + O_X]);
    pdata_t stdY = pfuclt_aux::calc_stdDev<pdata_t>(particles_[o_robot + O_Y]);
    pdata_t stdTheta =
        pfuclt_aux::calc_stdDev<pdata_t>(particles_[o_robot + O_THETA]);

    state_.robots[r].conf = stdX + stdY + stdTheta;

    ROS_DEBUG("OMNI%d stdX = %f, stdY = %f, stdTheta = %f", r + 1, stdX, stdY,
              stdTheta);
  }

  // Calc. sum of weights
  pdata_t weightSum = std::accumulate(particles_[O_WEIGHT].begin(),
                                      particles_[O_WEIGHT].end(), 0.0);

  ROS_DEBUG("WeightSum before resampling = %f", weightSum);

  // printWeights("before resampling: ");

  if (weightSum < MIN_WEIGHTSUM)
  {
    ROS_WARN("Zero weightsum - returning without resampling");

    // Print iteration and state information
    *iteration_oss << "FAIL! -> ";
  }

  else
  {
    // All resamplers use normalized weights
    for (uint p = 0; p < nParticles_; ++p)
      particles_[O_WEIGHT][p] = particles_[O_WEIGHT][p] / weightSum;

    modifiedMultinomialResampler();

    // printWeights("after resampling: ");
  }

  // Resampling done, find the current state belief
  estimate();
}

void ParticleFilter::estimate()
{
  *iteration_oss << "estimate() -> ";

  pdata_t weightSum = std::accumulate(particles_[O_WEIGHT].begin(),
                                      particles_[O_WEIGHT].end(), 0.0);

  // ROS_DEBUG("WeightSum when estimating = %f", weightSum);

  if (weightSum < MIN_WEIGHTSUM)
  {
    // Print iteration and state information
    *iteration_oss << "DONE without estimating!";
    ROS_DEBUG("Iteration: %s", iteration_oss->str().c_str());

    state_.print();

    // Clear ostringstream
    iteration_oss->str("");
    iteration_oss->clear();

    // Start next iteration
    return nextIteration();
  }

  subparticles_t normalizedWeights(particles_[O_WEIGHT]);

  // Normalize the weights
  for (uint i = 0; i < nParticles_; ++i)
    normalizedWeights[i] = normalizedWeights[i] / weightSum;

  // For each robot
  for (uint r = 0; r < nRobots_; ++r)
  {
    // If the robot isn't playing, skip it
    if (false == robotsUsed_[r])
      continue;

    uint o_robot = r * nStatesPerRobot_;

    // A vector of weighted means that will be calculated in the next loop
    std::vector<double> weightedMeans(nStatesPerRobot_ - 1, 0.0);

    // For theta we will obtain the mean of circular quantities, by converting
    // to cartesian coordinates, placing each angle in the unit circle,
    // averaging these points and finally converting again to polar
    double weightedMeanThetaCartesian[2] = { 0, 0 };

    // ..and each particle
    for (uint p = 0; p < nParticles_; ++p)
    {
      // Accumulate the state proportionally to the particle's normalized weight
      for (uint g = 0; g < nStatesPerRobot_ - 1; ++g)
      {
        weightedMeans[g] += particles_[o_robot + g][p] * normalizedWeights[p];
      }

      // Mean of circular quantities for theta
      weightedMeanThetaCartesian[O_X] +=
          cos(particles_[o_robot + O_THETA][p]) * normalizedWeights[p];
      weightedMeanThetaCartesian[O_Y] +=
          sin(particles_[o_robot + O_THETA][p]) * normalizedWeights[p];
    }

    // Put the angle back in polar coordinates
    double weightedMeanThetaPolar =
        atan2(weightedMeanThetaCartesian[O_Y], weightedMeanThetaCartesian[O_X]);

    // Save in the robot state
    // Can't use easy copy since one is using double precision
    state_.robots[r].pose[O_X] = weightedMeans[O_X];
    state_.robots[r].pose[O_Y] = weightedMeans[O_Y];
    state_.robots[r].pose[O_THETA] = weightedMeanThetaPolar;
  }

  // Target weighted means
  std::vector<double> targetWeightedMeans(STATES_PER_TARGET, 0.0);

  // For each particle
  for (uint p = 0; p < nParticles_; ++p)
  {
    for (uint t = 0; t < STATES_PER_TARGET; ++t)
    {
      targetWeightedMeans[t] +=
          particles_[O_TARGET + t][p] * normalizedWeights[p];
    }
  }

  // Update position
  // Can't use easy copy since one is using double precision
  state_.target.pos[O_TX] = targetWeightedMeans[O_TX];
  state_.target.pos[O_TY] = targetWeightedMeans[O_TY];
  state_.target.pos[O_TZ] = targetWeightedMeans[O_TZ];

  // Add to the velocity estimator
  double timeNow = ros::Time::now().toNSec() * 1e-9;
  state_.targetVelocityEstimator.insert(timeNow, targetWeightedMeans);

  // Ball velocity is estimated using linear regression
  if (state_.targetVelocityEstimator.isReadyToEstimate())
  {
    state_.target.vel[O_TX] = state_.targetVelocityEstimator.estimate(O_X);
    state_.target.vel[O_TY] = state_.targetVelocityEstimator.estimate(O_TY);
    state_.target.vel[O_TZ] = state_.targetVelocityEstimator.estimate(O_TZ);
  }

  // Print iteration and state information
  *iteration_oss << "DONE!";
  ROS_DEBUG("Iteration: %s", iteration_oss->str().c_str());

  state_.print();

  // Clear ostringstream
  iteration_oss->str("");
  iteration_oss->clear();

  // Start next iteration
  nextIteration();
}

void ParticleFilter::printWeights(std::string pre)
{
  std::ostringstream debug;
  debug << "Weights " << pre;
  for (uint i = 0; i < nParticles_; ++i)
    debug << particles_[O_WEIGHT][i] << " ";

  ROS_DEBUG("%s", debug.str().c_str());
}

void ParticleFilter::assign(const pdata_t value)
{
  for (int i = 0; i < nSubParticleSets_; ++i)
    assign(value, i);
}

void ParticleFilter::assign(const pdata_t value, const uint index)
{
  particles_[index].assign(nParticles_, value);
}

// TODO set different values for position and orientation, targets, etc
// Simple version, use default values - randomize all values between [-10,10]
void ParticleFilter::init()
{
  if (initialized_)
    return;

  int lvalue = -10;
  int rvalue = 10;

  std::vector<double> def((nSubParticleSets_ - 1) * 2);

  for (int i = 0; i < def.size(); i += 2)
  {
    def[i] = lvalue;
    def[i + 1] = rvalue;
  }

  // Call the custom function with these values
  init(def);
}

// Overloaded function when using custom values
void ParticleFilter::init(const std::vector<double> custom)
{
  if (initialized_)
    return;

  // Set flag
  initialized_ = true;

  ROS_INFO("Initializing particle filter");

  ROS_WARN_COND(custom.size() != ((nSubParticleSets_ - 1) * 2),
                "The provided vector for initilization does not have the "
                "correct size (should have %d elements",
                (nSubParticleSets_ - 1) * 2);

  // For all subparticle sets except the particle weights
  for (int i = 0; i < custom.size() / 2; ++i)
  {
    ROS_DEBUG("Values for distribution: %.4f %.4f", custom[2 * i],
              custom[2 * i + 1]);

    boost::random::uniform_real_distribution<> dist(custom[2 * i],
                                                    custom[2 * i + 1]);

    // Sample a value from the uniform distribution for each particle
    for (uint p = 0; p < nParticles_; ++p)
      particles_[i][p] = dist(seed_);
  }

  // Particle weights init with same weight (1/nParticles)
  resetWeights(1.0 / nParticles_);

  // Reset PF state
  state_.reset();

  ROS_INFO("Particle filter initialized");
}

void ParticleFilter::predict(const uint robotNumber, const Odometry odom)
{
  if (!initialized_)
    return;

  // Change predicted state
  state_.predicted[robotNumber] = true;

  *iteration_oss << "predict(OMNI" << robotNumber + 1 << ") -> ";

  using namespace boost::random;

  // Variables concerning this robot specifically
  int robot_offset = robotNumber * nStatesPerRobot_;
  float alpha[4] = { alpha_[robotNumber * 4 + 0], alpha_[robotNumber * 4 + 1],
                     alpha_[robotNumber * 4 + 2], alpha_[robotNumber * 4 + 3] };

  ROS_DEBUG("Predicting for OMNI%d with alphas [%f,%f,%f,%f]", robotNumber + 1,
            alpha[0], alpha[1], alpha[2], alpha[3]);

  // Determining the propagation of the robot state through odometry
  pdata_t deltaRot =
      atan2(odom.y, odom.x) -
      state_.robots[robotNumber]
          .pose[O_THETA]; // Uses the previous overall belief of orientation
  pdata_t deltaTrans = sqrt(odom.x * odom.x + odom.y * odom.y);
  pdata_t deltaFinalRot = odom.theta - deltaRot;

  // Create an error model based on a gaussian distribution
  normal_distribution<> deltaRotEffective(deltaRot, alpha[0] * fabs(deltaRot) +
                                                        alpha[1] * deltaTrans);

  normal_distribution<> deltaTransEffective(
      deltaTrans,
      alpha[2] * deltaTrans + alpha[3] * fabs(deltaRot + deltaFinalRot));

  normal_distribution<> deltaFinalRotEffective(
      deltaFinalRot, alpha[0] * fabs(deltaFinalRot) + alpha[1] * deltaTrans);

  ROS_DEBUG("When predicting for OMNI%d used means [%f,%f,%f] and stdDevs "
            "[%f,%f,%f] for Rot, Trans and FinalRot",
            robotNumber + 1, deltaRotEffective.mean(),
            deltaTransEffective.mean(), deltaFinalRotEffective.mean(),
            deltaRotEffective.sigma(), deltaTransEffective.sigma(),
            deltaFinalRotEffective.sigma());

  for (int i = 0; i < nParticles_; i++)
  {
    // Rotate to final position
    particles_[O_THETA + robot_offset][i] += deltaRotEffective(seed_);

    pdata_t sampleTrans = deltaTransEffective(seed_);

    // Translate to final position
    particles_[O_X + robot_offset][i] +=
        sampleTrans * cos(particles_[O_THETA + robot_offset][i]);
    particles_[O_Y + robot_offset][i] +=
        sampleTrans * sin(particles_[O_THETA + robot_offset][i]);

    // Rotate to final position and normalize angle
    particles_[O_THETA + robot_offset][i] = angles::normalize_angle(
        particles_[O_THETA + robot_offset][i] + deltaFinalRotEffective(seed_));
  }

  // If all robots have predicted, also predict the target state
  if (state_.allPredicted() && !state_.targetPredicted)
    predictTarget(robotNumber);

  // Start fusing if all robots have done their measurements and predictions
  if (!state_.robotsFused && state_.allLandmarkMeasurementsDone() &&
      state_.allPredicted())
    fuseRobots();
}

void ParticleFilter::saveAllLandmarkMeasurementsDone(const uint robotNumber)
{
  if (!initialized_)
    return;

  *iteration_oss << "allLandmarks(OMNI" << robotNumber + 1 << ") -> ";

  // Change state
  state_.landmarkMeasurementsDone[robotNumber] = true;

  // Start fusing if all robots have done their measurements and predictions
  if (!state_.robotsFused && state_.allLandmarkMeasurementsDone() &&
      state_.allPredicted())
    fuseRobots();
}

void ParticleFilter::saveAllTargetMeasurementsDone(const uint robotNumber)
{
  if (!initialized_)
    return;

  // Change state
  state_.targetMeasurementsDone[robotNumber] = true;

  *iteration_oss << "allTargets(OMNI" << robotNumber + 1 << ") -> ";

  // Start fusing if all robots have done their target measurements and the
  // robot fusion step has been performed
  if (!state_.targetFused && state_.robotsFused &&
      state_.allTargetMeasurementsDone())
    fuseTarget();
}

PFPublisher::PFPublisher(struct ParticleFilter::PFinitData& data,
                         struct PublishData publishData)
    : ParticleFilter(data), pubData(publishData),
      robotBroadcasters(data.nRobots), particleStdPublishers_(data.nRobots),
      robotGTPublishers_(data.nRobots), robotEstimatePublishers_(data.nRobots)
{
  // Prepare particle message
  msg_particles_.particles.resize(nParticles_);
  for (uint p = 0; p < nParticles_; ++p)
  {
    msg_particles_.particles[p].particle.resize(nSubParticleSets_);
  }

  // Subscribe and advertise the republishing of GT data, time synced with the
  // state publisher
  GT_sub_ = pubData.nh.subscribe<read_omni_dataset::LRMGTData>(
      "gtData_4robotExp", 10,
      boost::bind(&PFPublisher::gtDataCallback, this, _1));

  syncedGTPublisher_ = pubData.nh.advertise<read_omni_dataset::LRMGTData>(
      "/gtData_synced_pfuclt_estimate", 1000);

  // Other publishers
  robotStatePublisher_ = pubData.nh.advertise<read_omni_dataset::RobotState>(
      "/pfuclt_omni_poses", 1000);
  targetStatePublisher_ = pubData.nh.advertise<read_omni_dataset::BallData>(
      "/pfuclt_orangeBallState", 1000);
  particlePublisher_ = pubData.nh.advertise<pfuclt_omni_dataset::particles>(
      "/pfuclt_particles", 10);

  // Rviz visualization publishers
  // Target
  targetEstimatePublisher_ = pubData.nh.advertise<geometry_msgs::PointStamped>(
      "/target/estimatedPose", 1000);
  targetGTPublisher_ =
      pubData.nh.advertise<geometry_msgs::PointStamped>("/target/gtPose", 1000);
  targetParticlePublisher_ =
      pubData.nh.advertise<sensor_msgs::PointCloud>("/target/particles", 10);

  // Robots
  for (uint r = 0; r < nRobots_; ++r)
  {
    std::ostringstream robotName;
    robotName << "omni" << r + 1;

    // particle publisher
    particleStdPublishers_[r] = pubData.nh.advertise<geometry_msgs::PoseArray>(
        robotName.str() + "/particles", 1000);

    // estimated state
    robotEstimatePublishers_[r] =
        pubData.nh.advertise<geometry_msgs::PoseStamped>(
            robotName.str() + "/estimatedPose", 1000);

    // ground truth publisher
    robotGTPublishers_[r] = pubData.nh.advertise<geometry_msgs::PointStamped>(
        robotName.str() + "/gtPose", 1000);
  }

  ROS_INFO("It's a publishing particle filter!");
}

void PFPublisher::publishParticles()
{
  // The eval package would rather have the particles in the format
  // particle->subparticle instead, so we have to inverse it
  for (uint p = 0; p < nParticles_; ++p)
  {
    for (uint s = 0; s < nSubParticleSets_; ++s)
    {
      msg_particles_.particles[p].particle[s] = particles_[s][p];
    }
  }

  // Send it!
  particlePublisher_.publish(msg_particles_);

#ifdef PUBLISH_PTCLS
  // Also send as a series of PoseArray messages for each robot
  for (uint r = 0; r < nRobots_; ++r)
  {
    if (false == state_.robotsUsed[r])
      continue;

    uint o_robot = r * nStatesPerRobot_;
    geometry_msgs::PoseArray msgStd_particles;
    msgStd_particles.header.stamp = ros::Time::now();
    msgStd_particles.header.frame_id = "world";

    for (uint p = 0; p < nParticles_; ++p)
    {
      tf2::Quaternion tf2q(tf2::Vector3(0, 0, 1),
                           particles_[o_robot + O_THETA][p]);
      tf2::Transform tf2t(tf2q, tf2::Vector3(particles_[o_robot + O_X][p],
                                             particles_[o_robot + O_Y][p],
                                             pubData.robotHeight));

      geometry_msgs::Pose pose;
      tf2::toMsg(tf2t, pose);
      msgStd_particles.poses.insert(msgStd_particles.poses.begin(), pose);
    }

    particleStdPublishers_[r].publish(msgStd_particles);
  }

  // Send target particles as a pointcloud
  sensor_msgs::PointCloud target_particles;
  target_particles.header.stamp = ros::Time::now();
  target_particles.header.frame_id = "world";

  for (uint p = 0; p < nParticles_; ++p)
  {
    geometry_msgs::Point32 point;
    point.x = particles_[O_TARGET + O_TX][p];
    point.y = particles_[O_TARGET + O_TY][p];
    point.z = particles_[O_TARGET + O_TZ][p];

    target_particles.points.insert(target_particles.points.begin(), point);
  }
  targetParticlePublisher_.publish(target_particles);

#endif
}

void PFPublisher::publishRobotStates()
{
  // This is pretty much copy and paste
  for (uint r = 0; r < nRobots_; ++r)
  {
    if (false == state_.robotsUsed[r])
      continue;

    std::ostringstream robotName;
    robotName << "OMNI" << r + 1;

    ParticleFilter::State::robotState_s& pfState = state_.robots[r];
    geometry_msgs::Pose& rosState = msg_state_.robotPose[r].pose.pose;

    // Create from Euler angles
    tf2::Quaternion tf2q(tf2::Vector3(0, 0, 1), pfState.pose[O_THETA]);
    tf2::Transform tf2t(tf2q, tf2::Vector3(pfState.pose[O_X], pfState.pose[O_Y],
                                           pubData.robotHeight));

    // Transform to our message type
    tf2::toMsg(tf2t, rosState);

#ifdef BROADCAST_TF_AND_POSES
    // TF2 broadcast
    geometry_msgs::TransformStamped estTransf;
    estTransf.header.stamp = ros::Time::now();
    estTransf.header.frame_id = "world";
    estTransf.child_frame_id = robotName.str();
    estTransf.transform = tf2::toMsg(tf2t);
    robotBroadcasters[r].sendTransform(estTransf);

    // Publish as a standard pose msg using the previous TF
    geometry_msgs::PoseStamped estPose;
    estPose.header.stamp = estTransf.header.stamp;
    estPose.header.frame_id = estTransf.child_frame_id;
    // Pose is everything at 0 as it's the same as the TF

    robotEstimatePublishers_[r].publish(estPose);
#endif
  }

  robotStatePublisher_.publish(msg_state_);
}

void PFPublisher::publishTargetState()
{
  // Our custom message type
  msg_target_.x = state_.target.pos[O_TX];
  msg_target_.y = state_.target.pos[O_TY];
  msg_target_.z = state_.target.pos[O_TZ];
  targetStatePublisher_.publish(msg_target_);

#ifdef BROADCAST_TF_AND_POSES
  // Publish as a standard pose msg using the previous TF
  geometry_msgs::PointStamped estPoint;
  estPoint.header.stamp = ros::Time::now();
  estPoint.header.frame_id = "world";
  estPoint.point.x = state_.target.pos[O_TX];
  estPoint.point.y = state_.target.pos[O_TY];
  estPoint.point.z = state_.target.pos[O_TZ];

  targetEstimatePublisher_.publish(estPoint);
#endif
}

void PFPublisher::publishGTData()
{
  // Publish custom format
  syncedGTPublisher_.publish(msg_GT_);

  // Publish poses for every robot
  geometry_msgs::PointStamped gtPoint;
  gtPoint.header.stamp = ros::Time::now();
  gtPoint.header.frame_id = "world";

  gtPoint.point = msg_GT_.poseOMNI1.pose.position;
  robotGTPublishers_[0].publish(gtPoint);

  gtPoint.point = msg_GT_.poseOMNI3.pose.position;
  robotGTPublishers_[2].publish(gtPoint);

  gtPoint.point = msg_GT_.poseOMNI4.pose.position;
  robotGTPublishers_[3].publish(gtPoint);

  gtPoint.point = msg_GT_.poseOMNI5.pose.position;
  robotGTPublishers_[4].publish(gtPoint);

  // Publish for the target as well
  if (msg_GT_.orangeBall3DGTposition.found)
  {
    gtPoint.point.x = msg_GT_.orangeBall3DGTposition.x;
    gtPoint.point.y = msg_GT_.orangeBall3DGTposition.y;
    gtPoint.point.z = msg_GT_.orangeBall3DGTposition.z;
    targetGTPublisher_.publish(gtPoint);
  }
}

void PFPublisher::gtDataCallback(
    const read_omni_dataset::LRMGTData::ConstPtr& gtMsgReceived)
{
  msg_GT_ = *gtMsgReceived;
}

void PFPublisher::nextIteration()
{
  // Time stamps using ros time
  msg_state_.header.stamp = msg_GT_.header.stamp = msg_target_.header.stamp =
      ros::Time::now();

  // Publish the particles first
  publishParticles();

  // Publish robot states
  publishRobotStates();

  // Publish target state
  publishTargetState();

  // Publish GT data
  publishGTData();

  // Call the base class method
  ParticleFilter::nextIteration();
}

// end of namespace pfuclt_ptcls
}
