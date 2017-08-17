#include <pfuclt_omni_dataset/pfuclt_particles.h>
#include <boost/foreach.hpp>
#include <angles/angles.h>

//#define RECONFIGURE_ALPHAS true

namespace pfuclt_omni_dataset
{

ParticleFilter::ParticleFilter(struct PFinitData& data)
    : dynamicVariables_(data.nh, data.nRobots),
      nh_(data.nh), nParticles_((uint)dynamicVariables_.nParticles), mainRobotID_(data.mainRobotID - 1),
      nTargets_(data.nTargets), nStatesPerRobot_(data.statesPerRobot), nRobots_(data.nRobots),
      nSubParticleSets_(data.nTargets * STATES_PER_TARGET + data.nRobots * data.statesPerRobot + 1),
      nLandmarks_(data.nLandmarks),
      particles_(nSubParticleSets_, subparticles_t(nParticles_)),
      weightComponents_(data.nRobots, subparticles_t(nParticles_, 0.0)),
      seed_(time(0)), initialized_(false),
      landmarksMap_(data.landmarksMap),
      robotsUsed_(data.robotsUsed),
      bufLandmarkObservations_(data.nRobots, std::vector<LandmarkObservation>(data.nLandmarks)),
      bufTargetObservations_(data.nRobots),
      durationSum(ros::WallDuration(0)),
      numberIterations(0),
      state_(data.statesPerRobot, data.nRobots),
      iteration_oss(new std::ostringstream("")),
      O_TARGET(data.nRobots * data.statesPerRobot),
      O_WEIGHT(nSubParticleSets_ - 1)
{
  ROS_INFO("Created particle filter with dimensions %d, %d",
           (int)particles_.size(), (int)particles_[0].size());

  // Bind dynamic reconfigure callback
  dynamic_reconfigure::Server<DynamicConfig>::CallbackType
      callback;
  callback = boost::bind(&ParticleFilter::dynamicReconfigureCallback, this, _1);
  dynamicServer_.setCallback(callback);
}

void ParticleFilter::dynamicReconfigureCallback(DynamicConfig& config)
{
  // Skip first callback which is done automatically for some reason
  if (dynamicVariables_.firstCallback)
  {
    dynamicVariables_.firstCallback = false;
    return;
  }

  if (!initialized_)
    return;

  ROS_INFO("Dynamic Reconfigure Callback:\n\tparticles = "
           "%d\n\tresampling_percentage_to_keep = "
           "%f\n\tpredict_model_stddev = "
           "%f\n\tOMNI1_alpha=%s\n\tOMNI3_alpha=%s\n\tOMNI4_alpha=%s\n\tOMNI5_"
           "alpha=%s",
           config.particles,
           config.groups.resampling.percentage_to_keep,
           config.groups.target.predict_model_stddev,
           config.groups.alphas.OMNI1_alpha.c_str(),
           config.groups.alphas.OMNI3_alpha.c_str(),
           config.groups.alphas.OMNI4_alpha.c_str(),
           config.groups.alphas.OMNI5_alpha.c_str());

  // Resize particles and re-initialize the pf if value changed
  if (dynamicVariables_.nParticles != config.particles)
  {
    ROS_INFO("Resizing particles to %d and re-initializing the pf",
             config.particles);

    resize_particles(config.particles);
    nParticles_ = config.particles;
  }

  // Update with desired values
  dynamicVariables_.nParticles = config.particles;
  dynamicVariables_.resamplingPercentageToKeep =
      config.groups.resampling.percentage_to_keep;
  dynamicVariables_.targetRandStddev =
      config.groups.target.predict_model_stddev;

// Alpha values updated only if using the original dataset
#ifdef RECONFIGURE_ALPHAS
  dynamicVariables_.fill_alpha(0, config.groups.alphas.OMNI1_alpha);
  dynamicVariables_.fill_alpha(2, config.groups.alphas.OMNI3_alpha);
  dynamicVariables_.fill_alpha(3, config.groups.alphas.OMNI4_alpha);
  dynamicVariables_.fill_alpha(4, config.groups.alphas.OMNI5_alpha);
#endif
}

void ParticleFilter::spreadTargetParticlesSphere(float particlesRatio,
                                                 pdata_t center[3],
                                                 float radius)
{
  uint particlesToSpread = nParticles_ * particlesRatio;

  boost::random::uniform_real_distribution<> dist(-radius, radius);

  for (uint p = 0; p < particlesToSpread; ++p)
  {
    for (uint s = 0; s < STATES_PER_TARGET; ++s)
      particles_[O_TARGET + s][p] = center[s] + dist(seed_);
  }
}

void ParticleFilter::predictTarget()
{
  *iteration_oss << "predictTarget() -> ";

  using namespace boost::random;

  // Random acceleration model
  normal_distribution<> targetAcceleration(TARGET_RAND_MEAN,
                                           dynamicVariables_.targetRandStddev);

  for (uint p = 0; p < nParticles_; p++)
  {
    // Get random accelerations
    pdata_t accel[STATES_PER_TARGET] = { (pdata_t)targetAcceleration(seed_),
                                         (pdata_t)targetAcceleration(seed_),
                                         (pdata_t)targetAcceleration(seed_) };

    // Use random acceleration model
    for (uint s = 0; s < STATES_PER_TARGET; ++s)
    {
      particles_[O_TARGET + s][p] += 0.5 * accel[s] * pow(targetIterationTime_.diff, 2);
    }
  }
}

void ParticleFilter::fuseRobots()
{
  *iteration_oss << "fuseRobots() -> ";

  // Save the latest observation time to be used when publishing
  savedLatestObservationTime_ = latestObservationTime_;

  // Keeps track of number of landmarks seen for each robot
  std::vector<uint> landmarksSeen(nRobots_, 0);

  // Will track the probability propagation based on the landmark observations
  // for each robot
  std::vector<subparticles_t> probabilities(nRobots_,
                                            subparticles_t(nParticles_, 1.0));

  // For every robot
  for (uint r = 0; r < nRobots_; ++r)
  {
    // If not used, skip
    if (false == robotsUsed_[r])
      continue;

    // Index offset for this robot in the particles vector
    uint o_robot = r * nStatesPerRobot_;

    // For every landmark
    for (uint l = 0; l < nLandmarks_; ++l)
    {
      // If landmark not seen, skip
      if (false == bufLandmarkObservations_[r][l].found)
        continue;
      else
        ++(landmarksSeen[r]);

      // Reference to the observation for easier access
      LandmarkObservation& m = bufLandmarkObservations_[r][l];

      // Observation in robot frame
      Eigen::Matrix<pdata_t, 2, 1> Zrobot(m.x, m.y);

      // Landmark in global frame
      Eigen::Matrix<pdata_t, 2, 1> LMglobal(landmarksMap_[l].x,
                                            landmarksMap_[l].y);

#pragma omp parallel for
      for (uint p = 0; p < nParticles_; ++p)
      {

        // Robot pose <=> frame
        Eigen::Rotation2D<pdata_t> Rrobot(-particles_[o_robot + O_THETA][p]);
        Eigen::Matrix<pdata_t, 2, 1> Srobot(particles_[o_robot + O_X][p],
                                            particles_[o_robot + O_Y][p]);

        // Landmark to robot frame
        Eigen::Matrix<pdata_t, 2, 1> LMrobot = Rrobot * (LMglobal - Srobot);

        // Error in observation
        Eigen::Matrix<pdata_t, 2, 1> Zerr = LMrobot - Zrobot;

        // The values of interest to the particle weights
        // Note: using Eigen wasn't of particular interest here since it does
        // not allow for transposing a non-dynamic matrix
        float expArg = -0.5 * (Zerr(O_X) * Zerr(O_X) / m.covXX +
                               Zerr(O_Y) * Zerr(O_Y) / m.covYY);
        float detValue = 1.0; // pow((2 * M_PI * m.covXX * m.covYY), -0.5);

        /*
        ROS_DEBUG_COND(
            p == 0,
            "OMNI%d's particle 0 is at {%f;%f;%f}, sees landmark %d with "
            "certainty %f%%, and error {%f;%f}",
            r + 1, particles_[o_robot + O_X][p], particles_[o_robot + O_Y][p],
            particles_[o_robot + O_THETA][p], l, 100 * (detValue * exp(expArg)),
            Zerr(0), Zerr(1));
        */

        // Update weight component for this robot and particular particle
        probabilities[r][p] *= detValue * exp(expArg);
      }
    }
  }

  // Reset weights, later will be multiplied by weightComponents of each robot
  resetWeights(1.0);

  // Duplicate particles
  particles_t dupParticles(particles_);

  for (uint r = 0; r < nRobots_; ++r)
  {
    // Again, if robot not used, skip
    if (false == robotsUsed_[r])
      continue;

    // Check that at least one landmark was seen, if not send warning
    // If seen use probabilities vector, if not keep using the previous
    // weightComponents for this robot
    if (0 == landmarksSeen[r])
    {
      ROS_WARN("In this iteration, OMNI%d didn't see any landmarks", r + 1);

      // weightComponent stays from previous iteration
    }

    else
    {
      weightComponents_[r] = probabilities[r];
    }

    // Index offset for this robot in the particles vector
    uint o_robot = r * nStatesPerRobot_;

    // Create a vector of indexes according to a descending order of the weights
    // components of robot r
    std::vector<uint> sorted = order_index<pdata_t>(weightComponents_[r], DESC);

    // For every particle
    for (uint p = 0; p < nParticles_; ++p)
    {
      // Re-order the particle subsets of this robot
      uint sort_index = sorted[p];

      // Copy this sub-particle set from dupParticles' sort_index particle
      copyParticle(particles_, dupParticles, p, sort_index, o_robot,
                   o_robot + nStatesPerRobot_ - 1);

      // Update the particle weight (will get multiplied nRobots times and get a
      // lower value)
      particles_[O_WEIGHT][p] *= weightComponents_[r][sort_index];
    }
  }
}

void ParticleFilter::fuseTarget()
{
  *iteration_oss << "fuseTarget() -> ";

  // If ball not seen by any robot, just skip all of this
  bool ballSeen = false;
  for (std::vector<TargetObservation>::iterator it =
           bufTargetObservations_.begin();
       it != bufTargetObservations_.end(); ++it)
  {
    if (it->found)
    {
      ballSeen = true;
      break;
    }
  }

  // Update ball state
  state_.target.seen = ballSeen;

  // exit if ball not seen by any robot
  if (!ballSeen)
  {
    *iteration_oss << "Ball not seen ->";
    return;
  }
  // If program is here, at least one robot saw the ball

  // Instance variables to be worked in the loops
  pdata_t maxTargetSubParticleWeight, totalWeight;
  uint m, p, mStar, r, o_robot;
  float expArg, detValue, Z[3], Zcap[3], Z_Zcap[3];
  TargetObservation* obs;

  // For every particle m in the particle set [1:M]
  for (m = 0; m < nParticles_; ++m)
  {
    // Keep track of the maximum contributed weight and that particle's index
    maxTargetSubParticleWeight = -1.0f;
    mStar = m;

// Find the particle m* in the set [m:M] for which the weight contribution
// by the target subparticle to the full weight is maximum
#pragma omp parallel for private(p, r, o_robot, obs, expArg, detValue, Z,      \
                                 Zcap, Z_Zcap)
    for (p = m; p < nParticles_; ++p)
    {
      // Vector with probabilities for each robot, starting at 0.0 in case the
      // robot hasn't seen the ball
      std::vector<pdata_t> probabilities(nRobots_, 0.0);

      // Observations of the target by all robots
      for (r = 0; r < nRobots_; ++r)
      {
        if (false == robotsUsed_[r] || false == bufTargetObservations_[r].found)
          continue;

        // Usefull variables
        obs = &bufTargetObservations_[r];
        o_robot = r * nStatesPerRobot_;

        // Observation model
        Z[0] = obs->x;
        Z[1] = obs->y;
        Z[2] = obs->z;
        Zcap[0] =
            (particles_[O_TARGET + O_TX][p] - particles_[o_robot + O_X][m]) *
                (cos(particles_[o_robot + O_THETA][m])) +
            (particles_[O_TARGET + O_TY][p] - particles_[o_robot + O_Y][m]) *
                (sin(particles_[o_robot + O_THETA][m]));
        Zcap[1] =
            -(particles_[O_TARGET + O_TX][p] - particles_[o_robot + O_X][m]) *
                (sin(particles_[o_robot + O_THETA][m])) +
            (particles_[O_TARGET + O_TY][p] - particles_[o_robot + O_Y][m]) *
                (cos(particles_[o_robot + O_THETA][m]));
        Zcap[2] = particles_[O_TARGET + O_TZ][p];
        Z_Zcap[0] = Z[0] - Zcap[0];
        Z_Zcap[1] = Z[1] - Zcap[1];
        Z_Zcap[2] = Z[2] - Zcap[2];

        expArg = -0.5 * (Z_Zcap[0] * Z_Zcap[0] / obs->covXX +
                         Z_Zcap[1] * Z_Zcap[1] / obs->covYY +
                         Z_Zcap[2] * Z_Zcap[2] / .04);
        detValue =
            1.0; // pow((2 * M_PI * obs->covXX * obs->covYY * 10.0), -0.5);

        // Probability value for this robot and this particle
        probabilities[r] = detValue * exp(expArg);
      }

      // Total weight contributed by this particle
      totalWeight =
          std::accumulate(probabilities.begin(), probabilities.end(), 0.0);

      // If the weight is the maximum as of now, update the maximum and set
      // particle p as mStar
      if (totalWeight > maxTargetSubParticleWeight)
      {
// Swap particle m with m* so that the most relevant (in terms of
// weight)
// target subparticle is at the lowest indexes
#pragma omp critical
        {
          if (totalWeight > maxTargetSubParticleWeight)
          {
            maxTargetSubParticleWeight = totalWeight;
            mStar = p;
          }
        }
      }
    }

    // Particle m* has been found, let's swap the subparticles
    for (uint i = 0; i < STATES_PER_TARGET; ++i)
      std::swap(particles_[O_TARGET + i][m], particles_[O_TARGET + i][mStar]);

    // Update the weight of this particle
    particles_[O_WEIGHT][m] *= maxTargetSubParticleWeight;

    // The target subparticles are now reordered according to their weight
    // contribution

    // printWeights("After fuseTarget(): ");
  }
}

void ParticleFilter::modifiedMultinomialResampler(uint startAt)
{
  // Implementing a very basic resampler... a particle gets selected
  // proportional to its weight and startAt% of the top particles are kept

  particles_t duplicate(particles_);

  std::vector<pdata_t> cumulativeWeights(nParticles_);
  cumulativeWeights[0] = duplicate[O_WEIGHT][0];

  for (uint par = 1; par < nParticles_; par++)
  {
    cumulativeWeights[par] =
        cumulativeWeights[par - 1] + duplicate[O_WEIGHT][par];
  }

  int startParticle = nParticles_ * startAt;

  // Robot particle resampling starts only at startParticle
  for (uint par = startParticle; par < nParticles_; par++)
  {
    boost::random::uniform_real_distribution<> dist(0, 1);
    double randNo = dist(seed_);

    int m = 0;
    while (randNo > cumulativeWeights[m])
      m++;

    copyParticle(particles_, duplicate, par, m, 0, O_TARGET - 1);
  }

  // Target resampling is done for all particles
  for (uint par = 0; par < nParticles_; par++)
  {
    boost::random::uniform_real_distribution<> dist(0, 1);
    double randNo = dist(seed_);

    int m = 0;
    while (randNo > cumulativeWeights[m])
      m++;

    copyParticle(particles_, duplicate, par, m, O_TARGET,
                 nSubParticleSets_ - 1);
  }

  // ROS_DEBUG("End of modifiedMultinomialResampler()");
}

void ParticleFilter::resample()
{
  *iteration_oss << "resample() -> ";

  for (uint r = 0; r < nRobots_; ++r)
  {
    if (false == robotsUsed_[r])
      continue;

    uint o_robot = r * nStatesPerRobot_;

    pdata_t stdX = calc_stdDev<pdata_t>(particles_[o_robot + O_X]);
    pdata_t stdY = calc_stdDev<pdata_t>(particles_[o_robot + O_Y]);
    pdata_t stdTheta = calc_stdDev<pdata_t>(particles_[o_robot + O_THETA]);

    state_.robots[r].conf = 1 / (stdX + stdY + stdTheta);

    // ROS_DEBUG("OMNI%d stdX = %f, stdY = %f, stdTheta = %f", r + 1, stdX,
    // stdY,
    //          stdTheta);
  }

  // Calc. sum of weights
  double weightSum = std::accumulate(particles_[O_WEIGHT].begin(),
                                      particles_[O_WEIGHT].end(), 0.0);

  // ROS_DEBUG("WeightSum before resampling = %f", weightSum);

  // printWeights("before resampling: ");

  if (weightSum < MIN_WEIGHTSUM)
  {
    ROS_WARN("Zero weightsum - returning without resampling");

    // Print iteration and state information
    *iteration_oss << "FAIL! -> ";

    converged_ = false;
    resetWeights(1.0 / nParticles_);
    return;
  }

  converged_ = true;

  // All resamplers use normalized weights
  for (uint p = 0; p < nParticles_; ++p)
    particles_[O_WEIGHT][p] = (pdata_t)(particles_[O_WEIGHT][p] / weightSum);

  modifiedMultinomialResampler(dynamicVariables_.resamplingPercentageToKeep /
                               100.0);

  // printWeights("after resampling: ");
}

void ParticleFilter::estimate()
{
  *iteration_oss << "estimate() -> ";

  pdata_t weightSum = std::accumulate(particles_[O_WEIGHT].begin(),
                                      particles_[O_WEIGHT].end(), 0.0);

  // ROS_DEBUG("WeightSum when estimating = %f", weightSum);

  subparticles_t normalizedWeights(particles_[O_WEIGHT]);

  // Normalize the weights
  for (uint p = 0; p < nParticles_; ++p)
    normalizedWeights[p] = normalizedWeights[p] / weightSum;

  if (weightSum < MIN_WEIGHTSUM)
  {
    *iteration_oss << "DONE without estimating!";

    // Increase standard deviation for target prediction
    if (dynamicVariables_.targetRandStddev != TARGET_RAND_STDDEV_LOST)
    {
      dynamicVariables_.oldTargetRandSTddev =
          dynamicVariables_.targetRandStddev;
      dynamicVariables_.targetRandStddev = TARGET_RAND_STDDEV_LOST;
    }

    // Don't estimate
    return;
  }

  // Return (if necessary) to old target prediction model stddev
  if(dynamicVariables_.targetRandStddev != dynamicVariables_.oldTargetRandSTddev)
    dynamicVariables_.targetRandStddev = dynamicVariables_.oldTargetRandSTddev;

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

  *iteration_oss << "DONE!";
}

void ParticleFilter::printWeights(std::string pre)
{
  std::ostringstream debug;
  debug << "Weights " << pre;
  for (uint i = 0; i < nParticles_; ++i)
    debug << particles_[O_WEIGHT][i] << " ";

  ROS_DEBUG("%s", debug.str().c_str());
}

// TODO set different values for position and orientation, targets, etc
// Simple version, use default values - randomize all values between [-10,10]
void ParticleFilter::init()
{
  if (initialized_)
    return;

  int lvalue = -10;
  int rvalue = 10;

  std::vector<double> defRand((nSubParticleSets_ - 1) * 2);

  for (size_t i = 0; i < defRand.size(); i += 2)
  {
    defRand[i] = lvalue;
    defRand[i + 1] = rvalue;
  }

  std::vector<double> defPos((nRobots_ * 2), 0.0);

  // Call the custom function with these values
  init(defRand, defPos);
}

// Overloaded function when using custom values
void ParticleFilter::init(const std::vector<double>& customRandInit,
                          const std::vector<double>& customPosInit)
{
  if (initialized_)
    return;

  // Set flag
  initialized_ = true;

  bool flag_theta_given = (customPosInit.size() == nRobots_ * 3 &&
                           customRandInit.size() == nSubParticleSets_ * 3);
  size_t numVars =
      flag_theta_given ? customRandInit.size() / 3 : customRandInit.size() / 2;

  ROS_INFO("Initializing particle filter");

  // For all subparticle sets except the particle weights
  for (size_t i = 0; i < numVars; ++i)
  {
    ROS_DEBUG("Values for distribution: %.4f %.4f", customRandInit[2 * i],
              customRandInit[2 * i + 1]);

    boost::random::uniform_real_distribution<> dist(customRandInit[2 * i],
                                                    customRandInit[2 * i + 1]);

    // Sample a value from the uniform distribution for each particle
    for (uint p = 0; p < nParticles_; ++p)
      particles_[i][p] = (pdata_t)dist(seed_);
  }

  // Particle weights init with same weight (1/nParticles)
  resetWeights(1.0 / nParticles_);

  // Initialize pose with initial belief
  for (uint r = 0; r < nRobots_; ++r)
  {
    pdata_t tmp[] = { (pdata_t)customPosInit[2 * r + O_X], (pdata_t)customPosInit[2 * r + O_Y],
                      (pdata_t)-M_PI };
    state_.robots[r].pose = std::vector<pdata_t>(tmp, tmp + nStatesPerRobot_);
    if (flag_theta_given)
      state_.robots[r].pose.push_back(customPosInit[2 * r + O_THETA]);
  }

  // State should have the initial belief

  ROS_INFO("Particle filter initialized");
}

void ParticleFilter::predict(const uint robotNumber, const Odometry odom,
                             const ros::Time stamp)
{
  if (!initialized_)
    return;

  *iteration_oss << "predict(OMNI" << robotNumber + 1 << ") -> ";

  // If this is the main robot, update the odometry time
  if (mainRobotID_ == robotNumber)
  {
    odometryTime_.updateTime(ros::WallTime::now());
    iterationEvalTime_ = ros::WallTime::now();
  }
  using namespace boost::random;

  // Variables concerning this robot specifically
  int robot_offset = robotNumber * nStatesPerRobot_;
  std::vector<float>& alpha = dynamicVariables_.alpha[robotNumber];

  // Determining the propagation of the robot state through odometry
  pdata_t deltaRot = atan2(odom.y, odom.x);
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

  for (uint i = 0; i < nParticles_; i++)
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

  // Check if we should activate robotRandom
  // Only if no landmarks and no target seen
  uint nLandmarksSeen = 0;
  for (std::vector<LandmarkObservation>::iterator it =
           bufLandmarkObservations_[robotNumber].begin();
       it != bufLandmarkObservations_[robotNumber].end(); ++it)
  {
    if (it->found)
      nLandmarksSeen++;
  }

  if (nLandmarksSeen == 0 && !bufTargetObservations_[robotNumber].found)
  {
    // Randomize a bit for this robot since it does not see landmarks and target
    // isn't seen
    boost::random::uniform_real_distribution<> randPar(-0.05, 0.05);

    for (uint p = 0; p < nParticles_; ++p)
    {
      for (uint s = 0; s < nStatesPerRobot_; ++s)
        particles_[robot_offset + s][p] += randPar(seed_);
    }
  }

  // If this is the main robot, perform one PF-UCLT iteration
  if (mainRobotID_ == robotNumber)
  {
    // Lock mutex
    boost::mutex::scoped_lock(mutex_);

    // All the PF-UCLT steps
    predictTarget();
    fuseRobots();
    fuseTarget();
    resample();
    estimate();

    ROS_INFO("(WALL TIME) Odometry analyzed with = %fms",
             1e3 * odometryTime_.diff);

    deltaIteration_ = ros::WallTime::now() - iterationEvalTime_;
    if (deltaIteration_ > maxDeltaIteration_)
      maxDeltaIteration_ = deltaIteration_;

    durationSum += deltaIteration_;
    numberIterations++;

    ROS_INFO_STREAM("(WALL TIME) Iteration time: "
                    << 1e-6 * deltaIteration_.toNSec() << "ms ::: Worst case: "
                    << 1e-6 * maxDeltaIteration_.toNSec() << "ms ::: Average: "
                    << 1e-6 * (durationSum.toNSec() / numberIterations) << "ms");

    // ROS_DEBUG("Iteration: %s", iteration_oss->str().c_str());
    // Clear ostringstream
    iteration_oss->str("");
    iteration_oss->clear();

    // Start next iteration
    nextIteration();
  }
}

void ParticleFilter::saveAllLandmarkMeasurementsDone(const uint robotNumber)
{
  *iteration_oss << "allLandmarks(OMNI" << robotNumber + 1 << ") -> ";
}

void ParticleFilter::saveAllTargetMeasurementsDone(const uint robotNumber)
{
  *iteration_oss << "allTargets(OMNI" << robotNumber + 1 << ") -> ";
}

ParticleFilter::dynamicVariables_s::dynamicVariables_s(ros::NodeHandle& nh,
                                                       const uint nRobots)
    : firstCallback(true), alpha(nRobots, std::vector<float>(NUM_ALPHAS))
{
  // Get node parameters, assume they exist
  readParam<double>(nh, "percentage_to_keep", resamplingPercentageToKeep);

  readParam<int>(nh, "particles", nParticles);

  readParam<double>(nh, "predict_model_stddev", targetRandStddev);
  oldTargetRandSTddev = targetRandStddev;

  // Get alpha values for some robots (hard-coded for our 4 robots..)
  for (uint r = 0; r < nRobots; ++r)
  {
    std::string paramName =
        "OMNI" + boost::lexical_cast<std::string>(r + 1) + "_alpha";

    std::string str;
    if (readParam<std::string>(nh, paramName, str))
      fill_alpha(r, str); // value was provided
    else
      fill_alpha(r, "0.015,0.1,0.5,0.001"); // default
  }
}

void ParticleFilter::dynamicVariables_s::fill_alpha(const uint robot,
                                                    const std::string& str)
{
  // Tokenize the string of comma-separated values
  std::istringstream iss(str);
  std::string token;
  uint tokenCount = 0;
  while (std::getline(iss, token, ','))
  {
    if (tokenCount >= NUM_ALPHAS)
      break;

    std::istringstream tokss(token);
    float val;
    tokss >> val;

    if (val < 0)
    {
      ROS_WARN("Invalid alpha value %f", val);
      continue;
    }

    alpha[robot][tokenCount] = val;
    ++tokenCount;
  }

  ROS_WARN_COND(tokenCount != NUM_ALPHAS,
                "Number of alpha values provided is not the required number %d",
                NUM_ALPHAS);
}

// end of namespace pfuclt_omni_dataset
}
