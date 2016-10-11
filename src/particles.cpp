#include "particles.h"
#include "ros/ros.h"
#include <stdlib.h>
#include <boost/random.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>
#include <angles/angles.h>

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
      state_(data.nRobots, data.robotsUsed)
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

  for (int i = 0; i < nParticles_; i++)
  {
    // TODO what exactly should happen to ball velocity?

    // v = a*dt
    state_.target.vx += targetAcceleration(seed_) * iterationTime_;
    state_.target.vy += targetAcceleration(seed_) * iterationTime_;
    state_.target.vz += targetAcceleration(seed_) * iterationTime_;

    // x = v*dt + 0.5*a*dt^2 with a new random acceleration for each
    // component

    particles_[O_TARGET + O_TX][i] +=
        state_.target.vx * iterationTime_ +
        0.5 * targetAcceleration(seed_) * pow(iterationTime_, 2);
    particles_[O_TARGET + O_TY][i] +=
        state_.target.vy * iterationTime_ +
        0.5 * targetAcceleration(seed_) * pow(iterationTime_, 2);
    particles_[O_TARGET + O_TZ][i] +=
        state_.target.vz * iterationTime_ +
        0.5 * targetAcceleration(seed_) * pow(iterationTime_, 2);

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
    std::vector<float> probabilities(nRobots_, 1.0);
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
        Eigen::Vector2f Zrobot(m.x, m.y);

        // Transformation to global frame
        Eigen::Rotation2Df Rrobot(particles_[o_robot + O_THETA][p]);
        Eigen::Vector2f Srobot(particles_[o_robot + O_X][p],
                               particles_[o_robot + O_Y][p]);
        Eigen::Vector2f Zglobal = Srobot + Rrobot * Zrobot;

        // Error in observation
        Eigen::Vector2f LM(landmarksMap_[l].x, landmarksMap_[l].y);

        Eigen::Vector2f Zglobal_err = LM - Zglobal;
        Eigen::Vector2f Z_Zcap = Zrobot - Zglobal_err;

        ROS_DEBUG_COND(p == 0, "Error in seeing landmark %d = (%f , %f)", l,
                       Zglobal_err[O_X], Zglobal_err[O_Y]);
        ROS_DEBUG_COND(p == 0,
                       "Landmark %d is at (%f , %f) and I see it at (%f, %f)",
                       l, LM[O_X], LM[O_Y], Zglobal[O_X], Zglobal[O_Y]);

        // The values of interest to the particle weights
        // Note: using Eigen wasn't of particular interest here since it does
        // not allow for transposing a non-dynamic matrix
        float expArg = -0.5 * (Z_Zcap[0] * Z_Zcap[0] / m.covXX +
                               Z_Zcap[1] * Z_Zcap[1] / m.covYY);
        float detValue = 1.0; // powf( (2*PI*Q[0][0]*Q[1][1]),-0.5);

        probabilities[r] *= detValue * exp(expArg);
        landmarksUsed[r]++;
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
      particles_[o_robot + O_X][p] = dupParticles[o_robot + O_X][sort_index];
      particles_[o_robot + O_Y][p] = dupParticles[o_robot + O_Y][sort_index];
      particles_[o_robot + O_THETA][p] =
          dupParticles[o_robot + O_THETA][sort_index];

      // Update the particle weight (will get multiplied nRobots times and get a
      // lower value)
      particles_[O_WEIGHT][p] *= weightComponents_[r][sort_index];
    }
  }

  // Debugging
  // for(uint p=0; p<nParticles_/5; ++p)
  //  ROS_DEBUG("pWeight[%d] = %f", p, particles_[WEIGHT_INDEX][p]);

  // Change state
  state_.robotsFused = true;

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
    float maxTargetSubParticleWeight = 0.0;
    uint mStar = 0;

    // Find the particle m* in the set [m:M] for which the weight contribution
    // by the target subparticle to the full weight is maximum
    for (uint p = 0; p < nParticles_; ++p)
    {
      std::vector<float> probabilities(nRobots_, 1.0);

      // Observations of the target by all robots
      for (uint r = 0; r < nRobots_; ++r)
      {
        if (false == robotsUsed_[r] || false == bufTargetObservations_[r].found)
          continue;

        uint o_robot = r * nStatesPerRobot_;

        TargetObservation& obs = bufTargetObservations_[r];

        // Observation in robot frame
        Eigen::Vector3f Zrobot(obs.x, obs.y, obs.z);

        // Transformation to global frame
        // Affine creates a (Dim+1) * (Dim+1) matrix and sets
        // last row to [0 0 ... 1]
        Eigen::Transform<float, 2, Eigen::Affine> toGlobal(
            Eigen::Rotation2Df(particles_[o_robot + O_THETA][m]));
        Eigen::Vector3f Srobot(particles_[o_robot + O_X][m],
                               particles_[o_robot + O_Y][m], 0.0);
        Eigen::Vector3f Zglobal = Srobot + toGlobal * Zrobot;

        // Error in observation
        Eigen::Vector3f Target(particles_[O_TARGET + O_TX][p],
                               particles_[O_TARGET + O_TY][p],
                               particles_[O_TARGET + O_TZ][p]);

        // TODO should the Y frame be inverted?
        Eigen::Vector3f Z_Zcap = Zrobot - (Target - Zglobal);

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
      float totalWeight =
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

  // All that's missing is the resampling step
  resample();
}

void ParticleFilter::low_variance_resampler(const float weightSum)
{
  // Low variance resampling exactly as implemented in the book Probabilistic
  // robotics by thrun and burgard

  subparticles_t normalizedWeights(particles_[O_WEIGHT]);
  // Normalize the weights
  for (uint i = 0; i < nParticles_; ++i)
    normalizedWeights[i] = normalizedWeights[i] / weightSum;

  // Duplicate the particle set
  particles_t duplicate(particles_);

  // Generate a random number from a gaussian N~(0; 1/nParticles)
  pdata_t MInv = 1.0 / nParticles_;
  boost::random::normal_distribution<> genR(0.0, MInv);
  pdata_t r = genR(seed_);

  // c is equal to the first particle's weight
  pdata_t c = normalizedWeights[0];

  // Particle iterators
  uint i = 0, m;

  // Iterate over the particle set
  for (m = 0; m < nParticles_; ++m)
  {
    // Generate number from the random sample
    pdata_t u = r + m * MInv;

    // Add weight until it reaches u
    while (u > c)
      c += normalizedWeights[++i];

    // Check if i is off limits
    if (i >= nParticles_)
      break;

    // Add this particle to the set
    for (uint s = 0; s < nSubParticleSets_; ++s)
      particles_[s][m] = duplicate[s][i];
  }

  ROS_DEBUG("End of low_variance_resampler()");

  ROS_WARN_COND(m != nParticles_, "The resampler didn't add the "
                                  "required number of particles, from %d to "
                                  "%d the set was left unchanged!",
                m, nParticles_ - 1);
}

void ParticleFilter::myResampler(const float weightSum)
{
  // Implementing a very basic resampler... a particle gets selected
  // proportional to its weight and 50% of the top particles are kept

  subparticles_t normalizedWeights(particles_[O_WEIGHT]);
  // Normalize the weights
  for (uint i = 0; i < nParticles_; ++i)
    normalizedWeights[i] = normalizedWeights[i] / weightSum;

  particles_t duplicate(particles_);

  std::vector<pdata_t> cumulativeWeights(nParticles_);
  cumulativeWeights[0] = normalizedWeights[0];

  for (int par = 1; par < nParticles_; par++)
  {
    cumulativeWeights[par] =
        cumulativeWeights[par - 1] + normalizedWeights[par];
  }

  int halfParticles = nParticles_ / 2;

  // Keep top 50% of particles
  // Taken care of when duplicating, above
  /*
  for (int par = 0; par < halfParticles; par++)
  {
    for (int k = 0; k < nSubParticleSets_; k++)
    {
      particles_[k][par] = duplicate[k][par];
    }
  }
  */

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

  ROS_DEBUG("End of myResampler()");
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
      continue ;

    uint o_robot = r * nStatesPerRobot_;

    pdata_t stdX = pfuclt_aux::calc_stdDev<pdata_t>(particles_[o_robot + O_X]);
    pdata_t stdY = pfuclt_aux::calc_stdDev<pdata_t>(particles_[o_robot + O_Y]);
    pdata_t stdTheta =
        pfuclt_aux::calc_stdDev<pdata_t>(particles_[o_robot + O_THETA]);

    state_.robots[r].conf = stdX + stdY + stdTheta;

    ROS_DEBUG("OMNI%d stdX = %f, stdY = %f, stdTheta = %f", r+1, stdX, stdY, stdTheta);
  }

  // Calc. sum of weights
  float weightSum = std::accumulate(particles_[O_WEIGHT].begin(),
                                    particles_[O_WEIGHT].end(), 0.0);

  ROS_DEBUG("WeightSum before resampling = %f", weightSum);

  printWeights("before resampling: ");

  if (weightSum < 0.001)
  {
    ROS_ERROR("Zero weightsum - returning without resampling");

    // Print iteration and state information
    *iteration_oss << "FAIL!";
    ROS_DEBUG("Iteration: %s", iteration_oss->str().c_str());

    state_.print();

    // Clear ostringstream
    iteration_oss->str("");
    iteration_oss->clear();

    // Start next iteration
    nextIteration();

    // Exit
    return;
  }

  //low_variance_resampler(weightSum);
  myResampler(weightSum);

  printWeights("after resampling: ");

  // Resampling done, find the current state belief
  weightSum = std::accumulate(particles_[O_WEIGHT].begin(),
                              particles_[O_WEIGHT].end(), 0.0);

  ROS_DEBUG("WeightSum after resampling = %f", weightSum);

  // A vector of weighted means that will be calculated in the following nested
  // loops
  std::vector<double> weightedMeans(nStatesPerRobot_, 0);

  // Target weighted means
  std::vector<double> targetWeightedMeans(STATES_PER_TARGET, 0);

  /// TODO change this to a much more stable way of finding the mean of the
  /// cluster which will represent the ball position. Because it's not the
  /// highest weight particle which represent's the ball position

  // For each robot
  for (uint r = 0; r < nRobots_; ++r)
  {
    // If the robot isn't playing, skip it
    if( false == robotsUsed_[r])
      continue;

    uint o_robot = r * nStatesPerRobot_;

    // ..and each particle
    for (uint p = 0; p < nParticles_; ++p)
    {
      // Accumulate the state proportionally to the particle's weight
      for (uint g = 0; g < nStatesPerRobot_; ++g)
      {
        weightedMeans[g] +=
            particles_[o_robot + g][p] * particles_[O_WEIGHT][p];
      }

      for (uint t = 0; t < STATES_PER_TARGET; ++t)
      {
        targetWeightedMeans[t] +=
            particles_[O_TARGET + t][p] * particles_[O_WEIGHT][p];
      }
    }

    // Normalize the means
    for (uint g = 0; g < nStatesPerRobot_; ++g)
      weightedMeans[g] = weightedMeans[g] / weightSum;

    for (uint t = 0; t < STATES_PER_TARGET; ++t)
      targetWeightedMeans[t] = targetWeightedMeans[t] / weightSum;

    // Save in the robot state
    state_.robots[r].x = weightedMeans[O_X];
    state_.robots[r].y = weightedMeans[O_Y];
    state_.robots[r].theta = weightedMeans[O_THETA];

    // Save in the target state
    // First the velocity, then the position
    /*
    state_.target.vx =
        (state_.target.x - targetWeightedMeans[O_TX]) / iterationTime_;
    state_.target.x = targetWeightedMeans[O_TX];
    state_.target.vy =
        state_.target.y - targetWeightedMeans[O_TY] / iterationTime_;
    state_.target.y = targetWeightedMeans[O_TY];
    state_.target.vz =
        state_.target.z - targetWeightedMeans[O_TZ] / iterationTime_;
    state_.target.z = targetWeightedMeans[O_TZ];
    */

    //Velocities stay the same

    //Update position
    state_.target.x = targetWeightedMeans[O_TX];
    state_.target.y = targetWeightedMeans[O_TY];
    state_.target.z = targetWeightedMeans[O_TZ];
  }

  // Debug
  ROS_DEBUG("PF says OMNI4 at (x,y) = [ %f, %f ]", state_.robots[3].x,
            state_.robots[3].y);

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

  ROS_DEBUG_COND(custom.size() != ((nSubParticleSets_ - 1) * 2),
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

  ROS_DEBUG("Predicting for OMNI%d", robotNumber + 1);

  // Variables concerning this robot specifically
  int robot_offset = robotNumber * nStatesPerRobot_;
  float alpha[4] = { alpha_[robotNumber * 4 + 0], alpha_[robotNumber * 4 + 1],
                     alpha_[robotNumber * 4 + 2], alpha_[robotNumber * 4 + 3] };

  // Determining the propagation of the robot state through odometry
  float deltaRot =
      atan2(odom.y, odom.x) -
      state_.robots[robotNumber]
          .theta; // Uses the previous overall belief of orientation
  float deltaTrans = sqrt(odom.x * odom.x + odom.y * odom.y);
  float deltaFinalRot = odom.theta - deltaRot;

  // Create an error model based on a gaussian distribution
  normal_distribution<> deltaRotEffective(deltaRot, alpha[0] * fabs(deltaRot) +
                                                        alpha[1] * deltaTrans);
  normal_distribution<> deltaTransEffective(
      deltaTrans,
      alpha[2] * deltaTrans + alpha[3] * fabs(deltaRot + deltaFinalRot));
  normal_distribution<> deltaFinalRotEffective(
      deltaFinalRot, alpha[0] * fabs(deltaFinalRot) + alpha[1] * deltaTrans);

  for (int i = 0; i < nParticles_; i++)
  {
    // Rotate to final position
    particles_[O_THETA + robot_offset][i] += deltaRotEffective(seed_);

    // Translate to final position
    particles_[O_X + robot_offset][i] +=
        deltaTransEffective(seed_) * cos(particles_[O_THETA + robot_offset][i]);
    particles_[O_Y + robot_offset][i] +=
        deltaTransEffective(seed_) * sin(particles_[O_THETA + robot_offset][i]);

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
    : ParticleFilter(data), pubData(publishData)
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
      pubData.publishNamespace + "/gtData_synced_pfuclt_estimate", 1000);

  // Other publishers
  robotStatePublisher_ = pubData.nh.advertise<read_omni_dataset::RobotState>(
      pubData.publishNamespace + "/pfuclt_omni_poses", 1000);

  targetStatePublisher_ = pubData.nh.advertise<read_omni_dataset::BallData>(
      pubData.publishNamespace + "/pfuclt_orangeBallState", 1000);

  particlePublisher_ = pubData.nh.advertise<pfuclt_omni_dataset::particles>(
      pubData.publishNamespace + "/pfuclt_particles", 10);

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
}

void PFPublisher::publishRobotStates()
{
  // This is pretty much copy and paste
  for (uint r = 0; r < nRobots_; ++r)
  {
    ParticleFilter::State::robotState_s& pfState = state_.robots[r];
    geometry_msgs::PoseWithCovariance& rosState = msg_state_.robotPose[r].pose;

    rosState.pose.position.x = pfState.x;
    rosState.pose.position.y = pfState.y;
    rosState.pose.position.z = pubData.robotHeight;

    rosState.pose.orientation.x = 0;
    rosState.pose.orientation.y = 0;
    rosState.pose.orientation.z = sin(pfState.theta / 2);
    rosState.pose.orientation.w = cos(pfState.theta / 2);
  }

  robotStatePublisher_.publish(msg_state_);
}

void PFPublisher::publishTargetState()
{
  msg_target_.x = state_.target.x;
  msg_target_.y = state_.target.y;
  msg_target_.z = state_.target.z;

  targetStatePublisher_.publish(msg_target_);
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
  syncedGTPublisher_.publish(msg_GT_);

  // Call the base class method
  ParticleFilter::nextIteration();
}

// end of namespace pfuclt_ptcls
}
