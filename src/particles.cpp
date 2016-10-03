#include "particles.h"
#include "ros/ros.h"
#include <stdlib.h>
#include <boost/random.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>
#include <angles/angles.h>

namespace pfuclt_ptcls
{
void ParticleFilter::fuseRobots()
{
  ROS_DEBUG("Fusing Robots");

  state.robotsFused = true;

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

        // TODO should the Y frame be inverted?
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

  // Reset weights of the particle filter
  resetWeights();

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
      int sort_index = sorted[p];
      particles_[o_robot + O_X][p] = dupParticles[o_robot + O_X][sort_index];
      particles_[o_robot + O_Y][p] = dupParticles[o_robot + O_Y][sort_index];
      particles_[o_robot + O_THETA][p] =
          dupParticles[o_robot + O_THETA][sort_index];

      // Update the particle weight (will get multiplied nRobots times and get a
      // lower value)
      particles_[WEIGHT_INDEX][p] *= weightComponents_[r][sort_index];
    }
  }

  // Debugging
  // for(uint p=0; p<nParticles_/5; ++p)
  //  ROS_DEBUG("pWeight[%d] = %f", p, particles_[WEIGHT_INDEX][p]);

  // Change state
  state.robotsFused = true;

  // If the target measurements were performed prior to this function ending,
  // then we should call the target fusion step here
  if (!state.targetFused && state.allTargetMeasurementsDone())
    fuseTarget();
}

void ParticleFilter::fuseTarget()
{
  ROS_DEBUG("Fusing Target");

  state.targetFused = true;

  // If ball not seen by anyone, just skip all of this
  bool ballSeen = false;
  for(std::vector<TargetObservation>::iterator it = bufTargetObservations_.begin(); it != bufTargetObservations_.end(); ++it)
    ballSeen = ballSeen || it->found;

  if(!ballSeen)
    goto exitFuseTarget;

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
        Eigen::Transform<float, 2, Eigen::Affine> toGlobal(Eigen::Rotation2Df(
                                                             particles_[o_robot + O_THETA][m])); // Affine creates a (Dim+1) *
        // (Dim+1) matrix and sets last
        // row to [0 0 ... 1]
        Eigen::Vector3f Srobot(particles_[o_robot + O_X][m],
            particles_[o_robot + O_Y][m], 0);
        Eigen::Vector3f Zglobal = Srobot + toGlobal * Zrobot;

        // Error in observation
        Eigen::Vector3f Target(particles_[O_TARGET + O_X][p],
            particles_[O_TARGET + O_Y][p],
            particles_[O_TARGET + O_Z][p]);

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
      float totalWeight = std::accumulate(probabilities.begin(), probabilities.end(), 0.0);

      // If the weight is the maximum as of now, update the maximum and set particle p as mStar
      if(totalWeight > maxTargetSubParticleWeight)
      {
        // Swap particle m with m* so that the most relevant (in terms of weight)
        // target subparticle is at the lowest indexes
        maxTargetSubParticleWeight = totalWeight;
        mStar = p;
      }
    }

    // Particle m* has been found, let's swap the subparticles
    for(uint i=O_X; i<O_Z; ++i)
      std::swap(particles_[O_TARGET + i][m], particles_[O_TARGET + i][mStar]);

    // Update the weight of this particle
    particles_[WEIGHT_INDEX][m] *= maxTargetSubParticleWeight;
  }

  // The target subparticles are now reordered according to their weight contribution

exitFuseTarget:
  // All that's missing is the resampling step
  resample();
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

ParticleFilter::ParticleFilter(const uint nParticles, const uint nTargets,
                               const uint statesPerRobot, const uint nRobots,
                               const uint nLandmarks,
                               const std::vector<bool>& robotsUsed,
                               const std::vector<Landmark>& landmarksMap,
                               const std::vector<float> alpha)
  : nParticles_(nParticles), nTargets_(nTargets),
    nStatesPerRobot_(statesPerRobot), nRobots_(nRobots),
    nSubParticleSets_(nTargets * STATES_PER_TARGET +
                      nRobots * statesPerRobot + 1),
    nLandmarks_(nLandmarks),
    particles_(nSubParticleSets_, subparticles_t(nParticles)), seed_(time(0)),
    initialized_(false), state(nRobots), alpha_(alpha),
    bufLandmarkObservations_(nRobots,
                             std::vector<LandmarkObservation>(nLandmarks)),
    bufTargetObservations_(nRobots), iterationTimeS(TIME_NOTAVAILABLE),
    robotsUsed_(robotsUsed), landmarksMap_(landmarksMap),
    weightComponents_(nRobots, subparticles_t(nParticles, 0.0))
{

  // If vector alpha is not provided, use a default one
  if (alpha_.empty())
  {
    for (int r = 0; r < nRobots; ++r)
    {
      alpha_.push_back(0.015);
      alpha_.push_back(0.1);
      alpha_.push_back(0.5);
      alpha_.push_back(0.001);
    }
  }

  // Check size of vector alpha
  if (alpha_.size() != 4 * nRobots)
  {
    ROS_ERROR("The provided vector alpha is not of the correct size. Returning "
              "without particle filter! (should have %d=nRobots*4 elements)",
              nRobots * 4);
    return;
  }

  ROS_INFO("Created particle filter with dimensions %d, %d",
           (int)particles_.size(), (int)particles_[0].size());
}

ParticleFilter::ParticleFilter(const ParticleFilter& other)
  : nParticles_(other.nParticles_), nTargets_(other.nTargets_),
    nLandmarks_(other.nLandmarks_),
    nSubParticleSets_(other.nSubParticleSets_),
    nStatesPerRobot_(other.nStatesPerRobot_), nRobots_(other.nRobots_),
    particles_(other.nSubParticleSets_, subparticles_t(other.nParticles_)),
    seed_(time(0)), initialized_(other.initialized_),
    bufLandmarkObservations_(
      other.nRobots_, std::vector<LandmarkObservation>(other.nLandmarks_)),
    targetMotionState(other.targetMotionState), state(other.state),
    robotsUsed_(other.robotsUsed_), landmarksMap_(other.landmarksMap_),
    weightComponents_(other.weightComponents_)
{
  ROS_DEBUG("Creating copy of another pf");

  // Only thing left is to copy particle values
  particles_ = other.particles_; // std::vector assignment operator! yay C++
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
    BOOST_FOREACH (pdata_t& val, particles_[i])
    {
      val = dist(seed_);
    }
  }

  // Particle weights init with same weight (1/nParticles)
  resetWeights();

  // Set flag
  initialized_ = true;

  ROS_INFO("Particle filter initialized");
}

void ParticleFilter::predict(const uint robotNumber, const Odometry odom)
{
  if (!initialized_)
    return;

  using namespace boost::random;

  ROS_DEBUG("Predicting for Robot %d", robotNumber + 1);

  // Variables concerning this robot specifically
  int robot_offset = robotNumber * nStatesPerRobot_;
  float alpha[4] = { alpha_[robotNumber * 4], alpha_[robotNumber * 4 + 1],
                     alpha_[robotNumber * 4 + 2], alpha_[robotNumber * 4 + 3] };

  // TODO find out if this model is correct
  // Determining the propagation of the robot state through odometry
  float deltaRot = atan2(odom.y, odom.x) - odom.theta;
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

  // Debugging
  ROS_DEBUG("Before prediction: p[%d] = [%f %f %f]", nParticles_ / 2,
            particles_[O_X + robot_offset][nParticles_ / 2],
      particles_[O_Y + robot_offset][nParticles_ / 2],
      particles_[O_THETA + robot_offset][nParticles_ / 2]);
  ROS_DEBUG("Odometry = [%f %f %f]", odom.x, odom.y, odom.theta);

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

  // Debugging
  ROS_DEBUG("After prediction: p[%d] = [%f %f %f]", nParticles_ / 2,
            particles_[O_X + robot_offset][nParticles_ / 2],
      particles_[O_Y + robot_offset][nParticles_ / 2],
      particles_[O_THETA + robot_offset][nParticles_ / 2]);

  // Change predicted state
  state.predicted[robotNumber] = true;

  // If all robots have predicted, also predict the target state
  if (targetMotionState.started && state.allPredicted())
  {
    ROS_DEBUG("Robot%d is predicting the target state.", robotNumber);

    // Random acceleration model
    normal_distribution<> targetAcceleration(TARGET_RAND_MEAN,
                                             TARGET_RAND_STDDEV);

    for (int i = 0; i < nParticles_; i++)
    {
      // delta = v*dt + 0.5*a*dt^2 with a new random acceleration for each
      // component
      particles_[O_TARGET + O_TX][i] += targetMotionState.Vx * iterationTimeS *
          0.5 * targetAcceleration(seed_) *
          pow(iterationTimeS, 2);
      particles_[O_TARGET + O_TY][i] += targetMotionState.Vy * iterationTimeS *
          0.5 * targetAcceleration(seed_) *
          pow(iterationTimeS, 2);
      particles_[O_TARGET + O_TZ][i] += targetMotionState.Vz * iterationTimeS *
          0.5 * targetAcceleration(seed_) *
          pow(iterationTimeS, 2);
    }
  }

  // Start fusing if all robots have done their measurements and predictions
  if (!state.robotsFused && state.allLandmarkMeasurementsDone() &&
      state.allPredicted())
    fuseRobots();
}

void ParticleFilter::saveAllLandmarkMeasurementsDone(const uint robotNumber)
{
  // Change state
  state.landmarkMeasurementsDone[robotNumber] = true;

  // Start fusing if all robots have done their measurements and predictions
  if (!state.robotsFused && state.allLandmarkMeasurementsDone() &&
      state.allPredicted())
    fuseRobots();
}

void ParticleFilter::saveAllTargetMeasurementsDone(const uint robotNumber)
{
  // Change state
  state.targetMeasurementsDone[robotNumber] = true;

  // Start fusing if all robots have done their target measurements and the
  // robot fusion step has been performed
  if (!state.targetFused && state.robotsFused &&
      state.allTargetMeasurementsDone())
    fuseTarget();
}

// end of namespace pfuclt_ptcls
}
