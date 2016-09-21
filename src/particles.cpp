#include "particles.h"
#include "ros/ros.h"
#include <stdlib.h>
#include <boost/random.hpp>
#include <boost/foreach.hpp>
#include <vector>

namespace pfuclt_ptcls
{

particle_filter::particle_filter(int nParticles, int nDimensions)
    : nParticles_(nParticles), nDimensions_(nDimensions),
      particles_(nDimensions, subparticles_t(nParticles)),
      seed_(time(0))
{
  int size[2];
  size[0]=(int) particles_.size();
  size[1]=(int) particles_[0].size();

  ROS_INFO("Created particle set with dimensions %d, %d", size[0], size[1]);
}

//TODO set different values for position and orientation, targets, etc
// Simple version, use default values - randomize all values between [-10,10]
void particle_filter::init()
{
  boost::random::uniform_real_distribution<> dist(-10, 10);

  //For all subparticle sets except the particle weights
  for(int i=0; i < nDimensions_-1; ++i)
  {
    BOOST_FOREACH(float& val, particles_[i])
    {
      val = dist(seed_);
    }
  }

  //Particle weights init with same weight (1/nParticles)
  particles_[nDimensions_-1].assign(nParticles_, 1 / nParticles_);
}

// Overloaded fucntion when using custom values
void particle_filter::init(const std::vector<double> custom)
{
  //For all subparticle sets except the particle weights
  for(int i=0; i < nDimensions_-1; ++i)
  {
    boost::random::uniform_real_distribution<> dist(custom[2*i], custom[2*i+1]);

    // Sample a value from the uniform distribution for each particle
    BOOST_FOREACH(float& val, particles_[i])
    {
      val = dist(seed_);
    }
  }

  //Particle weights init with same weight (1/nParticles)
  particles_[nDimensions_-1].assign(nParticles_, 1 / nParticles_);
}


// end of namespace pfuclt_ptcls
}
