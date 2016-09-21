#include "particles.h"
#include "ros/ros.h"
#include <stdlib.h>

namespace pfuclt_ptcls
{

particles::particles(int nParticles, int nDimensions)
    : nParticles_(nParticles), nDimensions_(nDimensions),
      subparts_(nDimensions, subparticles_t(nParticles))
{
  int size[2];
  size[0]=(int) subparts_.size();
  size[1]=(int) subparts_[0].size();

  ROS_INFO("Created particle set with dimensions %d, %d", size[0], size[1]);
}

// end of namespace pfuclt_ptcls
}
