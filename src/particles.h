#ifndef PARTICLES_H
#define PARTICLES_H

#include <vector>
#include <algorithm>
#include <iostream>

namespace pfuclt_ptcls
{
typedef std::vector<float> subparticles_t;
typedef std::vector<subparticles_t> particles_t;

class particles
{
private:
  int nParticles_;
  int nDimensions_;
  particles_t subparts_;

public:
  particles(int nParticles, int nDimensions);

  // array subscripting operator
  subparticles_t operator[](int index) { return subparts_[index]; }

  // implement const overloaded operator for a const object
  const subparticles_t operator[](int index) const { return subparts_[index]; }

  // assignment operator
  // particles operator=(const particles& copyThis) { copyThis.subparts_.; }
};

// end of namespace pfuclt_ptcls
}
#endif // PARTICLES_H
