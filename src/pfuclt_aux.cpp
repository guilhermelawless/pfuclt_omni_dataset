#include "pfuclt_aux.h"
#include "minicsv.h"
#include <iostream>

namespace pfuclt_aux
{
std::vector<Landmark> getLandmarks(const char* filename)
{
  std::vector<Landmark> lm_vec;
  mini::csv::ifstream is(filename);

  if (is.is_open())
  {
    Landmark lm;
    while (is.read_line())
    {
      //order is serial,x,y\n
      is >> lm.serial >> lm.x >> lm.y;
      lm_vec.push_back(lm);
    }
  }

  is.close();
  return lm_vec;
}

//end of namespace
}
