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
      // order is serial,x,y\n
      is >> lm.serial >> lm.x >> lm.y;
      lm_vec.push_back(lm);
    }
  }

  is.close();
  return lm_vec;
}

double linearRegressionSlope(const std::vector<double>& x, const std::vector<double>& y)
{
  const size_t size = x.size();
  const double s_y = std::accumulate(y.begin(), y.end(), 0.0);
  const double s_x = std::accumulate(x.begin(), x.end(), 0.0);
  const double s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
  const double s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);

  const double slope = (size*s_xy - s_x*s_y) / (size*s_xx - s_x*s_x);
  return slope;
}

// end of namespace
}
