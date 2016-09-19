#ifndef _PFUCLT_AUX_H_
#define _PFUCLT_AUX_H_

#include <ros/ros.h>
#include <vector>
#include "pfuclt_aux.h"
#include <boost/ref.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

namespace pfuclt_aux
{
/**
 * @brief The ORDER_TYPE enum - use to define an ascending or descending
 * ordering
 */
enum ORDER_TYPE
{
  ASC,
  DESC
};

/**
  * @brief calc_stdDev - Calculates standard deviation from a vector of type T
  * @param vec - vector of type T with values to calculate std.dev
  * @return standard deviation as a double
  */
template <typename T> double calc_stdDev(std::vector<T>* vec)
{
  using namespace boost::accumulators;

  accumulator_set<T, stats<tag::variance> > acc;
  for_each(vec->begin(), vec->end(), boost::bind<void>(boost::ref(acc), _1));
  return (double)sqrt(extract::variance(acc));
}

/**
 * @brief order_index - Generates a vector of indexes ordered according to
 * another vector
 * @param values - vector of type T containing the values to order
 * @param order - enum of ORDER_TYPE to order in ascending or descending order -
 * defaults to DESC
 * @return vector of unsigned ints with the desired ordering of indexes
 * @remark vector values will not be modified
 */
template <typename T>
std::vector<unsigned int> order_index(std::vector<T> const& values,
                                      ORDER_TYPE order = pfuclt_aux::DESC)
{
  // from http://stackoverflow.com/a/10585614 and modified
  // return sorted indices of vector values

  using namespace boost::phoenix;
  using namespace boost::phoenix::arg_names;

  std::vector<unsigned int> indices(values.size());
  int i = 0;
  std::transform(values.begin(), values.end(), indices.begin(), ref(i)++);
  if (order == pfuclt_aux::DESC)
  {
    std::sort(indices.begin(), indices.end(),
              ref(values)[arg1] > ref(values)[arg2]);
  }
  else // ASC
  {
    std::sort(indices.begin(), indices.end(),
              ref(values)[arg1] < ref(values)[arg2]);
  }

  return indices;
}

/**
 * @brief readParamDouble - reads and returns a parameter from the ROS parameter
 * server
 * @param nh - pointer to the ROS nodehandle that will perform the parameter
 * reading task
 * @param name - the parameter's name
 * @param variable - pointer to the location where the parameter should be
 * stored as type T
 * @return
 */
template <typename T>
bool readParamDouble(ros::NodeHandle* nh, const std::string name, T* variable)
{
  double tmp;
  std::ostringstream oss;
  if (nh->getParam(name, tmp))
  {
    *variable = (T)tmp;
    oss << "Received parameter " << name << "=" << *variable;
    ROS_INFO("%s", oss.str().c_str());
    return true;
  }
  else
    ROS_ERROR("Failed to receive parameter %s", name.c_str());

  return false;
}

// end of namespace
}

#endif
