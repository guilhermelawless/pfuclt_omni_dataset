#ifndef PFUCLT_AUX_H
#define PFUCLT_AUX_H

#include <ros/ros.h>
#include <vector>
#include <fstream>
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
 * @brief The Landmark struct - used to store a landmark, defined by its serial
 * number, and its position {x,y}
 */
typedef struct landmark_s
{
  int serial;
  float x, y;
} Landmark;

/**
 * @brief getLandmarks - read landmark configuration from CSV file
 * @param filename - the CSV file
 * @return lm_vec
 * @remark if filename empty, not found or unreadable, will return empty vector
 */
std::vector<Landmark> getLandmarks(const char* filename);

/**
  * @brief calc_stdDev - Calculates standard deviation from a vector of type T
  * @param vec - vector of type T with values to calculate std.dev
  * @return standard deviation as a double
  */
template <typename T> double calc_stdDev(std::vector<T>* vec)
{
  using namespace boost::accumulators;

  accumulator_set<T, stats<tag::variance> > acc;
  std::for_each(vec->begin(), vec->end(),
                boost::bind<void>(boost::ref(acc), _1));
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
 * @brief readParam - reads and returns a parameter from the ROS parameter
 * server
 * @param nh - pointer to the ROS nodehandle that will perform the parameter
 * reading task
 * @param name - the parameter's name
 * @param variable - pointer to the location where the parameter should be
 * stored as type T
 * @return
 */
template <typename T>
bool readParam(ros::NodeHandle& nh, const std::string name, T& variable)
{
  std::ostringstream oss;
  if (nh.getParam(name, variable))
  {
    oss << "Received parameter " << name << "=" << variable;
    ROS_INFO("%s", oss.str().c_str());
    return true;
  }
  else
    ROS_ERROR("Failed to receive parameter %s", name.c_str());

  return false;
}

/* @brief readParam (vectors) - reads and returns a parameter from the ROS
 * parameter
 * server
 * @param nh - reference to the ROS nodehandle that will perform the parameter
 * reading task
 * @param name - the parameter's name
 * @param variable - reference to the location where the parameter should be
 * stored as type std::vector<T>
 * @remark this function is overloaded from readParam for use with vectors
 * @return
 */
template <typename T>
bool readParam(ros::NodeHandle& nh, const std::string name,
               std::vector<T>& variable)
{
  std::ostringstream oss;
  if (nh.getParam(name, variable))
  {
    oss << "Received parameter " << name << "=[ ";
    for (typename std::vector<T>::iterator it = variable.begin();
         it != variable.end(); ++it)
    {
      if (typeid(T) == typeid(bool))
        oss << std::boolalpha << *it << " ";
      else
        oss << *it << " ";
    }
    oss << "]";

    ROS_INFO("%s", oss.str().c_str());
    return true;
  }
  else
    ROS_ERROR("Failed to receive parameter %s", name.c_str());
}

// end of namespace
}

#endif // PFUCLT_AUX_H
