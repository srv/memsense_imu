/** @file
 *
 * @author Joan Pau Beltran
 * @brief ROS Memsense IMU filter presentation.
 *
 * This is a filter to be used with the Memsense IMU node.
 * IMU samples are stored in vectors, and filtered with the desired method
 * on demand.
 */

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "imu_sample.h"

namespace memsense_imu
{

/** IMU sample filter.
 *
 * Filter to be used with the Memsense IMU node.
 * IMU samples are stored in vectors, and filtered with the desired method
 * on demand.
 */
class Filter
{
public:

  Filter();

  unsigned int count();

  void reset();

  void update(const SampleArray& s);

  void mean(SampleArray& s);

private:

  unsigned int count_;
  std::vector<double> samples_[NUM_MAGNS][NUM_AXES];

};

} // namespace

#endif /* IMU_FILTER_H */
