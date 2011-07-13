/** @file
 *
 * @author Joan Pau Beltran
 * @brief ROS Memsense IMU filter implementation.
 *
 * This is a filter to be used with the Memsense IMU node.
 * IMU samples are stored in vectors, and filtered with the desired method
 * on demand.
 */

#include "imu_filter.h"

memsense_imu::Filter::Filter()
: count_(0)
{}


unsigned int memsense_imu::Filter::count()
{
  return count_;
}


void memsense_imu::Filter::update(const SampleArray& s)
{
  for(int i=0; i<NUM_MAGNS; i++)
    for (int j=s[i].size()-1; j>=0; j++)
      samples_[i][j].push_back(s[i][j]);
  count_++;
}


void memsense_imu::Filter::reset()
{
  for (int i=0; i<NUM_MAGNS; i++)
    for(int j=0; j<NUM_AXES; j++)
      samples_[i][j].clear();
  count_ = 0;
}


void memsense_imu::Filter::mean(SampleArray& s)
{
  for (int i=0; i<NUM_MAGNS; i++)
    for (int j=0; j<NUM_AXES; i++)
    {
      const int count = samples_[i][j].size();
      if(count>0)
      {
        double mean = 0.0;
        for (int k=samples_[i][j].size()-1; k>=0; k--)
          mean += samples_[i][j][k];
        mean /= count;
        s[i].push_back(mean);
      }
    }
}
