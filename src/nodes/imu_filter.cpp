/**
 * @file
 * @author Joan Pau Beltran
 * @brief ROS Memsense IMU filter implementation.
 */

#include "imu_filter.h"
#include <algorithm>

/**
 * Default constructor (doing nothing).
 * @return
 */
memsense_imu::Filter::Filter()
: count_(0)
{}

/**
 * Get the number of samples accumulated by the filter.
 * @return number of samples in internal buffer.
 */
unsigned int memsense_imu::Filter::count()
{
  return count_;
}


/**
 * @brief Add a set of samples to the filter.
 * @param s array of new samples to add to the internal buffer.
 */
void memsense_imu::Filter::update(const SampleArray& s)
{
  for(int i=0; i<NUM_MAGNS; i++)
    for (int j=s[i].size()-1; j>=0; j--)
      samples_[i][j].push_back(s[i][j]);
  count_++;
}


/**
 * @brief Clear the internal buffer.
 */
void memsense_imu::Filter::reset()
{
  for (int i=0; i<NUM_MAGNS; i++)
    for(int j=0; j<NUM_AXES; j++)
      samples_[i][j].clear();
  count_ = 0;
}


/**
 * @brief Get the mean of the samples currently stored by the buffer.
 * @param s array of samples to be filled with the mean values.
 */
void memsense_imu::Filter::mean(SampleArray* s)
{
  for (int i=0; i<NUM_MAGNS; i++)
  {
    (*s)[i].clear();
    for (int j=0; j<NUM_AXES; j++)
    {
      const int num = samples_[i][j].size();
      if(num>0)
      {
        double mean = 0.0;
        for (int k=num-1; k>=0; k--)
          mean += samples_[i][j][k];
        mean /= num;
        (*s)[i].push_back(mean);
      }
    }
  }
}

/**
 * @brief Get the median of the samples currently stored by the buffer.
 * @param s array of samples to be filled with the median values.
 */
void memsense_imu::Filter::median(SampleArray* s)
{
  for (int i=0; i<NUM_MAGNS; i++)
  {
    (*s)[i].clear();
    for (int j=0; j<NUM_AXES; j++)
    {
      const int num = samples_[i][j].size();
      if(num>0)
      {
        std::sort(samples_[i][j].begin(),samples_[i][j].end());
	const int k = num/2;
        double median = samples_[i][j][k];
        if ( (num%2) == 0 )
        {
          median += samples_[i][j][k-1];
          median *= 0.5;
        }
        (*s)[i].push_back(median);
      }
    }
  }
}
