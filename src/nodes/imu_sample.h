/** @file
 *
 * @brief ROS Memsense IMU generic driver presentation.
 * @author Joan Pau Beltran
 *
 * Common definitions for the IMU sample format
 * used by the node and the filter.
  */

#ifndef IMU_SAMPLE_H
#define IMU_SAMPLE_H

#include <vector>

namespace memsense_imu
{

// Magnitudes sampled by the IMU
enum Magnitude { MAGN_GYRO=0, MAGN_ACCEL=1, MAGN_MAG=2 };
const int NUM_MAGNS = 3;

// Axes
enum Axe {X_AXIS=0, Y_AXIS=1, Z_AXIS=2};
const int NUM_AXES = 3;

// Sample format
typedef std::vector<double> SampleArray[NUM_MAGNS];
typedef double BiasTable[NUM_MAGNS][NUM_AXES];
// There is only one variance per sensor
typedef double VarianceTable[NUM_MAGNS];

} // namespace

#endif /* IMU_SAMPLE_H */
