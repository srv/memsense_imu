#include "STDIMUSample.h"

#include "IMUMAGPayload.h"
#include "IMUHiGPayload.h"
#include "IMUTempPayload.h"
#include "IIMUPayload.h"
#include "IComm.h"
#include "CommonUtils.h"
#include "SensorDataUtils.h"
#include "ISensorDataTransform.h"
#include <cstddef> // NULL macro

using namespace mems;

const unsigned short STDIMUSample::SENSOR_SIZE_IN_BYTES = 2;
const int STDIMUSample::SENSOR_IDX_UNKNOWN = -1;
const double STDIMUSample::SENSOR_UNAVAILABLE_SENSOR_RANGE = -1;
const int STDIMUSample::USE_DEFAULT_TEMP_SENSORS = -1;

/**
* The constructor takes enough parameters to generalize to any device with
* any number of gyro, accelerometer, magnetometer, hi-G, and temperature
* sensors.
**/
STDIMUSample::STDIMUSample(unsigned short numGyroSensors, double gyroRange,
                           unsigned short numAccelSensors, double accelRange,
                           unsigned short numMagSensors, double magRange,
                           unsigned short numHiGSensors, double hiGRange,
                           unsigned short numTempSensors,
                           ISensorDataTransform* transform)
: m_numGyroSensors(numGyroSensors), m_numAccelSensors(numAccelSensors),
  m_numMagSensors(numMagSensors), m_numHiGSensors(numHiGSensors),
  m_numTempSensors(numTempSensors), m_gyroRange(gyroRange),
  m_accelRange(accelRange), m_magRange(magRange), m_hiGRange(hiGRange),
  m_idxAccel(SENSOR_IDX_UNKNOWN), m_idxMag(SENSOR_IDX_UNKNOWN),
  m_idxHiG(SENSOR_IDX_UNKNOWN), m_idxTemp(SENSOR_IDX_UNKNOWN),
  m_idxChecksum(SENSOR_IDX_UNKNOWN), m_magPayload(NULL), m_higPayload(NULL),
  m_tempPayload(NULL), m_commBuffer(NULL), m_bytes(NULL)
{
   //The transform determines how we process the data before outputting it.
   //If it isn't provided in the constructor, we use the default.
   if( transform == NULL )
      m_transform = new mems::SDTDefault();
   else
      m_transform = transform;

   //This sampler makes the following assumptions:

   //1. Data ordering will always be gyro, accel, mag, hiG, temp, checksum.
   //2. Gyros are always there, and they start at index 13.
   //3. Checksum is size 1.

   //This function creates our payloads
   updateIndicesAndPayloads();
}

STDIMUSample* STDIMUSample::getStandardSample(unsigned short deviceCode,
                                              int packetSize, double gyroRange,
                                              double accelRange,
                                              double hiGRange)
{
   //Get the device type:
   E_DeviceType devType = mems::deviceTypeFromHeaderDeviceID(deviceCode);
   E_DeviceType newDeviceType;

   //Get the number of sensors based upon the device type/packet size:
   int numGyro = 3;
   int numAccel = 3;
   int numMag = 3;
   int numTemp = -1;
   int numHiG = -1;
   double magRange = 1.9;

   switch( devType )
   {
      case UIMU:
         newDeviceType = narrowDownUIMU(packetSize);
      break;

      case UIMU_HIG_3AXES:
         newDeviceType = narrowDownUIMUw3HiG(packetSize);
      break;

      case BLUETOOTH:
      case NIMU:
      case N2IMU:
         newDeviceType = narrowDownNIMU(packetSize);
      break;

      //These devices cannot be handled by the STDIMUHandler.
      case CDG:
      case HARS:
      case xAHRS:
         return NULL;
      break;

      //If invalid, we just try to determine based upon packet size:
      case DT_INVALID:
      case UNKNOWN_IMU:
      default:
          newDeviceType = narrowDownUnknownIMU(packetSize);
      break;
   }

   numTemp = getNumberOfTempSensors(newDeviceType);
   numHiG = getNumberOfHiGAxes(newDeviceType);

   STDIMUSample* retPointer = new STDIMUSample(numGyro, gyroRange,
                                               numAccel, accelRange,
                                               numMag, magRange,
                                               numHiG, hiGRange,
                                               numTemp, NULL);
   return retPointer;

}


STDIMUSample* STDIMUSample::getStandardSample(E_DeviceType deviceType,
                         double gyroRange, double accelRange,
                         double magRange, double hiGRange)
{
   int numGyroSensors = 3;
   int numAccelSensors = 3;
   int numMagSensors = 3;
   int numHiGSensors = getNumberOfHiGAxes(deviceType);
   int numTempSensors = getNumberOfTempSensors(deviceType);

   switch( deviceType )
   {
      //Stuff we can't deal with:
      case CDG:
      case HARS:
      case xAHRS:
      case MG:
      case AR:
      case TR:
      case UNKNOWN_IMU:
      case DT_INVALID:
         return NULL;

      //Otherwise return a new sample:
      default:
         return new STDIMUSample( numGyroSensors, gyroRange, numAccelSensors,
                                  accelRange, numMagSensors, magRange,
                                  numHiGSensors, hiGRange, numTempSensors,
                                  NULL);
   }
}

/**
This function looks at the new number of samples for each sensor, and resets
all of the indices and payloads.
**/
void STDIMUSample::updateIndicesAndPayloads()
{
   //All standard devices cover bytes 1-12 with non-sensor data:
   short currentIndex = 13;

   //Gyro is always first, so it always starts at 13:
   m_idxGyro = currentIndex;

   //Advance the index enough to cover the gyro bytes:
   currentIndex += SENSOR_SIZE_IN_BYTES * m_numGyroSensors;

   //If there are accelerometers, mark where they start and advance the
   //index to cover them:
   if( m_numAccelSensors )
   {
      m_idxAccel = currentIndex;
      currentIndex += SENSOR_SIZE_IN_BYTES * m_numAccelSensors;
   }

   //If there are magnetometers, mark where they start and advance the
   //index to cover them:
   if( m_numMagSensors )
   {
      m_idxMag = currentIndex;
      currentIndex += SENSOR_SIZE_IN_BYTES * m_numMagSensors;
   }

   //If there are HiGs, mark where they start and advance the index to
   //cover them:
   if( m_numHiGSensors )
   {
      m_idxHiG = currentIndex;
      currentIndex += SENSOR_SIZE_IN_BYTES * m_numHiGSensors;
   }

   //If there are temperature samples, mark where they start and advance the
   //index to cover them:
   if( m_numTempSensors )
   {
      m_idxTemp = currentIndex;
      currentIndex += SENSOR_SIZE_IN_BYTES * m_numTempSensors;
   }

   //Wherever we ended must be the checksum byte.
   m_idxChecksum = currentIndex;

   //The sample size is one bigger than the checksum placement:
   m_sampleSize = m_idxChecksum + 1;

   //Delete and recreate our communication buffer because it might be a
   //different size now:
   delete m_commBuffer;
   m_commBuffer = new unsigned char[m_sampleSize];

   //Delete all of the payloads:
   delete m_magPayload;
   m_magPayload = NULL;
	delete m_higPayload;
   m_higPayload = NULL;
	delete m_tempPayload;
   m_tempPayload = NULL;

   //Recreate them now that they maybe be of a different size, holding a
   //different range, or different transform:
   m_magPayload = new mems::IMUMAGPayload(m_gyroRange, m_accelRange, m_magRange,
                                          m_numGyroSensors, m_numAccelSensors,
                                          m_numMagSensors, m_transform);

   m_higPayload = new mems::IMUHiGPayload(m_hiGRange, m_numHiGSensors);

   m_tempPayload = new mems::IMUTempPayload(m_numTempSensors);
}

//Set the gyro range and update the indices/payloads:
void STDIMUSample::setGyroRange(double range)
{
   m_gyroRange = range;
   updateIndicesAndPayloads();
}

//Set the accelerometer range and the indices/payloads:
void STDIMUSample::setAccelRange(double range)
{
   m_accelRange = range;
   updateIndicesAndPayloads();
}

//Set the magnetometer range and update the indices/payloads:
void STDIMUSample::setMagRange(double range)
{
   m_magRange = range;
   updateIndicesAndPayloads();
}

//Set the HiG accel range and update the indices/payloads:
void STDIMUSample::setHiGAccelRange(double range)
{
   m_hiGRange = range;
   updateIndicesAndPayloads();
}

//Set the number of gyro sensors and update the indices/payloads:
void STDIMUSample::setNumberGyroSensors(unsigned short sensors)
{
   m_numGyroSensors = sensors;
   updateIndicesAndPayloads();
}

void STDIMUSample::setNumberAccelSensors(unsigned short sensors)
{
   m_numAccelSensors = sensors;
   updateIndicesAndPayloads();
}

//Set the number the magnetometer samples and update the indices/payloads:
void STDIMUSample::setNumberMagSensors(unsigned short sensors)
{
   m_numMagSensors = sensors;
   updateIndicesAndPayloads();
}

//Update the number of HiG samples and the indices/payloads:
void STDIMUSample::setNumberHiGSensors(unsigned short sensors)
{
   m_numHiGSensors = sensors;
   updateIndicesAndPayloads();
}

//Update the number of temperature samples and the indices/payloads:
void STDIMUSample::setNumberTemperatureSensors(unsigned short sensors)
{
   m_numTempSensors = sensors;
   updateIndicesAndPayloads();
}

//Read a new sample and fill in the our sample with the bytes read.
bool STDIMUSample::readSample(IComm * comm, int & status,
                     const unsigned short findSynchMaxConsecutiveFailures)
{

   unsigned long bytesRead;
   bool success = false;

   /**
   The call to findSynch will read until it finds the 4 0xFFs in a row that
   indicate the beginning of the data packet. The readData that follows it
   asks for (sampleSize - numberOfSynchBytes) because that function leaves
   the data stream at the end of those synch bytes.
   **/
   if(mems::findSynch(comm, m_commBuffer, StandardIMUHeader::NUM_SYNCH_BYTES,
      StandardIMUHeader::SYNCH_BYTE, findSynchMaxConsecutiveFailures))
   {
      //Read data returns whether or not it was able to read enough data.
      success = comm->readData(
                  m_sampleSize - mems::StandardIMUHeader::NUM_SYNCH_BYTES,
                  m_commBuffer + mems::StandardIMUHeader::NUM_SYNCH_BYTES,
                  bytesRead, status);

      //Set the data we read into our sample:
      if(success)
	   {
         bytesRead = m_sampleSize;
		   setSample(m_commBuffer);
	   }

   }

   //Return whether or not we succeeded in reading or not:
	return(success);
}

/**
Set sample sets the bytes we read from the device into our sample. This
means we place sections of the array into the header and payloads.
**/
void STDIMUSample::setSample(unsigned char* sample)
{
   m_bytes = sample;
   m_header.setHeaderAndChecksum(sample, sample[m_idxChecksum]);
	m_magPayload->setPayload(&sample[m_idxGyro]);

   if( m_numHiGSensors )
      m_higPayload->setPayload(&sample[m_idxHiG]);

   if( m_numTempSensors )
      m_tempPayload->setPayload(&sample[m_idxTemp]);
}

/**
This function returns a vector of the characters returned from the latest data
packet.
**/
std::vector<unsigned char> STDIMUSample::getBytes()
{
   std::vector<unsigned char> vec(m_sampleSize);

	for(int i = 0; i < m_sampleSize; ++i)
		vec[i] = m_bytes[i];

	return(vec);
}

/**
Return the header from the latest data packet
**/
IIMUHeader* STDIMUSample::getHeader()
{
   return(&m_header);
}

/**
Generate a clone of this instance of the class, and set the clone's sample
to this instance's current sample.
**/
STDIMUSample* STDIMUSample::clone()
{
   //Just pass all of our class values into the clone's constructor:
   STDIMUSample* retClone = new STDIMUSample(m_numGyroSensors, m_gyroRange,
                              m_numAccelSensors, m_accelRange,
                              m_numMagSensors, m_magRange,
                              m_numHiGSensors, m_hiGRange, m_numTempSensors,
                              m_transform);

   //Set the clone's sample to our current sample.
   retClone->setSample( m_commBuffer );

   return retClone;
}

/**
Return a vector of pointers to all of the payloads:
**/
std::vector<IIMUPayload*> STDIMUSample::getPayload()
{
   std::vector<IIMUPayload*> vec;

   //Always have a mag payload, push it back:
   vec.push_back(m_magPayload);

   //If there are HiG samples, push the payload back:
   if( m_numHiGSensors )
      vec.push_back(m_higPayload);

   //If we have temperatures, push the payload back:
   if( m_numTempSensors )
      vec.push_back(m_tempPayload);

   return vec;
}

//Just return the header's size:
unsigned short STDIMUSample::getHeaderSize()
{
   return(m_header.getSize());
}

//Return the size of the payloads:
unsigned short STDIMUSample::getPayloadSize()
{
   //Always have the mag payload, start with its size:
   unsigned short retSize = m_magPayload->getSize();

   //If we have a hiG payload, add its size:
   if( m_numHiGSensors )
      retSize += m_higPayload->getSize();

   //If we have a temperature payload, add its size:
   if( m_numTempSensors )
      retSize += m_tempPayload->getSize();

   return retSize;
}

//Total size is the header size and the payload size. Return as such:
unsigned short STDIMUSample::getSize()
{
   return getHeaderSize() + getPayloadSize();
}

//Check if the sample had a valid checksum, and return as such:
bool STDIMUSample::isValidChecksum()
{
   //Get the bytes for the whole sample, and the bytes with no checksum:
   std::vector<unsigned char> sampleChecksum(getBytes());
	std::vector<unsigned char> sampleNoChecksum(sampleChecksum.begin(),
		--(sampleChecksum.end()));

   //SensorDataUtils has a function for checking for a valid checksum,
   //return its results:
   return(SensorDataUtils::isValidChecksum(sampleNoChecksum,
		                                     m_header.getChecksum()));
}

void STDIMUSample::setTransform(ISensorDataTransform* transform)
{
   //The transform determine how we interpret the data from the device, as
   //bytes, engineering units, volts, etc. When we are asked to use a new
   //transform, we also need to give that transform to all of our payloads:
   m_transform = transform;
   m_magPayload->setTransform(m_transform);
   m_higPayload->setTransform(m_transform);
   m_tempPayload->setTransform(m_transform);
}

//Simple function to return our currently used data transform:
ISensorDataTransform* STDIMUSample::getTransform()
{
   return m_transform;
}
