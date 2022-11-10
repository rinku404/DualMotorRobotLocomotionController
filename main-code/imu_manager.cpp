#include "imu_manager.h"

#include "src/libs/SparkFun_MPU-9250-DMP/SparkFunMPU9250-DMP.h"

static MPU9250_DMP imu;

static float gyroZ;


void IMUStart(void)
{
    // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {

    }
  }
  
  imu.setSensors(INV_XYZ_GYRO); // Enable gyroscope only
  imu.setGyroFSR(2000); // Set gyro to 2000 dps

  imu.dmpBegin(DMP_FEATURE_GYRO_CAL |   // Enable gyro cal
              DMP_FEATURE_SEND_CAL_GYRO,// Send cal'd gyro values
              10); 
}

void IMUUpdate(void)
{
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      gyroZ = imu.calcGyro(imu.gz);
    }
  }
}

float IMUGetYaw(void)
{
    return gyroZ;
}