/* This file is part of the Razor AHRS Firmware */

#include  <PString.h>

char      dataBuffer[100];                                  // ADDED MATTHEW DUNBABIN
PString   dataPkt( dataBuffer, sizeof( dataBuffer ) );      // ADDED MATTHEW DUNBABIN

/* Output angles: yaw, pitch, roll */
/* Modified by Matthew Dunbabin 16 April 2020 to 
   Output Y,P,R,Fx,Fy,Fz,Rz and do proper checksum calculations */ 
void output_angles()
{
     int i;
     
     if (output_format == OUTPUT__FORMAT_BINARY) {
          float ypr[3];  
          ypr[0] = TO_DEG(yaw);
          ypr[1] = TO_DEG(pitch);
          ypr[2] = TO_DEG(roll);
          LOG_PORT.write((byte*) ypr, 12);  // No new-line
     } else if (output_format == OUTPUT__FORMAT_TEXT) {
          // LOG_PORT.print("#YPR,");
          // LOG_PORT.print(TO_DEG(yaw)); LOG_PORT.print(",");
          // LOG_PORT.print(TO_DEG(pitch)); LOG_PORT.print(",");
          // LOG_PORT.print(TO_DEG(roll)); LOG_PORT.println();

          if ( ( imu.mx != 0 ) && ( imu.my != 0 ) ) {
               for ( i=1; i<numMoveAveCompass; i++ ) {
                    Fx[i-1] = Fx[i];
                    Fy[i-1] = Fy[i];
                    Fz[i-1] = Fz[i];
               }
               Fx[numMoveAveCompass-1] = imu.mx;
               Fy[numMoveAveCompass-1] = imu.my;
               Fz[numMoveAveCompass-1] = imu.mz;

               FxAve = 0.0;
               FyAve = 0.0;
               FzAve = 0.0;
               for (i=0; i<numMoveAveCompass; i++ ) {
                    FxAve += Fx[i];
                    FyAve += Fy[i];
                    FzAve += Fz[i];
               }
               FxAve = FxAve / (1.0 * numMoveAveCompass );
               FyAve = FyAve / (1.0 * numMoveAveCompass );
               FzAve = FzAve / (1.0 * numMoveAveCompass );
               
               char crc = 0;
               dataPkt.begin();
               dataPkt.print( "$IMU," );
               dataPkt.print( TO_DEG( yaw ) );
               dataPkt.print( "," );
               dataPkt.print( TO_DEG( pitch ) ); 
               dataPkt.print( "," );
               dataPkt.print( TO_DEG( roll ) );
               dataPkt.print( "," );
               dataPkt.print( FxAve );
               //dataPkt.print( magnetom[0] );
               dataPkt.print( "," );
               dataPkt.print( FyAve );
               //dataPkt.print( magnetom[1] );
               dataPkt.print( "," );
               dataPkt.print( FzAve );
               //dataPkt.print( magnetom[2] );
               dataPkt.print( "," );
               // dataPkt.print( gyro[0] );
               // dataPkt.print( "," );
               // dataPkt.print( gyro[1] );
               // dataPkt.print( "," );
               dataPkt.print( gyro[2] );
               /* Calculate and print the CRC */
               for (int k=1; k<dataPkt.length(); k++) {
                    crc^=dataBuffer[k];
               }
               dataPkt.print( "*" );
               if ( crc < 16 ) { dataPkt.print( "0" ); }
               dataPkt.print( crc, HEX );
               dataPkt.print( "\r\n" );
               LOG_PORT.print( dataPkt );
          } else {
               LOG_PORT.println( "no data" );
          }
     }
}

void output_calibration(int calibration_sensor)
{
  if (calibration_sensor == 0)  // Accelerometer
  {
    // Output MIN/MAX values
    LOG_PORT.print("accel x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
      if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
      LOG_PORT.print(accel_min[i]);
      LOG_PORT.print("/");
      LOG_PORT.print(accel_max[i]);
      if (i < 2) LOG_PORT.print("  ");
      else LOG_PORT.println();
    }
  }
  else if (calibration_sensor == 1)  // Magnetometer
  {
    // Output MIN/MAX values
    LOG_PORT.print("magn x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
      if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
      LOG_PORT.print(magnetom_min[i]);
      LOG_PORT.print("/");
      LOG_PORT.print(magnetom_max[i]);
      if (i < 2) LOG_PORT.print("  ");
      else LOG_PORT.println();
    }
  }
  else if (calibration_sensor == 2)  // Gyroscope
  {
    // Average gyro values
    for (int i = 0; i < 3; i++)
      gyro_average[i] += gyro[i];
    gyro_num_samples++;
      
    // Output current and averaged gyroscope values
    LOG_PORT.print("gyro x,y,z (current/average) = ");
    for (int i = 0; i < 3; i++) {
      LOG_PORT.print(gyro[i]);
      LOG_PORT.print("/");
      LOG_PORT.print(gyro_average[i] / (float) gyro_num_samples);
      if (i < 2) LOG_PORT.print("  ");
      else LOG_PORT.println();
    }
  }
}

void output_sensors_text(char raw_or_calibrated)
{
  LOG_PORT.print("#A-"); LOG_PORT.print(raw_or_calibrated); LOG_PORT.print('=');
  LOG_PORT.print(accel[0]); LOG_PORT.print(",");
  LOG_PORT.print(accel[1]); LOG_PORT.print(",");
  LOG_PORT.print(accel[2]); LOG_PORT.println();

  LOG_PORT.print("#M-"); LOG_PORT.print(raw_or_calibrated); LOG_PORT.print('=');
  LOG_PORT.print(magnetom[0]); LOG_PORT.print(",");
  LOG_PORT.print(magnetom[1]); LOG_PORT.print(",");
  LOG_PORT.print(magnetom[2]); LOG_PORT.println();

  LOG_PORT.print("#G-"); LOG_PORT.print(raw_or_calibrated); LOG_PORT.print('=');
  LOG_PORT.print(gyro[0]); LOG_PORT.print(",");
  LOG_PORT.print(gyro[1]); LOG_PORT.print(",");
  LOG_PORT.print(gyro[2]); LOG_PORT.println();
}

void output_both_angles_and_sensors_text()
{
  LOG_PORT.print("#YPRAG=");
  LOG_PORT.print(TO_DEG(yaw)); LOG_PORT.print(",");
  LOG_PORT.print(TO_DEG(pitch)); LOG_PORT.print(",");
  LOG_PORT.print(TO_DEG(roll)); LOG_PORT.print(",");
  
  LOG_PORT.print(Accel_Vector[0]); LOG_PORT.print(",");
  LOG_PORT.print(Accel_Vector[1]); LOG_PORT.print(",");
  LOG_PORT.print(Accel_Vector[2]); LOG_PORT.print(",");

  LOG_PORT.print(Gyro_Vector[0]); LOG_PORT.print(",");
  LOG_PORT.print(Gyro_Vector[1]); LOG_PORT.print(",");
  LOG_PORT.print(Gyro_Vector[2]); LOG_PORT.println();
}

void output_sensors_binary()
{
  LOG_PORT.write((byte*) accel, 12);
  LOG_PORT.write((byte*) magnetom, 12);
  LOG_PORT.write((byte*) gyro, 12);
}

void output_sensors()
{
  if (output_mode == OUTPUT__MODE_SENSORS_RAW)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('R');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_CALIB)
  {
    // Apply sensor calibration
    compensate_sensor_errors();
    
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('C');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_BOTH)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
    {
      output_sensors_binary();
      compensate_sensor_errors();
      output_sensors_binary();
    }
    else if (output_format == OUTPUT__FORMAT_TEXT)
    {
      output_sensors_text('R');
      compensate_sensor_errors();
      output_sensors_text('C');
    }
  }
}
