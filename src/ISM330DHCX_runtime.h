void configureWakeupInterrupt_IMU() {
  // Configure the interrupt pin outputs
  ism330dhcx.configIntOutputs(true, true);

  // Set the interrupt to be triggered on INT1 pin for wake-up detection
  ism330dhcx.configInt1(false, false, false, false, true);
  
  // Enable wake-up detection
  ism330dhcx.enableWakeup(true, 20, 40); //Threshold (0 to 64) = 40 * 0.0313 g = 1.252 g
                                        //Threshold (0 to 64) = 20 * 0.0313 g = 0.626 g
                                        // The duration is set to 0, meaning the wake-up interrupt will be triggered immediately when the threshold is crossed                
}

void IRAM_ATTR wakeUpDetectedIMU() {
  EXT_IMU_INT = true;
}

void clearIMUWakeInterrupt() {
  // enableWakeup() sets LIR=1 (latched interrupt) on the ISM330DHCX.
  // Reading ALL_INT_SRC (0x1A) clears ALL latched interrupt sources and lets INT1 return high.
  // Reading WAKE_UP_SRC (0x1B) afterward confirms the wakeup condition is gone.
  const uint8_t ISM330DHCX_ADDR = 0x6A;
  uint8_t dummy;
  Wire.beginTransmission(ISM330DHCX_ADDR);
  Wire.write(0x1A); // ALL_INT_SRC register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ISM330DHCX_ADDR, (uint8_t)2); // reads ALL_INT_SRC + WAKE_UP_SRC
  while (Wire.available()) {
    dummy = Wire.read();
  }
  (void)dummy; // suppress unused variable warning
}

void collectIMUData() {
  int ms;
  DateTime imuTime;

  sensors_event_t accel, g, tmp;
  sensors_event_t event;
  // not sure if we want to do running average stats
  for(int i = 0; i < NUM_IMU_SAMPLES; i++) {
    char dateBuffer[32] = "MM/DD/YYYY";
    char timeBuffer[32] = "hh:mm:ss";
    rtc.now().toString(dateBuffer);
    rtc.now().toString(timeBuffer);

    ism330dhcx.getEvent(&accel, &g, &tmp);
    lis3mdl.getEvent(&event);

    ax = accel.acceleration.x;
    ay = accel.acceleration.y;
    az = accel.acceleration.z;

    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;

    mx = event.magnetic.x;
    my = event.magnetic.y;
    mz = event.magnetic.z;

    temp = tmp.temperature;

    snprintf(imuBuffer, sizeof(imuBuffer), "%s,%s,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%3.1f,%i",dateBuffer, timeBuffer, 
              ax, ay, az, gx, gy, gz, mx, my, mz, temp, imuWakeupCount);
              
    appendFile(SD, imuFile, imuBuffer);
    delay(IMU_RATE);
    } 
  }