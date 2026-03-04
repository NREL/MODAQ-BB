char PWRdata[128];
u_int32_t uptime; // in hours

void collectPWRData() {
  char dateBuffer[32] = "MM/DD/YYYY";
  char timeBuffer[32] = "hh:mm:ss";
  
  for (int i = 0; i < NUM_PWR_SAMPLES; i++) {
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    uptime = millis() / 3600000.0; // Convert uptime to hours

    rtc.now().toString(dateBuffer);
    rtc.now().toString(timeBuffer);
    
    snprintf(pwrBuffer, sizeof(pwrBuffer), "%s,%s,%4.2f,%4.2f,%4.2f,%.2f",dateBuffer, timeBuffer, busvoltage, current_mA, power_mW, uptime);
    appendFile(SD, pwrFile, pwrBuffer);
    delay(PWR_RATE);
  }
}
