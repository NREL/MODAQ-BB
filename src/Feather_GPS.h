#include <TinyGPS++.h>

// Module Definitions
TinyGPSPlus gps;
TinyGPSDate d;
TinyGPSTime t;

void saveGPSData(){
  d = gps.date;
  t = gps.time;
  // Adjust the RTC based on the time from the GPS
  rtc.adjust(DateTime(d.year(), d.month(), d.day(), t.hour(), t.minute(), t.second()));

  char sz[32];
  char dz[32];
  
  sprintf(sz, "%02d/%02d/%02d", d.month(), d.day(), d.year());
  sprintf(dz, "%02d:%02d:%02d", t.hour(), t.minute(), t.second());
  sprintf(gpsBuffer, "%s,%s,%3.6f,%3.6f,%d,%4.1f,%d,%2.2f,%3.2f", sz, dz,
            gps.location.lat(), gps.location.lng(), gps.location.age(), 
            gps.altitude.meters(), gps.satellites.value(), gps.speed.knots(), 
            gps.course.deg() 
            );
  appendFile(SD, gpsFile, gpsBuffer);
  toLogFile(SD, logFile, gpsBuffer);
}


// Collect GPS data and store in gpsBuffer
void collectGPSData() {
  GPS.sendCommand(""); //wake from standby 
  timer = millis();
  // Loop until we get an updated GPS location or we hit the GPS_FIX_TIMEOUT
  toLogFile(SD, logFile, "Searching for GPS fix...");
  while (millis() - timer < GPS_FIX_TIMEOUT) {
    while(GPS.available() > 0) {
      gps.encode(GPS.read());
      if (gps.location.isUpdated()) {
        // Process updated GPS data here
        toLogFile(SD, logFile, "GPS fix acquired");
        saveGPSData();
        GPS.sendCommand(PMTK_STANDBY);
        return; // Exit after processing updated data
      }
    }
  }
  sprintf(gpsBuffer, "No Fix ,,,,,,,,,");
  toLogFile(SD, logFile, gpsBuffer);
  appendFile(SD, gpsFile, gpsBuffer);
  GPS.sendCommand(PMTK_STANDBY);
}
