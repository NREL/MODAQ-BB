void IRAM_ATTR wakeUpDetectedRTC() {
  // Serial.println("Wake-up event from the RTC detected!");
  // Add your code to handle the wake-up event here
  EXT_RTC_INT = true;
}

void printTime(DateTime now) {
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print('T');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
}

unsigned long unix(DateTime now) {
  unsigned long unixTime = (now.year() - 1970) * 365 * 24 * 3600 +
                            (now.month() - 1) * 30 * 24 * 3600 +
                            (now.day() - 1) * 24 * 3600 +
                            now.hour() * 3600 +
                            now.minute() * 60 +
                            now.second();
  return unixTime;
}

void syncTimeWithRTC()
{
  DateTime now = rtc.now();

  // Create time structure
  struct tm timeinfo;
  timeinfo.tm_year = now.year() - 1900; // Years since 1900
  timeinfo.tm_mon = now.month() - 1;    // Months since January (0-11)
  timeinfo.tm_mday = now.day();
  timeinfo.tm_hour = now.hour();
  timeinfo.tm_min = now.minute();
  timeinfo.tm_sec = now.second();
  timeinfo.tm_isdst = -1; // Let system determine DST

  // Convert to time_t
  time_t t = mktime(&timeinfo);

  // Set system time
  struct timeval tv = {.tv_sec = t};
  settimeofday(&tv, NULL);
}