
int moStatus = -1;
int mtStatus = -1;
int moMSN = -1;
int mtMSN = -1;
int mtLength = -1;
int mtQueued = -1;

void sendSatMsg(char* message) {
  // Begin satellite modem operation
  toLogFile(SD, logFile, "Starting modem...");
  int err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    char errBuf[64];
    snprintf(errBuf, sizeof(errBuf), "Begin failed: error %d", err);
    toLogFile(SD, logFile, errBuf);
    if (err == ISBD_NO_MODEM_DETECTED)
      toLogFile(SD, logFile, "No modem detected: check wiring.");
    return;
  }

  // Send the message
  toLogFile(SD, logFile, "Trying to send a message.  This might take several minutes.");
  err = modem.sendSBDText(message);
  if (err != ISBD_SUCCESS)
  {
    char errBuf[64];
    snprintf(errBuf, sizeof(errBuf), "sendSBDText failed: error %d", err);
    toLogFile(SD, logFile, errBuf);
    if (err == ISBD_SENDRECEIVE_TIMEOUT)
      toLogFile(SD, logFile, "Try again with a better view of the sky.");
  }

  else
  {
    toLogFile(SD, logFile, "Hey, it worked!");
  }

  // Put modem to sleep
  toLogFile(SD, logFile, "Putting modem to sleep.");
  err = modem.sleep();
  if (err != ISBD_SUCCESS)
  {
    char errBuf[64];
    snprintf(errBuf, sizeof(errBuf), "Sleep failed: error %d", err);
    toLogFile(SD, logFile, errBuf);
  }
}


void parseSBDResponse(String response) {
  // Look for the "+SBDIX" response
  // if (response.startsWith("+SBDIX:")) {
    // Remove the "+SBDIX: " prefix
    response.remove(0, 10);

    // Tokenize the response using commas as delimiters
    moStatus = response.substring(0, response.indexOf(',')).toInt();
    response = response.substring(response.indexOf(',') + 1);

    moMSN = response.substring(0, response.indexOf(',')).toInt();
    response = response.substring(response.indexOf(',') + 1);
    
    mtStatus = response.substring(0, response.indexOf(',')).toInt();
    response = response.substring(response.indexOf(',') + 1);

    mtMSN = response.substring(0, response.indexOf(',')).toInt();
    response = response.substring(response.indexOf(',') + 1);

    mtLength = response.substring(0, response.indexOf(',')).toInt();
    response = response.substring(response.indexOf(',') + 1);

    mtQueued = response.toInt();

    #ifdef DEBUG_SAT
      // Print parsed values for verification
      Serial.println("Parsed SBDIX Response:");
      Serial.print("MO Status: "); Serial.println(moStatus);
      Serial.print("MT Status: "); Serial.println(mtStatus);
      Serial.print("MO MSN: "); Serial.println(moMSN);
      Serial.print("MT MSN: "); Serial.println(mtMSN);
      Serial.print("MT Length: "); Serial.println(mtLength);
      Serial.print("MT Queued: "); Serial.println(mtQueued);
    #endif
}