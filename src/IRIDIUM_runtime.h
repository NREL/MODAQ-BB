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
