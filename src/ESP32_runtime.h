void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    #ifdef DEBUG
      Serial.println("Failed to open file for appending");
    #endif
    return;
  }
  if (file.println(message)) {
  } else {
    #ifdef DEBUG
      Serial.println("Append failed");
    #endif
  }
  file.close();
}

void toLogFile(fs::FS &fs, const char *path, const char *message) {
  #ifdef DEBUG
    // Calculate proper buffer size: message length + timestamp overhead + null terminator
    size_t msgLen = strlen(message);
    size_t bufSize = msgLen + 64;  // 64 bytes for timestamp prefix
    char *timestamped_message = (char*)malloc(bufSize);
    if (timestamped_message == NULL) {
      Serial.println("ERROR: Memory allocation failed in toLogFile()");
      return;
    }
    char dateBuffer[32] = "MM/DD/YYYY";
    char timeBuffer[32] = "hh:mm:ss";

    rtc.now().toString(dateBuffer);
    rtc.now().toString(timeBuffer);

    snprintf(timestamped_message, bufSize, "[%s %s] %s", dateBuffer, timeBuffer, message);
    appendFile(fs, path, timestamped_message);
    
    // Also print message to serial for real-time debugging
    Serial.println(timestamped_message);
    free(timestamped_message); 
    timestamped_message = NULL; // Avoid dangling pointer
  #endif
}

void combine_data_buffers() {
    const char* gpsN = gpsBuffer;
    const char* accN = imuBuffer;
    const char* pwrN = pwrBuffer;
    const char* comma = ",";

    // Calculate required size with proper bounds
    size_t required_size = strlen(gpsN) + strlen(accN) + strlen(pwrN) + 2 + 1; // 2 commas + null terminator
    
    char* tmp = (char*)malloc(required_size);
    
    if (tmp == NULL) {
        Serial.println("ERROR: Memory allocation failed in combine_data_buffers()");
        toLogFile(SD, logFile, "ERROR: Memory allocation failed in combine_data_buffers()");
        return; // Exit early to prevent crash
    }
    
    // Use snprintf for safe concatenation
    int written = snprintf(tmp, required_size, "%s,%s,%s", gpsN, accN, pwrN);
    
    if (written < 0 || written >= (int)required_size) {
        Serial.println("ERROR: Buffer overflow in combine_data_buffers()");
        toLogFile(SD, logFile, "ERROR: Buffer overflow in combine_data_buffers()");
        free(tmp);
        return;
    }
    
    int s = strlen(tmp);
    Serial.print("Buffer Size = ");
    Serial.println(s);
    
    // Use snprintf instead of sprintf for safety
    snprintf(fileBuffer, sizeof(fileBuffer), "%s", tmp);
    
    free(tmp);
    tmp = NULL; // Avoid dangling pointer
}

void createDir(fs::FS &fs, const char *path) {
  #ifdef DEBUG_FILE
    Serial.printf("Creating Dir: %s\n", path);
  #endif

  if (fs.mkdir(path)) {
    #ifdef DEBUG_FILE
      Serial.println("Dir created");
    #endif
  } else {
    #ifdef DEBUG_FILE
      Serial.println("mkdir failed");
    #endif
  }
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  #ifdef DEBUG_FILE
    Serial.printf("Writing file: %s\n", path);
  #endif

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    #ifdef DEBUG_FILE
      Serial.println("Failed to open file for writing");
    #endif
    return;
  }
  if (file.print(message)) {
    #ifdef DEBUG_FILE
      Serial.println("File written");
    #endif
  } else {
    #ifdef DEBUG_FILE
      Serial.println("Write failed");
    #endif
  }
  file.close();
}
