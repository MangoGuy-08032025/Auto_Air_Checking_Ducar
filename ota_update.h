#include <WiFi.h>
#include <HTTPUpdate.h>
#include <Update.h>
#include <HTTPClient.h>
#define FW_VERSION "9"
const char* firmware_url = "https://raw.githubusercontent.com/MangoGuy-08032025/Auto_Air_Checking_Ducar/main/Auto_air_checking.ino.bin";
const char* version_url  = "https://raw.githubusercontent.com/MangoGuy-08032025/Auto_Air_Checking_Ducar/main/version.txt";
String ota_result = "Nothing";
bool startOTAUpdate(WiFiClient* client, int contentLength);
void downloadAndApplyFirmware();
String checkAndUpdateFirmware();

String checkAndUpdateFirmware() {
      HTTPClient http;
      http.begin(version_url);
      int httpCode = http.GET();

      if (httpCode == 200) {
        String newVersion = http.getString();
        ota_result = newVersion;
        newVersion.trim();

        Serial.printf("Current version: %s, New version: %s\n", FW_VERSION, newVersion.c_str());
        if (newVersion != FW_VERSION) {
          Serial.println("New version available -> Start OTA update");
          downloadAndApplyFirmware();
        } else {
          Serial.println("Already up-to-date -> Skip OTA");
          ota_result = "No newer version";
        }
      } else {
        Serial.printf("Failed to fetch version file, HTTP code: %d\n", httpCode);
        ota_result = "Failed to fetch";
      }
      http.end();
      return ota_result;
}


void downloadAndApplyFirmware() {
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.begin(firmware_url);

  int httpCode = http.GET();
  Serial.printf("HTTP GET code: %d\n", httpCode);

  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    Serial.printf("Firmware size: %d bytes\n", contentLength);

    if (contentLength > 0) {
      WiFiClient* stream = http.getStreamPtr();
      if (startOTAUpdate(stream, contentLength)) {
        Serial.println("OTA update successful, restarting...");
        delay(2000);
        ESP.restart();
      } else {
        Serial.println("OTA update failed");
      }
    } else {
      Serial.println("Invalid firmware size");
    }
  }
  else 
  {
    Serial.printf("Failed to fetch firmware. HTTP code: %d\n", httpCode);
  }
  http.end();
}


bool startOTAUpdate(WiFiClient* client, int contentLength) {
  if (!Update.begin(contentLength)) {
    Serial.printf("Update begin failed: %s\n", Update.errorString());
    return false;
  }
  size_t written = 0;
  int progress = 0;
  int lastProgress = 0;

  // Timeout variables
  const unsigned long timeoutDuration = 120*1000;  // 10 seconds timeout
  unsigned long lastDataTime = millis();

  while (written < contentLength) {
    if (client->available())
    {
      uint8_t buffer[128];
      size_t len = client->read(buffer, sizeof(buffer));
      if (len > 0) {
        Update.write(buffer, len);
        written += len;

        // Calculate and print progress
        progress = (written * 100) / contentLength;
        if (progress != lastProgress) {
          Serial.printf("Writing Progress: %d%%\n", progress);
          lastProgress = progress;
        }
      }
    }
    // Check for timeout
    if (millis() - lastDataTime > timeoutDuration) {
      Serial.println("Timeout: No data received for too long. Aborting update...");
      Update.abort();
      return false;
    }

    yield();
  }
  Serial.println("\nWriting complete");

  if (written != contentLength) {
    Serial.printf("Error: Write incomplete. Expected %d but got %d bytes\n", contentLength, written);
    Update.abort();
    return false;
  }

  if (!Update.end()) {
    Serial.printf("Error: Update end failed: %s\n", Update.errorString());
    return false;
  }

  Serial.println("Update successfully completed");
  return true;
}

