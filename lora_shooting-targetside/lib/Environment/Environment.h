#include <ESP32Encoder.h>
#define TICKS_PER_REV  600  // Total encoder ticks per revolution
#define ANGLE_PER_TICK (TICKS_PER_REV / 12.0) // Ticks per wind direction segment

// Abstract Environment class
class Environment {
public:
  virtual void begin() = 0;          // Initialize environment sensors
  virtual int getWindDirection() = 0; // Get wind direction as an integer (1-12)
  virtual int getWindSpeed() = 0;
};

// Implementation for wind direction using an encoder
class WindEnvironment : public Environment {
private:
  ESP32Encoder encoder;
  ESP32Encoder encoder2;  //Speed
  unsigned long lastMeasureTime = 0;
  int64_t lastEncoderSpeedCount = 0;
  int lastRPM = 0;
  
public:
  // Constructor for the encoder with specific pins
  WindEnvironment(int dirA, int dirB, int spdA, int spdB) {
    encoder.attachHalfQuad(dirA, dirB);
    encoder.clearCount();

    encoder2.attachHalfQuad(spdA, spdB);
    encoder2.clearCount();
  }

  void begin() override {
    encoder.clearCount(); // Initialize the encoder count to 0
    encoder2.clearCount();
    lastMeasureTime = millis();
    lastEncoderSpeedCount = encoder2.getCount();
  }

  int getWindDirection() override {
    // Calculate wind direction from encoder value
    int currentWindValue = (encoder.getCount()/2) ;
    currentWindValue = currentWindValue % TICKS_PER_REV;
    currentWindValue = currentWindValue / ANGLE_PER_TICK; // Map ticks to wind directions
    if (currentWindValue < 0) {
      currentWindValue += 12; // Handle negative counts
    }
    // currentWindValue = 12 - currentWindValue; // Reverse the wind direction
    if (currentWindValue == 0) {
      currentWindValue = 12; // Ensure 0 becomes 12
    }
    return currentWindValue;
  }

  int getWindSpeed() override {   //WindSpeed
    unsigned long now = millis();
    unsigned long deltaTime = now - lastMeasureTime;

    if(deltaTime < 200) return lastRPM; //overrun
    int64_t currentCount = encoder2.getCount();
    int64_t deltaCount = currentCount - lastEncoderSpeedCount;

    float revolutions = (float)deltaCount / TICKS_PER_REV;
    float minutes = (float)deltaTime / 60000.0;
    int rpm = abs((int)(revolutions / minutes));
    lastMeasureTime = now;
    lastEncoderSpeedCount = currentCount;
    lastRPM = rpm;
    return rpm;
  }
};
