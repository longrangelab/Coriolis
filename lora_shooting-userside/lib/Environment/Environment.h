#include <ESP32Encoder.h>
#define TICKS_PER_REV  600  // Total encoder ticks per revolution
#define ANGLE_PER_TICK (TICKS_PER_REV / 12.0) // Ticks per wind direction segment

// Abstract Environment class
class Environment {
public:
  virtual void begin() = 0;          // Initialize environment sensors
  virtual int getWindDirection() = 0; // Get wind direction as an integer (1-12)
};

// Implementation for wind direction using an encoder
class WindEnvironment : public Environment {
private:
  ESP32Encoder encoder;
  
public:
  // Constructor for the encoder with specific pins
  WindEnvironment(int encoderPinA, int encoderPinB) {
    encoder.attachHalfQuad(encoderPinA, encoderPinB);
    encoder.clearCount();
  }

  void begin() override {
    encoder.clearCount(); // Initialize the encoder count to 0
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
};
