#include <Arduino.h>

// Abstract External Notification class
class ExternalNotification
{
public:
    virtual void notify() = 0; // Trigger a notification
};

// Buzzer class inheriting from ExternalNotification
class Buzzer : public ExternalNotification
{
private:
    int buzzerPin;

public:
    Buzzer(int pin) : buzzerPin(pin)
    {
        pinMode(buzzerPin, OUTPUT);
        digitalWrite(buzzerPin, LOW);
    }

    void notify() override
    {
        for (int i = 0; i < 3; i++)
        {
            digitalWrite(buzzerPin, HIGH); // Turn the buzzer ON
            delay(500);                    // Wait for 1 second
            digitalWrite(buzzerPin, LOW);  // Turn the buzzer OFF
        }
    }
};
