#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SensorLib.h>
#include "SensorQMI8658.hpp"

#define SPI_MOSI (35)

#define SPI_SCK (36)

#define SPI_MISO (37)

#define IMU_CS (34)

#define IMU_INT (33)

float impactThreshold = 5;
// Abstract Motion class
class Motion
{
public:
    virtual void begin() = 0;                // Initialize the sensor
    virtual bool detectMotion() = 0;         // Detect motion and return true if motion is detected
    virtual void displayAccelerometer() = 0; // Display acceleration
};

// MPU6050 implementation
class MPU6050Motion : public Motion
{
private:
    Adafruit_MPU6050 mpu;

    float accelSamples[5];
    int sampleIndex = 0;

public:
    void begin() override
    {
        if (!mpu.begin())
        {
            Serial.println("Tbeam:Failed to initialize MPU6050!");
            while (true)
                ; // Halt on failure
        }
        Serial.println("Tbeam:MPU6050 initialized.");
        // mpu.setMotionDetectionThreshold(5);
        mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
        mpu.setMotionDetectionThreshold(20);
        mpu.setMotionDetectionDuration(20);
        mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
        mpu.setInterruptPinPolarity(true);
    }

    void displayAccelerometer() override {}
    bool detectMotion() override
    {

        Serial.println("Checking motion");
        if (mpu.getMotionInterruptStatus())
        {
            Serial.println("Checking motion true");
            return true;
        }
        else
            return false;
    }
};

// QMI8658 implementation
volatile bool interruptFlag = false;
void setFlag()
{
    interruptFlag = true;
}
class QMI8658Motion : public Motion
{
private:
    SensorQMI8658 qmi;
    IMUdata acc;
    IMUdata gyr;
    int csPin, intPin;

public:
    QMI8658Motion(int cs, int irq) : csPin(cs), intPin(irq) {}

    void begin() override
    {
         qmi.setPins(IMU_INT);
        if (!qmi.begin(csPin, SPI_MOSI, SPI_MISO, SPI_SCK))
        {
            Serial.println("Failed to find QMI8658 - check your wiring!");
            while (true)
            {
                delay(1000);
                Serial.println("Failed to find QMI8658 - check your wiring!");
            }
        }

        Serial.print("Device ID:");
        Serial.println(qmi.getChipID(), HEX);

        //** The recommended output data rate for detection is higher than 500HZ
        qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_500Hz);

        // Enable the accelerometer
        qmi.enableAccelerometer();

        //* Configure the motion detection axis direction
        uint8_t modeCtrl = SensorQMI8658::ANY_MOTION_EN_X |
                           SensorQMI8658::ANY_MOTION_EN_Y |
                           SensorQMI8658::ANY_MOTION_EN_Z |
                           SensorQMI8658::NO_MOTION_EN_X |
                           SensorQMI8658::NO_MOTION_EN_Y |
                           SensorQMI8658::NO_MOTION_EN_Z;

        //* Define the slope threshold of the x-axis for arbitrary motion detection
        float AnyMotionXThr = 100.0; //  x-axis 100mg threshold
        //* Define the slope threshold of the y-axis for arbitrary motion detection
        float AnyMotionYThr = 100.0; //  y-axis 100mg threshold
        //* Define the slope threshold of the z-axis for arbitrary motion detection
        float AnyMotionZThr = 1.0; //  z-axis 1mg threshold
        //* Defines the minimum number of consecutive samples (duration) that the absolute
        //* of the slope of the enabled axis/axes data should keep higher than the threshold
        uint8_t AnyMotionWindow = 1; //  1 samples

        // TODO: No motion detection does not work
        //* Defines the slope threshold of the x-axis for no motion detection
        float NoMotionXThr = 0.1;
        //* Defines the slope threshold of the y-axis for no motion detection
        float NoMotionYThr = 0.1;
        //* Defines the slope threshold of the z-axis for no motion detection
        float NoMotionZThr = 0.1;

        //* Defines the minimum number of consecutive samples (duration) that the absolute
        //* of the slope of the enabled axis/axes data should keep lower than the threshold
        uint8_t NoMotionWindow = 1; //  1 samples
        //* Defines the wait window (idle time) starts from the first Any-Motion event until
        //* starting to detecting another Any-Motion event form confirmation
        uint16_t SigMotionWaitWindow = 1; //  1 samples
        //* Defines the maximum duration for detecting the other Any-Motion
        //* event to confirm Significant-Motion, starts from the first Any -Motion event
        uint16_t SigMotionConfirmWindow = 1; //  1 samples

        qmi.configMotion(modeCtrl,
                         AnyMotionXThr, AnyMotionYThr, AnyMotionZThr, AnyMotionWindow,
                         NoMotionXThr, NoMotionYThr, NoMotionZThr, NoMotionWindow,
                         SigMotionWaitWindow, SigMotionConfirmWindow);

        // Enable the Motion Detection and enable the interrupt
        qmi.enableMotionDetect(SensorQMI8658::INTERRUPT_PIN_1);

        /*
         * When the QMI8658 is configured as Wom, the interrupt level is arbitrary,
         * not absolute high or low, and it is in the jump transition state
         */
        attachInterrupt(IMU_INT, setFlag, CHANGE);
    }

    bool detectMotion() override
    {
        if (interruptFlag)
        {
            interruptFlag = false;
            uint8_t status = qmi.getStatusRegister();

            if (status & SensorQMI8658::EVENT_ANY_MOTION)
            {
                Serial.println("Motion detected: EVENT_ANY_MOTION");
                return true;
            }
            else if (status & SensorQMI8658::EVENT_WOM_MOTION)
            {
                Serial.println("Motion detected: EVENT_WOM_MOTION");
                return true;
            }
        }
        return false;
    }
    void displayAccelerometer() override
    {
        // if (qmi.getDataReady())
        // {

        //     // Serial.print("Timestamp:");
        //     // Serial.print(qmi.getTimestamp());

        //     if (qmi.getAccelerometer(acc.x, acc.y, acc.z))
        //     {

        //         // Print to serial plotter
        //         Serial.print("ACCEL.x:");
        //         Serial.print(acc.x);
        //         Serial.print(",");
        //         Serial.print("ACCEL.y:");
        //         Serial.print(acc.y);
        //         Serial.print(",");
        //         Serial.print("ACCEL.z:");
        //         Serial.print(acc.z);
        //         Serial.println();

        //         /*
        //         m2/s to mg
        //         Serial.print(" ACCEL.x:"); Serial.print(acc.x * 1000); Serial.println(" mg");
        //         Serial.print(",ACCEL.y:"); Serial.print(acc.y * 1000); Serial.println(" mg");
        //         Serial.print(",ACCEL.z:"); Serial.print(acc.z * 1000); Serial.println(" mg");
        //         */
        //     }
        // }
    }
};

// Motion factory to choose between MPU6050 and QMI8658
class MotionFactory
{
public:
    static Motion *createMotion(bool useMPU6050, int csPin = -1, int intPin = -1)
    {
        if (useMPU6050)
        {
            return new MPU6050Motion();
        }
        else
        {
            return new QMI8658Motion(csPin, intPin);
        }
    }
};
