#include <Arduino.h>
#include <tle94112-ino.hpp>
#include <Arduino_BMI270_BMM150.h>

// PID constants - tune these
float Kp = 4.4;   // Proportional
float Ki = 0.71;    // Integral
float Kd = 0.08;    // Derivative

// PID variables
float error = 0, previous_error = 0;
float integral = 0, derivative = 0;
float setpoint = 0.0; // desired angle (upright)
float output = 0;

// Timing
unsigned long last_time = 0;
float dt = 0.01; // 10ms loop



// Motor driver
Tle94112Ino controller = Tle94112Ino(3,4);


volatile uint8_t oldDirection [] = { LL_HH, HH_LL };


// IMU
float angle = 0.0;


uint8_t motorReg[2][2] = {
  {REG_ACT_1, REG_PWM_DC_1},
  {REG_ACT_3, REG_PWM_DC_3}
};


void motorSet(uint8_t motorNum, uint8_t dir, uint8_t speed, bool errorCheck = false)
{
    if (dir != oldDirection[motorNum])
    {
        controller.directWriteReg(motorReg[motorNum][0], dir);
        oldDirection[motorNum] = dir;
    }
    controller.directWriteReg(motorReg[motorNum][1], speed);
    if (errorCheck)
    {
        controller.clearErrors();
    }
}



void motor_pwm(int motor, int speed)
{
    if (speed > 0){
        motorSet(motor, LL_HH, abs(speed));
    } else {
        motorSet(motor, HH_LL, abs(speed));
    }
}



void setMultiHalfbridge(bool state) {
    if (state) {
        digitalWrite(4, HIGH); // Turn on the Multi-Halfbridge
        Serial.println("Multi-Halfbridge turned ON. Reinitializing...");
        
        // Reinitialize the motor controller
        controller.begin();

        // Reconfigure half bridges and PWM channels for motors
        controller.configHB(controller.TLE_HB1, controller.TLE_HIGH, controller.TLE_PWM1);
        controller.configHB(controller.TLE_HB2, controller.TLE_HIGH, controller.TLE_PWM1);
        controller.configHB(controller.TLE_HB3, controller.TLE_LOW,  controller.TLE_NOPWM);
        controller.configHB(controller.TLE_HB4, controller.TLE_LOW,  controller.TLE_NOPWM);

        controller.configHB(controller.TLE_HB9,  controller.TLE_HIGH, controller.TLE_PWM3);
        controller.configHB(controller.TLE_HB10, controller.TLE_HIGH, controller.TLE_PWM3);
        controller.configHB(controller.TLE_HB11, controller.TLE_LOW,  controller.TLE_NOPWM);
        controller.configHB(controller.TLE_HB12, controller.TLE_LOW,  controller.TLE_NOPWM);

        // Ensure motors are stopped
        controller.configPWM(controller.TLE_PWM1, controller.TLE_FREQ200HZ, 0);
        controller.configPWM(controller.TLE_PWM3, controller.TLE_FREQ200HZ, 0);
        controller.clearErrors();

        Serial.println("Multi-Halfbridge reinitialized.");
    } else {
        digitalWrite(4, LOW); // Turn off the Multi-Halfbridge
        Serial.println("Multi-Halfbridge turned OFF.");
    }
}



void setup() {
  
    pinMode(4, OUTPUT); // Configure pin 4 as output
    digitalWrite(4, HIGH); // Default: Multi-Halfbridge is ON
    delay(100);

    pinMode(LED2, OUTPUT);

    controller.begin();

    controller.configHB(controller.TLE_HB1, controller.TLE_HIGH, controller.TLE_PWM1);
    controller.configHB(controller.TLE_HB2, controller.TLE_HIGH, controller.TLE_PWM1);
    controller.configHB(controller.TLE_HB3, controller.TLE_LOW,  controller.TLE_NOPWM);
    controller.configHB(controller.TLE_HB4, controller.TLE_LOW,  controller.TLE_NOPWM);

    controller.configHB(controller.TLE_HB9,  controller.TLE_HIGH, controller.TLE_PWM3);
    controller.configHB(controller.TLE_HB10, controller.TLE_HIGH, controller.TLE_PWM3);
    controller.configHB(controller.TLE_HB11, controller.TLE_LOW,  controller.TLE_NOPWM);
    controller.configHB(controller.TLE_HB12, controller.TLE_LOW,  controller.TLE_NOPWM);

    // All stop
    controller.configPWM(controller.TLE_PWM1, controller.TLE_FREQ200HZ, 0);
    controller.configPWM(controller.TLE_PWM3, controller.TLE_FREQ200HZ, 0);
    controller.clearErrors();


    Serial.begin(9600);
    while (!Serial) ;  // Wait for serial connection
    Serial.println("Started");

  // Init IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
}

void loop() {
  if (IMU.accelerationAvailable()) {
    float accX, accY, accZ;
    IMU.readAcceleration(accX, accY, accZ);\
    float gyroX, gyroY, gyroZ;
    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    unsigned long current_time = millis();
    float elapsed = (current_time - last_time) / 1000.0;
    last_time = current_time;

    // Calculate pitch angle using complementary filter
    float accAngle = atan2(accX, accZ) * 180 / PI;
    angle = 0.94 * (angle + gyroY * elapsed) + 0.06 * accAngle;


    // PID Control
    error = setpoint - angle;
    integral += error * elapsed;
    derivative = (error - previous_error) / elapsed;
    output = Kp * error + Ki * integral + Kd * derivative;

    previous_error = error;

    // Drive motors based on output
    int16_t out = constrain(output, -100, 100);

    out = out * -1;

    motor_pwm(0, out); 
    motor_pwm(1, out);  

    // Debug
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" | Output: ");
    Serial.println(out);
    delay(0.2);
  }
  
}