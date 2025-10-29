#include <Arduino.h>
#include "CustomEncoder.hpp"
#include <ESP32Servo.h>

#define ENCODER_PIN_A 5
#define ENCODER_PIN_B 4
#define ENCODER_RES 360
CustomEncoder encoder(ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_RES);

#define MOTOR_PIN 6
Servo motor;

void updateEncoder()
{
    // esp32 interrupts cannot be defined in the class
    encoder.update();
}

void setup()
{
    Serial.begin(115200);
    delay(50);
    
    // encoder setup
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

    // motor setup
    motor.attach(MOTOR_PIN);
    motor.write(90); // mid-point, 0 velocity
}

void loop()
{
    static float lastPosition = 0.0;
    float position = encoder.getPosition();
    if (position != lastPosition)
    {
        lastPosition = position;

        int motor_position = map(position, -36, 36, 0, 180);
        motor_position = constrain(motor_position, 0, 180);
        Serial.println(motor_position);

        motor.write(motor_position);
    }

}
