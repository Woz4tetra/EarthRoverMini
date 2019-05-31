#include <DCMotorController.h>

// DCMotorController motor(29, 12, 11, 2, 3);
// DCMotorController motor(30, 25, 24, 4, 5);
// DCMotorController motor(36, 26, 27, 7, 6);
DCMotorController motor(35, 32, 31, 9, 8);

double max_speed_rps = 210.0 * 2.0 * PI / 60.0;  // 210 rpm -> 22 rad/s

void setup()
{
    motor.setGearboxProperties(12, 150.58);
    motor.setPid(50.0, 1.0, 0.1);
    motor.setPointBounds(max_speed_rps, 20);
    motor.updateDelay = 10000;
    // motors[index]->flipEncoder = true;
#ifdef NOISE_FILTER_LOW_PASS
    motor.speedSmoothK = 0.5;
#endif

    motor.begin();

    Serial.begin(9600);
}

void loop()
{
    motor.update();

    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        char c = command.charAt(0);
        if (c == '-' || ('0' <= c && c <= '9')) {
            motor.setSpeed(command.toFloat());
            // motor.setMotorCommand(command.toInt());
        }
        else if (c == 's') {
            motor.stop();
            delay(100);  // ensure motor is stopped
            motor.reset();
        }
        else {
            Serial.println((float)motor.getSpeed());
        }
    }
}
