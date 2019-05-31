#include <DCMotorController.h>

#define NUM_MOTORS 4
DCMotorController* motors[NUM_MOTORS];

double max_speed_rps = 210.0 * 2.0 * PI / 60.0;  // 210 rpm -> 22 rad/s

void setupMotors()
{
    motors[0] = new DCMotorController(29, 11, 12, 2, 3);
    motors[1] = new DCMotorController(30, 24, 25, 4, 5);
    motors[2] = new DCMotorController(36, 26, 27, 6, 7);
    motors[3] = new DCMotorController(35, 31, 32, 8, 9);

    // motors[0]->flipMotorLeads();
    motors[0]->flipEncoderLeads();

    // motors[1]->flipMotorLeads();
    motors[1]->flipEncoderLeads();

    motors[2]->flipMotorLeads();
    // motors[2]->flipEncoderLeads();

    // motors[3]->flipMotorLeads();
    // motors[3]->flipEncoderLeads();

    for (size_t index = 0; index < NUM_MOTORS; index++)
    {
        motors[index]->setGearboxProperties(12, 150.58);
        motors[index]->setPid(5.0, 1.0, 0.1);
        motors[index]->setPointBounds(max_speed_rps, 20);
        motors[index]->updateDelay = 10000;
        // motors[index]->flipEncoder = true;
#ifdef NOISE_FILTER_LOW_PASS
        motors[index]->speedSmoothK = 0.25;
#endif

        motors[index]->begin();
    }
}

void resetAll()
{
    for (size_t index = 0; index < NUM_MOTORS; index++) {
        motors[index]->stop();
    }

    delay(100);  // ensure motors are stopped

    for (size_t index = 0; index < NUM_MOTORS; index++) {
        motors[index]->reset();
    }
}

void driveForward(float speed)
{
    for (size_t index = 0; index < NUM_MOTORS; index++) {
        motors[index]->setSpeed(speed);
    }
}

void driveSideways(float speed)
{
    // set_motor_command(
    //     linear_y - linear_x + angular_z,
    //     linear_y + linear_x + angular_z,
    //     linear_y + linear_x - angular_z,
    //     linear_y - linear_x - angular_z
    // );

    motors[0]->setSpeed(-speed);
    motors[1]->setSpeed(speed);
    motors[2]->setSpeed(speed);
    motors[3]->setSpeed(-speed);
}

void rotate(float speed)
{
    motors[0]->setSpeed(-speed);
    motors[1]->setSpeed(-speed);
    motors[2]->setSpeed(speed);
    motors[3]->setSpeed(speed);
}


void setup()
{
    setupMotors();

    Serial.begin(115200);
}

void loop()
{
    for (size_t index = 0; index < NUM_MOTORS; index++) {
        motors[index]->update();
    }

    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        char c = command.charAt(0);
        if (c == '-' || ('0' <= c && c <= '9')) {
            driveForward(command.toFloat());
            // driveSideways(command.toFloat());
            // rotate(command.toFloat());
            // for (size_t index = 0; index < NUM_MOTORS; index++) {
            //     motors[index]->setMotorCommand(command.toInt());
            // }
        }
        else if (c == 's') {
            resetAll();
        }
        else if (c == 'k') {
            for (size_t index = 0; index < NUM_MOTORS; index++) {
                switch (command.charAt(1)) {
                    case 'p': motors[index]->Kp = command.substring(2).toFloat(); break;
                    case 'i': motors[index]->Ki = command.substring(2).toFloat(); break;
                    case 'd': motors[index]->Kd = command.substring(2).toFloat(); break;
                }
            }
            Serial.print(motors[0]->Kp); Serial.print('\t');
            Serial.print(motors[0]->Ki); Serial.print('\t');
            Serial.println(motors[0]->Kd);
        }
        else {
            for (size_t index = 0; index < NUM_MOTORS; index++) {
                Serial.print((float)motors[index]->getCurrentCommand()); Serial.print('\t');
                Serial.print((float)motors[index]->getSpeed()); Serial.print('\t');
                Serial.println((float)motors[index]->getPosition());
            }
            Serial.println();
        }
    }
}
