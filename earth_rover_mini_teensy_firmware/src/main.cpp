#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <DCMotorController.h>
#include <Servo.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Max current draw: 2.5A @ ~11.5 supply voltage, peak power: 26.824 W
Adafruit_INA219 ina219;

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;

float maxCurrent_mA = 0;
float maxPower_mW = 0;


#define NUM_MOTORS 4
DCMotorController* motors[NUM_MOTORS];

double max_speed_rps = 210.0 * 2.0 * PI / 60.0;  // 210 rpm -> 22 rad/s

Servo tilt_servo;
int servo_val = 0;

Adafruit_BNO055 bno = Adafruit_BNO055();

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
        motors[index]->setPid(5.0, 0.0, 0.1);
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
    Serial.begin(115200);

    setupMotors();

    ina219.begin();  // defaults to max range 32V, +/-3.2A
    //ina219.setCalibration_32V_1A();
    // ina219.setCalibration_16V_400mA();

    tilt_servo.attach(20);

    /* Initialise the sensor */
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    delay(1000);

    /* Display the current temperature */
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");

    bno.setExtCrystalUse(true);
}

void loop()
{
    for (size_t index = 0; index < NUM_MOTORS; index++) {
        motors[index]->update();
    }

    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    if (current_mA > maxCurrent_mA) {
        maxCurrent_mA = current_mA;
    }
    if (power_mW > maxPower_mW) {
        maxPower_mW = power_mW;
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
        else if (c == 't') {
            if (servo_val == 0) {
                servo_val = 180;
            }
            else {
                servo_val = 0;
            }
            tilt_servo.write(servo_val);
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

            Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
            Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
            Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
            Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
            Serial.print("Peak Current:  "); Serial.print(maxCurrent_mA); Serial.println(" mA");
            Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
            Serial.print("Peak Power:    "); Serial.print(maxPower_mW); Serial.println(" mW");
            Serial.println();

            imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            Serial.print("X: ");
            Serial.print(euler.x());
            Serial.print(" Y: ");
            Serial.print(euler.y());
            Serial.print(" Z: ");
            Serial.print(euler.z());
            Serial.print("\t\t");

            uint8_t system, gyro, accel, mag = 0;
            bno.getCalibration(&system, &gyro, &accel, &mag);
            Serial.print("CALIBRATION: Sys=");
            Serial.print(system, DEC);
            Serial.print(" Gyro=");
            Serial.print(gyro, DEC);
            Serial.print(" Accel=");
            Serial.print(accel, DEC);
            Serial.print(" Mag=");
            Serial.println(mag, DEC);
            Serial.println();
        }
    }
}
