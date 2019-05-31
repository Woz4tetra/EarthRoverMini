#include <DCMotorController.h>

DCMotorController::DCMotorController(int pwm_pin, int dir_pin_1, int dir_pin_2, int enc_pin_1, int enc_pin_2)
{
    MOTOR_ENCODER_PIN_1 = enc_pin_1;
    MOTOR_ENCODER_PIN_2 = enc_pin_2;
    TB6612_PWM_MOTOR_PIN = pwm_pin;
    TB6612_MOTOR_DIRECTION_PIN_1 = dir_pin_1;
    TB6612_MOTOR_DIRECTION_PIN_2 = dir_pin_2;

    motorTickConversion = 1.0;

    Kp = 1.0;
    Ki = 0.0;
    Kd = 0.0;

    openLoopK = 255.0;

    updateDelay = 5000;
    // flipEncoder = false;
#ifdef NOISE_FILTER_LOW_PASS
    speedSmoothK = 1.0;
#endif
}

void DCMotorController::flipEncoderLeads()
{
    int swap = MOTOR_ENCODER_PIN_2;
    MOTOR_ENCODER_PIN_2 = MOTOR_ENCODER_PIN_1;
    MOTOR_ENCODER_PIN_1 = swap;
}

void DCMotorController::flipMotorLeads()
{
    int swap = TB6612_MOTOR_DIRECTION_PIN_2;
    TB6612_MOTOR_DIRECTION_PIN_2 = TB6612_MOTOR_DIRECTION_PIN_1;
    TB6612_MOTOR_DIRECTION_PIN_1 = swap;
}

void DCMotorController::setGearboxProperties(double counts_per_rotation, double gear_ratio) {
    motorTickConversion = 2 * PI / (counts_per_rotation * gear_ratio);
}

void DCMotorController::setPid(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void DCMotorController::setPointBounds(double max_speed_rps, int min_usable_command, int max_usable_command)
{
    minCommand = min_usable_command;
    maxCommand = max_usable_command;
    maxSpeedRps = max_speed_rps;

    openLoopK = (double)max_usable_command / max_speed_rps;
}

void DCMotorController::begin()
{
    _clearVariables();

    pinMode(TB6612_PWM_MOTOR_PIN, OUTPUT);
    pinMode(TB6612_MOTOR_DIRECTION_PIN_1, OUTPUT);
    pinMode(TB6612_MOTOR_DIRECTION_PIN_2, OUTPUT);
    motorEncoder = new Encoder(MOTOR_ENCODER_PIN_1, MOTOR_ENCODER_PIN_2);
}
void DCMotorController::stop() {
    setMotorCommand(0);
}

void DCMotorController::reset() {
    _clearVariables();
    motorEncoder->write(0);
}

void DCMotorController::setSpeed(double rps) {
    pidEnabled = true;
    pidSpeedSetPoint = rps;
    if (rps > 0.0) {
        openLoopSetPoint = (int)(openLoopK * pidSpeedSetPoint + minCommand);
    }
    else if (rps < 0.0) {
        openLoopSetPoint = (int)(openLoopK * pidSpeedSetPoint - minCommand);
    }
    else {
        openLoopSetPoint = 0;
    }

    if (openLoopSetPoint > maxCommand) {
        openLoopSetPoint = maxCommand;
    }
    if (openLoopSetPoint < -maxCommand) {
        openLoopSetPoint = -maxCommand;
    }
}

void DCMotorController::update() {
    uint32_t current_time = micros();

    if (prevPidTimer > current_time) {  // if timer loops, reset timer
        prevPidTimer = current_time;
        return;
    }

    if ((current_time - prevPidTimer) < updateDelay) {
        return;
    }

    double dt = (double)(current_time - prevPidTimer) * 1E-6;
    prevPidTimer = micros();

    _computeSpeed(_computePosition(), dt);

    if (!pidEnabled) {
        return;
    }

    double error = pidSpeedSetPoint - currentMotorSpeed;
    double d_error = (error - pidPrevError) / dt;
    double i_error = pidSumError * dt;

    int motorOutput = openLoopSetPoint + (int)(Kp * error + Ki * i_error + Kd * d_error);

    if (abs(motorOutput) <= minCommand) {
        motorOutput = 0;
    }

    pidPrevError = error;
    pidSumError += error;

    setMotorCommand(motorOutput);
}

void DCMotorController::_clearVariables() {
    motorPosRad = 0.0;
    currentMotorCommand = 0;
    currentMotorSpeed = 0.0;

    prevTime = micros();
    prevMotorPos = 0.0;

#ifdef NOISE_FILTER_ROLLING_AVERAGE
    positionBufferIndex = 0;
    motorDeltaPositionRollingTotal = 0;
    timeRollingTotal = 0;
#endif

    pidEnabled = false;
    pidSpeedSetPoint = 0.0;
    openLoopSetPoint = 0;
    pidPrevError = 0.0;
    pidSumError = 0.0;
    prevPidTimer = micros();

#ifdef NOISE_FILTER_ROLLING_AVERAGE
    _clearBuffers();
#endif
}

void DCMotorController::_clearBuffers() {
    for (size_t i = 0; i < CONTROLLER_POSITION_BUFFER_SIZE; i++) {
        motorDeltaPositionBuffer[i] = 0.0;
        timeBuffer[i] = 0.0;
    }
}

void DCMotorController::setMotorCommand(int speed)
{
    currentMotorCommand = speed;
    if (speed > 0) {
        digitalWrite(TB6612_MOTOR_DIRECTION_PIN_1, LOW);
        digitalWrite(TB6612_MOTOR_DIRECTION_PIN_2, HIGH);
    }
    else if (speed < 0) {
        digitalWrite(TB6612_MOTOR_DIRECTION_PIN_1, HIGH);
        digitalWrite(TB6612_MOTOR_DIRECTION_PIN_2, LOW);
    }

    speed = abs(speed);
    if (speed > maxCommand) {
        speed = maxCommand;
    }
    analogWrite(TB6612_PWM_MOTOR_PIN, speed);
}

double DCMotorController::getPosition() {
    return currentMotorPosition;
}

int DCMotorController::getCurrentCommand() {
    return currentMotorCommand;
}

double DCMotorController::getCommandedSpeed() {
    return pidSpeedSetPoint;
}

double DCMotorController::getSpeed() {
    return currentMotorSpeed;
}

double DCMotorController::_computePosition()
{
    long motorPosition = motorEncoder->read();
    // if (flipEncoder) {
    //     motorPosition = -motorPosition;
    // }
    currentMotorPosition = (double)motorPosition * motorTickConversion;
    return currentMotorPosition;
}


void DCMotorController::_computeSpeed(double current_pos, double dt)
{
    double deltaPos = current_pos - prevMotorPos;
    prevMotorPos = current_pos;

#ifdef NOISE_FILTER_ROLLING_AVERAGE
    motorDeltaPositionRollingTotal -= motorDeltaPositionBuffer[positionBufferIndex];
    motorDeltaPositionBuffer[positionBufferIndex] = deltaPos;
    motorDeltaPositionRollingTotal += deltaPos;

    timeRollingTotal -= timeBuffer[positionBufferIndex];
    timeBuffer[positionBufferIndex] = dt;
    timeRollingTotal += dt;

    positionBufferIndex++;
    if (positionBufferIndex >= CONTROLLER_POSITION_BUFFER_SIZE) {
        positionBufferIndex = 0;
    }

    if (timeRollingTotal > 0) {
        currentMotorSpeed = motorDeltaPositionRollingTotal / timeRollingTotal;
    }
    else {
        currentMotorSpeed = 0.0;
    }

#endif  // NOISE_FILTER_ROLLING_AVERAGE
#ifdef NOISE_FILTER_LOW_PASS

    currentMotorSpeed += speedSmoothK * (deltaPos / dt - currentMotorSpeed);


#endif  // NOISE_FILTER_LOW_PASS
    // return currentMotorSpeed;
}
