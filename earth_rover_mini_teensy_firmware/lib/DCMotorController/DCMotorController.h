#include <Arduino.h>
#include <Encoder.h>

// #define NOISE_FILTER_ROLLING_AVERAGE
#define NOISE_FILTER_LOW_PASS
#define CONTROLLER_POSITION_BUFFER_SIZE 20

// Convention: rps = radians per second

class DCMotorController
{
    public:
        DCMotorController(int pwm_pin, int dir_pin_1, int dir_pin_2, int enc_pin_1, int enc_pin_2);
        void setGearboxProperties(double counts_per_rotation, double gear_ratio);
        void setPid(double Kp = 1.0, double Kd = 0.0, double Ki = 0.0);
        void setPointBounds(double max_speed_rps, int min_usable_command, int max_usable_command = 255);

        void flipMotorLeads();
        void flipEncoderLeads();

        void begin();
        void reset();

        void setMotorCommand(int speed);
        void setSpeed(double rps);
        double getPosition();
        double getSpeed();

        int getCurrentCommand();
        double getCommandedSpeed();

        void update();

        void stop();
        void waitForStop();

        bool pidEnabled;
        // bool flipEncoder;

        //
        // global properies
        //
        double motorTickConversion;
        double Kp, Ki, Kd;
        double updateDelay;  // microseconds

#ifdef NOISE_FILTER_LOW_PASS
        double speedSmoothK;
#endif

    private:
        int MOTOR_ENCODER_PIN_1;
        int MOTOR_ENCODER_PIN_2;
        int TB6612_PWM_MOTOR_PIN;
        int TB6612_MOTOR_DIRECTION_PIN_1;
        int TB6612_MOTOR_DIRECTION_PIN_2;

        //
        // Private motor properties
        //
        double openLoopK;
        double maxSpeedRps;
        int maxCommand;
        int minCommand;

        Encoder* motorEncoder;

        void _computeSpeed(double current_pos, double dt);
        double _computePosition();

        void _clearBuffers();
        void _clearVariables();

        //
        // live variables reset by _clearVariables
        //
        double motorPosRad;
        int currentMotorCommand;
        double currentMotorSpeed;
        double currentMotorPosition;

        uint32_t prevTime;
        double prevMotorPos;

#ifdef NOISE_FILTER_ROLLING_AVERAGE
        size_t positionBufferIndex;
        double motorDeltaPositionRollingTotal;
        double timeRollingTotal;
#endif

        // bool pidEnabled;  // global
        double pidSpeedSetPoint;
        int openLoopSetPoint;
        double pidPrevError;
        double pidSumError;
        uint32_t prevPidTimer;

        double motorDeltaPositionBuffer[CONTROLLER_POSITION_BUFFER_SIZE];
        double timeBuffer[CONTROLLER_POSITION_BUFFER_SIZE];
};
