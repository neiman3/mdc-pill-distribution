
#include <Stepper.h>

#define PIN_VACUUM_MOTOR        11
#define PIN_VACUUM_BREAKER      12
#define PIN_VACUUM_MOTOR_SHUNT  A0
#define PIN_STEPPER_1           2
#define PIN_STEPPER_2           3
#define PIN_STEPPER_3           4
#define PIN_STEPPER_4           5
#define PIN_LIMSW_L             6
#define PIN_LIMSW_H             7
#define PIN_LIN_MOTOR_EN        8
#define PIN_LIN_MOTOR_CTL1      9
#define PIN_LIN_MOTOR_CTL2      10
#define PIN_POSITION_SENSOR     A1

#define PARAM_VACUUM_BREAKER_DELAY 200      // time in ms to break vacuum
#define PARAM_SENSOR_AVG_READ_CT 10.0       // number of averages for sensor reading
#define PARAM_SENSOR_AVG_READ_DELAY 10      // delay between sensor readings
#define PARAM_VACUUM_MOTOR_CURRENT_THRESHOLD 1024 // current threshold for pill pickup
#define PARAM_LIN_MOTOR_TC 100              // approx. time constant of dc motor- used for stopping
#define PARAM_CTL_TIME_STEP 100             // time step in ms for control system logic
#define PARAM_STEPPER_STEPS 2048            // number of steps in stepper
#define PARAM_CTL_HYSTERESIS 3              // What's "close enough" for position sensor value
#define PARAM_PROPORTIONAL_STEPPER_GAIN 10  // gain for digital control system

// globals
Stepper myStepper(PARAM_STEPPER_STEPS, PIN_STEPPER_1, PIN_STEPPER_2, PIN_STEPPER_3, PIN_STEPPER_4);




void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  pinMode(PIN_VACUUM_MOTOR, OUTPUT);
  pinMode(PIN_VACUUM_BREAKER, OUTPUT);
  pinMode(PIN_STEPPER_1, OUTPUT);
  pinMode(PIN_STEPPER_2, OUTPUT);
  pinMode(PIN_STEPPER_3, OUTPUT);
  pinMode(PIN_STEPPER_4, OUTPUT);
  pinMode(PIN_LIMSW_L, INPUT_PULLUP);
  pinMode(PIN_LIMSW_H, INPUT_PULLUP);
  pinMode(PIN_LIN_MOTOR_EN, OUTPUT);
  pinMode(PIN_LIN_MOTOR_CTL1, OUTPUT);
  pinMode(PIN_LIN_MOTOR_CTL2, OUTPUT);
  pinMode(PIN_POSITION_SENSOR, INPUT);
  pinMode(PIN_VACUUM_MOTOR_SHUNT, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

}
