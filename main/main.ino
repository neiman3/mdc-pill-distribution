// Alex
//brandon
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
#define PARAM_CTL_STEPPER_STEP 10           // number of steps per move cycle
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
  int pill_selected = -1;
  int platter_position;
  int platter_position_selected;
  int hole_position;
  // put your main code here, to run repeatedly:

  reset();

  // wait
  delay(PARAM_CTL_TIME_STEP);

  // Did user make any input?
  pill_selected = check_user_input();
  if (pill_selected != -1) {
    // user made an input- this is the main flow here.
    platter_position_selected = convert_to_position(pill_selected);
    while (true) {
      // Move to the pill
      platter_position = avg_read(PIN_POSITION_SENSOR);
      if (position_sensor_ok(platter_position, platter_position_selected)) {
        // exit this while loop
        break;
      }
      move_stepper(platter_position, platter_position_selected);
    }
    // Turn the vacuum on and move the actuator down
    vacuum_on();
    actuator_down();
    actuator_up();
    while (true) {
      // Move to the hole
      hole_position = calculate_closest_hole(platter_position);
      platter_position = avg_read(PIN_POSITION_SENSOR);
      if (position_sensor_ok(platter_position, hole_position)) {
        break;
      }
      move_stepper(platter_position, hole_position);
    }
    vacuum_off();
  }

}




int check_user_input() {
  // reads serial input and returns pill that the user wants to dispense.
  // returns a -1 for no input.
  if (Serial.available() > 0) {
    // user typed something in the serial terminal
    uint8_t incomingByte = Serial.read();
    if (incomingByte == '\n') {
      // newline- skip
    } else if(incomingByte=='1') {
      // pill 1
      return 1;
    } else if(incomingByte=='2') {
      // pill 2
      return 2;
    }
  }
  return -1;
}

int convert_to_position (int pill_number){
  // Converts a pill number to a position number.
  if (pill_number == 1){
    return 200;
  }
  else if (pill_number == 2){
    return 800;
  }
  return -1;
}

uint32_t avg_read(int PIN) {
  // average reading for a sensor
  uint32_t res = 0;
  for(int i=0; i<=PARAM_SENSOR_AVG_READ_CT; i++) {
    res += (analogRead(PIN) / PARAM_SENSOR_AVG_READ_CT);
    delay(PARAM_SENSOR_AVG_READ_DELAY);
  }
  return res;
}

bool position_sensor_ok(int position_sensor, int position_sensor_goal) {
  // returns whether the position sensor is close enough
  if (((position_sensor_goal - PARAM_CTL_HYSTERESIS) >= position_sensor) && ((position_sensor_goal + PARAM_CTL_HYSTERESIS) <= position_sensor)) {
    // it's between the hysteresis goal
    return true;
  }
  return false;
}

void move_stepper(int position_sensor, int position_sensor_goal) {
  // stepper step speed needs to be calculated
  int distance = PARAM_CTL_STEPPER_STEP;
  myStepper.setSpeed(10);
  if (position_sensor > position_sensor_goal) {
    // do notihng
  } else {
    distance = distance * -1;
  }
  myStepper.step(distance);
  return;
}


void vacuum_on() {
  digitalWrite(PIN_VACUUM_MOTOR, HIGH);
  digitalWrite(PIN_VACUUM_BREAKER, LOW);
  return;
}

void vacuum_off() {
  digitalWrite(PIN_VACUUM_MOTOR, LOW);
  digitalWrite(PIN_VACUUM_BREAKER, HIGH);
  delay(200); // time that vacuum breaker is open
  digitalWrite(PIN_VACUUM_BREAKER, LOW);
  return;
}

void lin_motor_move(uint8_t directn, uint8_t spd) {
  digitalWrite(PIN_LIN_MOTOR_EN, HIGH);
  // dumb move- no control logic
  // time step driven
  // directn 0: down
  // directn 1: up
  // spd: pwm speed range from 0 to 255
  if (directn) {
    // move up
    // ctl1 hi
    // ctl2 lo
    digitalWrite(PIN_LIN_MOTOR_CTL2, LOW);
    analogWrite(PIN_LIN_MOTOR_CTL1, spd);
  } else {
    digitalWrite(PIN_LIN_MOTOR_CTL1, LOW);
    analogWrite(PIN_LIN_MOTOR_CTL2, spd);
  }
}

void lin_motor_stop() {
  digitalWrite(PIN_LIN_MOTOR_EN, HIGH);
  digitalWrite(PIN_LIN_MOTOR_CTL1, HIGH);
  digitalWrite(PIN_LIN_MOTOR_CTL2, HIGH);
  delay(PARAM_LIN_MOTOR_TC);
  digitalWrite(PIN_LIN_MOTOR_EN, LOW);
  digitalWrite(PIN_LIN_MOTOR_CTL1, LOW);
  digitalWrite(PIN_LIN_MOTOR_CTL2, LOW);
  return;
  
}

void reset() {
  vacuum_off();
}

void actuator_down() {
  while (true) {
    if (!digitalRead(PIN_LIMSW_L)) {
      // Lower limit switch is triggered
      // stop motor
      lin_motor_stop();
      return;
    } else if (!digitalRead(PIN_LIMSW_H)) {
      // limit switch is triggered
      lin_motor_stop();
      return;
    } else if ((analogRead(PIN_VACUUM_MOTOR_SHUNT) > PARAM_VACUUM_MOTOR_CURRENT_THRESHOLD)) {
      // motor current exceeded steady-state threshold
      lin_motor_stop();
      return;
    } else {
      // ok to continue moving stepper
      lin_motor_move(0, 128);
    }
    delay(PARAM_CTL_TIME_STEP);
  }
}


void actuator_up() {
  while (true) {
    if (!digitalRead(PIN_LIMSW_L)) {
      // Lower limit switch is triggered
      // stop motor and move it down just a little bit to relieve the pressure on the gears
      lin_motor_stop();
      delay(PARAM_CTL_TIME_STEP);
      lin_motor_move(1, 127);
      delay(PARAM_CTL_TIME_STEP);
      lin_motor_stop();
      return;
    } else if (!digitalRead(PIN_LIMSW_H)) {
      // limit switch is triggered- upper
      lin_motor_stop();
      delay(PARAM_CTL_TIME_STEP);
      lin_motor_move(1, 127);
      delay(PARAM_CTL_TIME_STEP);
      lin_motor_stop();
      return;
    } else {
      // ok to continue moving stepper
      lin_motor_move(1, 128);
    }
    delay(PARAM_CTL_TIME_STEP);
  }
}

int calculate_closest_hole(int position) {
  return 123;
}
