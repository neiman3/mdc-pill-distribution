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

typedef enum {ST_RST, ST_WAIT, ST_MOVE_PLATTER, ST_DISPENSE_PILL} fsm_state_var;
typedef enum {ST_MOVE_TO_PILL, ST_LIN_DOWN, ST_LIN_UP, ST_MOVE_TO_HOLE} subfsm_dispense_state_var;


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
  
  uint32_t carousel_position;
  fsm_state_var state = ST_WAIT; // main FSM
  fsm_state_var next_state = ST_WAIT;
  subfsm_dispense_state_var subfsm_dispense = ST_MOVE_TO_PILL; // dispensing FSM
  uint8_t desired_platter_position;
  uint8_t lin_moving = 0;
  uint8_t step_moving = 0;
  uint8_t step_speed = 0;
  uint8_t vacuum_state = 0;
  uint16_t position_sensor_value;
  String msg = "default";
  uint8_t skip_delay = 0;

  // it's a fsm- it will do serial read and then go thru each case.
  // Calculation and IO should be instantaneous. At the end of the loop,
  // there will be a delay of PARAM_CTL_TIME_STEP.

  while (1) {

    // get user's input
    if (Serial.available() > 0) {
      // user typed something in the serial terminal
      uint8_t incomingByte = Serial.read();
      
      if (incomingByte == '\n') {
        // newline- skip
        
      } else if(incomingByte=='1') {
        desired_platter_position = 0;
      } else if(incomingByte=='2') {
        desired_platter_position = 200;
      } else if(incomingByte=='3') {
        desired_platter_position = 400;
      } else if(incomingByte=='4') {
        desired_platter_position = 600;
      } else if(incomingByte=='5') {
        desired_platter_position = 800;
      } else if(incomingByte=='d') { // D for dispense
        state = ST_MOVE_PLATTER;
      }
      
    }
  
  
    switch (state) {
      case ST_RST:
        msg = "Reset mode";
        break;
               
  
      case ST_MOVE_PLATTER:
      // move one step
        step_speed = min(map(desired_platter_position - position_sensor_value, 0, 1024, 0, PARAM_PROPORTIONAL_STEPPER_GAIN),1);
        skip_delay = 1;
        step_speed = 1;
        unsigned long tcurrent = millis();
        myStepper.setSpeed(step_speed);
        while (tcurrent + PARAM_CTL_TIME_STEP < millis()) {
          myStepper.step(10); // need to update direction here.
        }
        stepper_stop();
        state = next_state;
        break;
  
      default:
        // Just in case...this should never happen but it's good practice
         print_status(0, 0, 0, 0, 0, 0, 0, 0, 0, "Critical error");
        while (1) {
          // do nothing
        }
        break;
      }

   if (!skip_delay) {
    delay(PARAM_CTL_TIME_STEP);
    skip_delay = 0;
   }
   print_status(carousel_position, state, next_state, desired_platter_position, lin_moving, step_moving, step_speed, position_sensor_value, vacuum_state, msg);
   }
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

void vacuum_ctl(uint8_t vacuum_state) {
  if (vacuum_state) {
    vacuum_on();
  } else {
    vacuum_off();
  }
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

uint32_t avg_read(int PIN) {
  uint32_t res = 0;
  for(int i=0; i<=PARAM_SENSOR_AVG_READ_CT; i++) {
    res += (analogRead(PIN) / PARAM_SENSOR_AVG_READ_CT);
    delay(PARAM_SENSOR_AVG_READ_DELAY);
  }
  return res;
}

void stepper_stop() {
  digitalWrite(PIN_STEPPER_1, LOW);
  digitalWrite(PIN_STEPPER_2, LOW);
  digitalWrite(PIN_STEPPER_3, LOW);
  digitalWrite(PIN_STEPPER_4, LOW);
}

void print_status(uint32_t carousel_position, fsm_state_var state, fsm_state_var next_state, uint8_t desired_platter_position, uint8_t lin_moving, uint8_t step_moving, uint8_t step_speed, uint16_t position_sensor_value, uint8_t vacuum_state, String msg) {
  // Serial clear screen

  int ps_shunt = analogRead(PIN_VACUUM_MOTOR_SHUNT);
  int ps_limswL = digitalRead(PIN_LIMSW_L);
  int ps_limswH = digitalRead(PIN_LIMSW_H);
  int ps_pos = analogRead(PIN_POSITION_SENSOR);
  
  Serial.write(27);
  Serial.print("[2J");
  Serial.write(27);
  Serial.print("[H");

  Serial.println("MDC Pill Dispenser Status Monitor\n");
  Serial.print("Status- ");
  Serial.println(msg);
  Serial.println(" ");
  Serial.println("---------------- Variables ----------------");
  Serial.print("    Carousel position: "); Serial.println(carousel_position);
  Serial.print("       Main FSM state: "); Serial.println(state);
  Serial.print("          Pill select: "); Serial.println(desired_platter_position);
  Serial.print("        LIN is moving: "); Serial.println(lin_moving);
  Serial.print("    Stepper is moving: "); Serial.println(step_moving);
  Serial.print("        Stepper speed: "); Serial.println(step_speed);
  Serial.print("Position sensor value: "); Serial.println(position_sensor_value);
  Serial.print("          Vacuum pump: "); Serial.println(vacuum_state);
  
  Serial.print("\n\r----------------    I/O   ----------------\n\r");
  Serial.print("          Motor shunt: "); Serial.println(ps_shunt);
  Serial.print("       Limit switch L: "); Serial.println(ps_limswL);
  Serial.print("       Limit switch H: "); Serial.println(ps_limswH);
  Serial.print("      Position sensor: "); Serial.println(ps_pos);
  
}
