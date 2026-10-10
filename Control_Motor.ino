/*
  2-Wheel Robot Motor Control using SimpleFOC for ESP32
  Motor Parameters:
  - Pole pairs: 7
  - Supply voltage: 15V - 21V
  - Max motor voltage: 12V
  - Max velocity: 1.0 m/s
  - Velocity control using PID 
  - Closed-loop control with AS5600 sensors
  - Commands: Forward (F), Backward (B), Turn Left (L), Turn Right (R), Stop (S)
*/

#include <SimpleFOC.h>

// Define Parameter

const int   pole_pairs              = 7;       // (unit: pair)  
const float voltage_power_supply    = 17.5;    // (unit: V)
const float voltage_limit           = 10;      // (unit V)
const float velocity_limit          = 40;      // (unit: rad/s)

const float DEADZONE  = 5;  
const float STOP_ZONE = 1.0;  

MagneticSensorI2C sensor  = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

BLDCMotor motor  = BLDCMotor(pole_pairs); 
BLDCMotor motor1 = BLDCMotor(pole_pairs);
BLDCDriver3PWM driver  = BLDCDriver3PWM(32, 33, 25, 22);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

float target_left = 0;
float target_right = 0;

float applyDeadzone(float v) {
  if (abs(v) < STOP_ZONE) return 0;
  if (v > 0 && v < DEADZONE) return DEADZONE;
  if (v < 0 && v > -DEADZONE) return -DEADZONE;
  return v;
}

void TurnRight(float spd)   { target_left = spd; target_right = spd; }
void TurnLeft(float spd)  { target_left = -spd; target_right = -spd; }
void Forward (float spd)  { target_left = -spd; target_right = spd; }
void Backward (float spd) { target_left = spd;  target_right = -spd; }
void Stop()               { target_left = 0; target_right = 0; }

void setup() {
  Serial.begin(115200);

  I2Cone.begin(19, 18, 400000);
  I2Ctwo.begin(23, 5, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);

  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  driver.voltage_power_supply  = voltage_power_supply;
  driver.init();
  driver1.voltage_power_supply = voltage_power_supply;
  driver1.init();

  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);

  motor.foc_modulation  = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.controller      = MotionControlType::velocity;
  motor1.controller     = MotionControlType::velocity;

  motor.PID_velocity.P  = 0.1; 
  motor.PID_velocity.I  = 1; 
  motor.PID_velocity.D  = 0;

  motor1.PID_velocity.P = 0.1; 
  motor1.PID_velocity.I = 1; 
  motor1.PID_velocity.D = 0;

  motor.LPF_velocity.Tf = 0.01; 
  motor1.LPF_velocity.Tf = 0.01;

  motor.voltage_limit   = voltage_limit; 
  motor1.voltage_limit  = voltage_limit;

  motor.velocity_limit  = velocity_limit; 
  motor1.velocity_limit = velocity_limit;

  motor.init();
  motor1.init();

  motor.initFOC();
  motor1.initFOC();

  Serial.println(F("Motor ready."));
  // Serial.println("F: Forward | B: Backward | L: Left | R: Right | S: Stop");
}

void loop() {
  motor.loopFOC();
  motor1.loopFOC();

  float left_cmd  = applyDeadzone(target_left);
  float right_cmd = applyDeadzone(target_right);

  motor.move(left_cmd);
  motor1.move(right_cmd);

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    char dir = cmd.charAt(0);
    float value = cmd.substring(1).toFloat();

    switch (dir) {
      case 'F': Forward(value); break;
      case 'B': Backward(value); break;
      case 'L': TurnLeft(value); break;
      case 'R': TurnRight(value); break;
      case 'S': Stop(); break;
    }
  }
}