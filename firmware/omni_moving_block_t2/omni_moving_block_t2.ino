#include <SimpleFOC.h>
#include "PWMrelay.h"

#define A 0
#define B 1

//block ID is marked on hardware, change when coreprogramming the corresponding block
int block_id = A;

//pwm pin for the bridge heating coil
PWMrelay bridge_coil_a(PC6);
//bridge coil pwm - low side
int bridge_coil_b = PC7;
//bridge coil controller - enable pin
int coil_en = PA12;
//bridge temp sensor AD
int bridge_ad = PA4;

int coil_heating_flag = 0;

//BLDC motor & driver & encoder instance - mot A
BLDCMotor motor_1 = BLDCMotor(5);
BLDCDriver3PWM driver_1 = BLDCDriver3PWM(PA5, PA6, PA7, PB1);
Encoder encoder_1 = Encoder(PB8, PB9, 4096);
void doA1() {
  encoder_1.handleA();
}
void doB1() {
  encoder_1.handleB();
}

//BLDC motor & driver & encoder instance - mot B
BLDCMotor motor_2 = BLDCMotor(5);
BLDCDriver3PWM driver_2 = BLDCDriver3PWM(PA11, PA10, PA9, PB0);
Encoder encoder_2 = Encoder(PB10, PC12, 4096);
void doA2() {
  encoder_2.handleA();
}
void doB2() {
  encoder_2.handleB();
}

// wireless serial
int tx2 = PA0;
int rx2 = PA1;
HardwareSerial hs1(rx2, tx2);

////////////////////////////////////////////////////////////////////////////////////////////////////

union float2byte
{
  float f;
  char c[4];
};

//class to receive & decode command from PC side
class command_receiver
{
  public:
    char package_head[2];
    char received_arr_tmp[18];
    float2byte vel_arr[4];
    int coil_ctr[2];

    command_receiver(char *pkg_head);
    int serial_upadate(HardwareSerial &serial);
};

command_receiver::command_receiver(char *pkg_head)
{
  //define the package head
  package_head[0] = 's';
  package_head[1] = 't';

  for (int s = 0; s < 4; s++)
    vel_arr[s].f = 0;

  coil_ctr[0] = 0;
  coil_ctr[1] = 0;
}

int command_receiver::serial_upadate(HardwareSerial &serial)
{

  int update_flag = 0;
  //seeking the package head
  if (serial.available())
  {
    char tmp = serial.read();
    if (tmp == package_head[0])
    {
      //copy target velocity into union
      for (int s = 0; s < 4; s++)
        vel_arr[s].f = serial.parseFloat();

      //copy desired coil state
      for (int s = 0; s < 2; s++)
        coil_ctr[s] = serial.parseFloat();

      update_flag = 1;
    }
  }
  return update_flag;
}

//instance a global command receiver
command_receiver block_receiver("st");

////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() 
{
  //motor controller init
  //encoder.quadrature = Quadrature::OFF;
  //encoder.pullup = Pullup::USE_INTERN;
  encoder_1.init();
  encoder_1.enableInterrupts(doA1, doB1);
  motor_1.linkSensor(&encoder_1);

  driver_1.voltage_power_supply = 12;
  driver_1.voltage_limit = 12;
  driver_1.init();
  motor_1.linkDriver(&driver_1);

  //encoder.quadrature = Quadrature::OFF;
  //encoder.pullup = Pullup::USE_INTERN;
  encoder_2.init();
  encoder_2.enableInterrupts(doA2, doB2);
  motor_2.linkSensor(&encoder_2);

  driver_2.voltage_power_supply = 12;
  driver_2.voltage_limit = 12;
  driver_2.init();
  motor_2.linkDriver(&driver_2);

  motor_1.voltage_sensor_align = 3;
  motor_1.torque_controller = TorqueControlType::voltage;
  motor_1.controller = MotionControlType::velocity;

  motor_1.PID_velocity.P = 0.14;
  motor_1.PID_velocity.I = 13;
  motor_1.PID_velocity.D = 0;
  motor_1.PID_velocity.output_ramp = 1000;
  motor_1.LPF_velocity.Tf = 0.001;
  motor_1.voltage_limit = 9;
  motor_1.current_limit = 2;

  motor_2.voltage_sensor_align = 3;
  motor_2.torque_controller = TorqueControlType::voltage;
  motor_2.controller = MotionControlType::velocity;

  motor_2.PID_velocity.P = 0.14;
  motor_2.PID_velocity.I = 13;
  motor_2.PID_velocity.D = 0;
  motor_2.PID_velocity.output_ramp = 1000;
  motor_2.LPF_velocity.Tf = 0.001;
  motor_2.voltage_limit = 9;
  motor_2.current_limit = 2;

  //wireless serial setup
  hs1.begin(115200);

  //align the motor and the encoder
  motor_1.init();
  motor_1.initFOC();

  motor_2.init();
  motor_2.initFOC();

  //bridge init
  pinMode(bridge_coil_b, OUTPUT);
  pinMode(coil_en, OUTPUT);

  //setup soft pwm for the bridge coil
  bridge_coil_a.setLevel(HIGH);
  bridge_coil_a.setPeriod(200);
  bridge_coil_a.setPWM(0);
  //coil - low side init
  digitalWrite(bridge_coil_b, 0);
  digitalWrite(coil_en, 0);

  _delay(1000);

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() 
{
  //read command from wireless serial
  int update_flag = block_receiver.serial_upadate(hs1);

  //set target velocity for motor
  motor_1.loopFOC();
  motor_1.move(block_receiver.vel_arr[block_id*2].f);

  motor_2.loopFOC();
  motor_2.move(block_receiver.vel_arr[1 + block_id*2].f);

  //set bridge state
  coil_heating_flag = block_receiver.coil_ctr[block_id];

  if (coil_heating_flag == 1)
  {
    int temp_raw = analogRead(bridge_ad);
    
    //uncomment this [if] to enable a bang-bang controller for the bridge temperature
    //however, the temperature sensor has a low respond speed when the bridge is cooling, this can lead to slow reheating in any soft-rigid-soft situations
    //where the bridge has already cooled down, but the sensor reading is still high enough to prevent the coil from a new heating command
    if (1)//temp_raw < 225)
    {
      bridge_coil_a.setPWM(100);
      digitalWrite(bridge_coil_b, 0);
      digitalWrite(coil_en, 1);
    }
    else
    {
      bridge_coil_a.setPWM(0);
      digitalWrite(bridge_coil_b, 0);
      digitalWrite(coil_en, 0);
    }
  }
  else
  {
    bridge_coil_a.setPWM(0);
    digitalWrite(bridge_coil_b, 0);
    digitalWrite(coil_en, 0);
  }

  //loop bridge coil's software pwm
  bridge_coil_a.tick();
}
