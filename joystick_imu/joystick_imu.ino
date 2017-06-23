#include "MPU6050_6Axis_MotionApps20.h"
#include "Joystick.h"

#define NUM_BUTTONS 4
#define NUM_LEDS 4

// arrays defining which pins are the various buttons and LEDs are connected to
uint8_t button_pins[NUM_BUTTONS] = {4, 5, 6, 7};
uint8_t led_pins[NUM_LEDS] = {8, 9, 16, 10};

// array defining which analog pins are connected for x and y input
uint8_t pot_pins[] = {A0, A1};

// pin to use for mode selection
#define MODE_PIN 14

#define JOYSTICK_DEBOUNCE 50

MPU6050 mpu;
bool init_success = false;
// packet size of FIFO on MPU6050
uint8_t packet_size;
uint8_t fifo_buffer[64];

Joystick_ joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  4, // number of buttons
  0, // number of hat switches
  true, // X axis available
  true, // Y axis available
  false, //Z axis available
  false, // Rx axis available
  false, // Ry axis available
  false, // Rz axis available
  false, // Rudder available
  true, // Throttle available
  false, // Accelerator available
  false, // Brake available
  false); // Steering available

void setup() {
  // iterate over button pins and configure all of them as input, with the internal pullup enabled
  // internal pullup eliminates the need to solder a external pullup
  for(uint8_t i = 0; i < NUM_BUTTONS; i++) {
    pinMode(button_pins[i], INPUT_PULLUP);
  }

  // iterate over LED pins and configure all of them as output
  for(uint8_t i = 0; i < NUM_LEDS; i++) {
    pinMode(led_pins[i], OUTPUT);
  }

  pinMode(MODE_PIN, INPUT_PULLUP);

  // initialise I2C bus
  Wire.begin();
  Wire.setClock(100000);

  // initialise Serial (UART), used to print info to connected PC
  Serial.begin(115200);
  Serial.println("Starting setup...");

  // initialise the MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Failed to detect MPU6050!");
  }

  uint8_t devStatus = mpu.dmpInitialize();

  // offsets calculated with https://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/
  mpu.setXAccelOffset(-237);
  mpu.setYAccelOffset(-942);
  mpu.setZAccelOffset(2208);
  mpu.setXGyroOffset(11);
  mpu.setYGyroOffset(39);
  mpu.setZGyroOffset(59);

  // checking for whether the MPU6050 was initialised successfully
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    init_success = true;
    packet_size = mpu.dmpGetFIFOPacketSize();

    // set FIFO output rate to 20Hz cause we cant handle so many samples anyway
    // commented out because its a undefined reference?
    //mpu.dmpSetFIFORate(20);
  }
  else {
    Serial.print("Failed to enable DMP on the MPU6050! Error code:");
    Serial.println(devStatus);
  }

  while(!init_success) {
    Serial.println("Failed to initialise MPU6050! Check and restart the Arduino.");
    delay(500);

    for(uint8_t i = 0; i < NUM_LEDS; i++) {
      digitalWrite(led_pins[i], !digitalRead(led_pins[i]));
    }
  }

  joystick.begin(false);
}

void loop() {
  static uint32_t joystick_debounce = 0;
  static uint16_t imu_scaling_factor = 10430;
  
  uint8_t mpuIntStatus = mpu.getIntStatus();
  uint8_t fifo_count = mpu.getFIFOCount();
  Quaternion q;
  
  float euler[3];

  uint8_t mode = digitalRead(MODE_PIN);

  if (mode == 1) {
    // in mode 1, use x pot to adjust sensitivity of imu
    imu_scaling_factor = 10430 / 1024 * analogRead(pot_pins[0]);
  }

  for(uint8_t i = 0; i < NUM_BUTTONS; i++) {
    uint8_t state = !digitalRead(button_pins[i]);
    joystick.setButton(i, state);
    digitalWrite(led_pins[i], state);
  }
  
  if ((mpuIntStatus & 0x10) || fifo_count == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }
  
  // if DMP ready
  if (mpuIntStatus & 0x02) {
    if (fifo_count < packet_size) {
      // not enough data in DMP FIFO, exit this loop and wait for next one
      return;
    }

    mpu.getFIFOBytes(fifo_buffer, packet_size);
    mpu.dmpGetQuaternion(&q, fifo_buffer);
    // euler[0]: yaw, euler[1]: pitch, euler[2]: roll
    mpu.dmpGetEuler(euler, &q);

    Serial.print("X: ");
    Serial.print(euler[0]);
    Serial.print("Y: ");
    Serial.print(euler[1]);
    Serial.print("Z: ");
    Serial.println(euler[2]);
  }

  if (mode == 0) {
    // mode 0: x & y relying on potentiometer, throttle relying on pitch
    joystick.setXAxisRange(0, 1024);
    joystick.setYAxisRange(0, 1024);
    joystick.setThrottleRange(-32768, 32767);
    
    joystick.setXAxis(analogRead(pot_pins[0]));
    joystick.setYAxis(analogRead(pot_pins[1]));
    // 20860 converts the (-pi, pi) value to (-32768, 32768) 
    joystick.setThrottle(euler[1]*imu_scaling_factor);
  }
  else {
    // mode 1: x and y relying on yaw and pitch respectively, throttle relying on Y pot
    joystick.setXAxisRange(-32768, 32767);
    joystick.setYAxisRange(-32768, 32767);
    joystick.setThrottleRange(0, 1024);
    
    joystick.setXAxis(euler[0]*imu_scaling_factor);
    joystick.setYAxis(euler[1]*imu_scaling_factor);
    joystick.setThrottle(analogRead(pot_pins[1]));
  }

  if (millis() - joystick_debounce > JOYSTICK_DEBOUNCE) {
    joystick.sendState();
    joystick_debounce = millis();
  }
}
