/*
 Controlling a servo position using a serial port command
 by zahra boroujeni 
 modified on 30 Nov 2015
*/

#include <Servo.h>
#include <MsTimer2.h>

Servo myservo;  // create servo object to control a servo
int state =0;

int potpin = 0;  // analog pin used to connect the potentiometer
int val=90;
int servo_pw=1500;    // variable to read the value from the analog pin
int last_pw=0;
String inputLight = "";         // a string to hold incoming data
String inputServo = "";         // a string to hold incoming data
String lastLight ="";
boolean stringComplete = false;  // whether the string is complete
boolean servoComplete=false;
boolean interrupt_flag=false;
boolean servo_control=false;
boolean ledState=false;
boolean flashingLedState=false;
boolean startFlashing=false;

int battery_voltage = 0; 

#define servo_pin 10
#define POWER_STATE_D0 8
#define POWER_STATE_D1 9
#define BUTTON_GREEN 4
#define BUTTON_RED 5

#define LED_STATUS_EN 3   //LED_STATUS_EN
#define LED_BRACK 11  //LED_LIGHTS_B0
#define LED_PARK_TAIL 13  //LED_LIGHTS_B1
#define LED_TURN_LEFT A0  //LED_LIGHTS_C0
#define LED_TURN_RIGHT A1  //LED_LIGHTS_C1
#define LED_BACKUP A2  //LED_LIGHTS_C2
#define LED_HEAD A3  //LED_LIGHTS_C3

#define ADC6_BAT_VOLT_LEVEL A6
#define ADC7_BAT_VOLT A7
#define AIN0_BAT_VOLT_LEVEL 9
#define AIN1_BAT_VOLT 10


volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

static int PCintMode[24];

typedef void (*voidFuncPtr)(void);

volatile static voidFuncPtr PCintFunc[24] = { 
  NULL };

volatile static uint8_t PCintLast[3];

/* ============================================
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */


// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  //servo motor angle : stright forward
  val=90;
  // attaches the servo on pin 9 to the servo object
  myservo.attach(servo_pin);
   
  // initialize serial:
  Serial.begin(115200);
  //myservo.writeMicroseconds(servo_pw);
  
  // reserve 200 bytes for the inputString:
  inputLight.reserve(100);
  inputServo.reserve(200);
  // power board control
  pinMode(POWER_STATE_D0, OUTPUT);
  pinMode(POWER_STATE_D1, OUTPUT);
  pinMode(BUTTON_GREEN, INPUT);
  pinMode(BUTTON_RED, INPUT);

  // LED signals
  pinMode(LED_STATUS_EN, OUTPUT);
  pinMode(LED_BRACK, OUTPUT);
  pinMode(LED_PARK_TAIL, OUTPUT);
  pinMode(LED_TURN_LEFT, OUTPUT);
  pinMode(LED_TURN_RIGHT, OUTPUT);
  pinMode(LED_BACKUP, OUTPUT);
  pinMode(LED_HEAD, OUTPUT);
  // Enable Power light
  digitalWrite(LED_STATUS_EN, LOW);
  // power 5 volt
  digitalWrite(POWER_STATE_D0, LOW);
  digitalWrite(POWER_STATE_D1, LOW); 

  resetLights();
  lastLight="";
  MsTimer2::stop();

  //check battery
  //analogReadResolution(8);
  delay(1000);
  checkBattery();

  // green button interrupt
  PCattachInterrupt(4, count, FALLING );

  ///////////////////////////////////////////////////////////////////////////

  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(105);
    mpu.setYGyroOffset(-24);
    mpu.setZGyroOffset(2);
    mpu.setXAccelOffset(-2187);
    mpu.setYAccelOffset(-1455);
    mpu.setZAccelOffset(1209); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
}

void loop() {
  //control Power board switch between 5 volt and 12 volt for the brushless motor
 if (interrupt_flag==true)
    powerBoard();
  // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        
    } 
     serialEvent(); //read serial port commands from odroid
        if (stringComplete) 
        {
          lightControl(); // control lights of the car, this function should call befor control motor
          servoControl(); // control servo motor
          // inputServo = "";
          inputLight = "";
          stringComplete = false;
        }
      // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");//yaw
            Serial.println(ypr[0] * 180/M_PI);
            //Serial.print("pitch\t");
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print("roll\t");
            //Serial.println(ypr[2] * 180/M_PI);
        #endif
    }
    checkBattery();
    turnOffCar();
 
}

/* Check voltage of the battery */
void checkBattery()
{
  // read the input on A7 at default resolution (10 bits)  
  battery_voltage = analogRead(A7);
 
  //Serial.print(battery_voltage);
  if ((battery_voltage<770) && (battery_voltage>100)) //check also if there is a battery availabe! //14.8 volt
  {
    if (startFlashing==false)
    {
      MsTimer2::set(500, lowBattery); // 0.5 s period
      MsTimer2::start();
      startFlashing=true;
    }
    if (battery_voltage<740) //14.5 volt
    {
      Serial.print("Please charge the Battery!"); 
      startFlashing==false;
      digitalWrite(LED_STATUS_EN, LOW);
      digitalWrite(POWER_STATE_D0, LOW);
      digitalWrite(POWER_STATE_D1, HIGH);
    }
  }
}
void lowBattery()
{
  flashingLedState=!flashingLedState;
  digitalWrite(LED_STATUS_EN, flashingLedState);
}
/* Control power board*/
void powerBoard(){
  interrupt_flag=false;
  if (state%2==0)//Yellow
  {
    digitalWrite(POWER_STATE_D0, LOW);
    digitalWrite(POWER_STATE_D1, LOW); 

  } else if (state%2==1) //green
  {
    digitalWrite(POWER_STATE_D0, HIGH);
    digitalWrite(POWER_STATE_D1, LOW);
  } 
//else if (state%3==2)//Red
//  {
//    digitalWrite(POWER_STATE_D0, LOW);
//    digitalWrite(POWER_STATE_D1, HIGH); 
//  }
}
void count()
{
  state++;
  interrupt_flag=true;
}
void turnOffCar()
{
  int carShutdown=digitalRead(BUTTON_RED);
  if (carShutdown==0)
  {
    digitalWrite(POWER_STATE_D0, HIGH);
    digitalWrite(POWER_STATE_D1, HIGH); 
  }
  //MsTimer2::stop();
}
/* Control lights*/
void servoControl(){
    if (inputServo=="en")
    {
      myservo.attach(servo_pin);
      //Serial.println("enable");
      //Serial.println(inputString);
    }
    else if (inputServo=="di")
    {
      myservo.detach();
      //Serial.println("detach");
      //Serial.println(inputString);
    }
    else
    {
      //Serial.println(inputString);
      val=inputServo.toInt();
      if ((val<=180) && (val>=0))
      {
        servo_pw = map(val, 0, 180, 700, 1900);     // scale it to use it with the servo (value between 0 and 180)
        //if (last_pw!=servo_pw)
          myservo.writeMicroseconds(servo_pw); 
        last_pw=servo_pw; 
      }
    }
} 
/* Control lights*/
void lightControl(){ 
  //resetLights();
  if (inputLight=="Lle\r")
  {
     if (lastLight!="Lle\r")
      { 
        digitalWrite(LED_TURN_RIGHT, LOW);
        MsTimer2::set(500, flashLeftLight); // 0.5 s period
        MsTimer2::start();
      }
  }  
  else if (inputLight=="Lri\r")
  {
    if (lastLight!="Lri\r")
      { 
        digitalWrite(LED_TURN_LEFT, LOW);
        MsTimer2::set(500, flashRightLight); // 0.5 s period
        MsTimer2::start();
      }
  }
  else if (inputLight=="Lstop\r")
  {
    if (lastLight!="Lstop\r")
      digitalWrite(LED_BRACK, HIGH);
  }
  else if ((inputLight=="Lpa\r") || (inputLight=="Lta\r"))
  {
    if (lastLight!="Lpa\r")
    {
      if (lastLight!="Lta\r")
        digitalWrite(LED_PARK_TAIL, HIGH);
    }
  }
  else if (inputLight=="Lre\r")
  {
    if (lastLight!="Lre\r")
      digitalWrite(LED_BACKUP, HIGH);
  }
  else if (inputLight=="Lfr\r")
  {
    if (lastLight!="Lfr\r")
      digitalWrite(LED_HEAD, HIGH);
  }
  else if (inputLight=="LdiL\r")
  {
    if (lastLight!="LdiL\r")
      resetLights();
  }
  
  lastLight=inputLight;
  
} 
void resetLights()
{
  startFlashing==false;
  MsTimer2::stop(); //stop flashing lights
  digitalWrite(LED_TURN_LEFT, LOW);
  digitalWrite(LED_TURN_RIGHT, LOW);
  digitalWrite(LED_BRACK, LOW);
  digitalWrite(LED_PARK_TAIL, LOW);
  digitalWrite(LED_BACKUP, LOW);
  digitalWrite(LED_HEAD, LOW);
}
void flashLeftLight()
{
  ledState=!ledState;
  digitalWrite(LED_TURN_LEFT, ledState);
}
void flashRightLight()
{
  ledState=!ledState;
  digitalWrite(LED_TURN_RIGHT, ledState);
}
/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) 
  {
    //Serial.println("here");
    // get the new byte:
    char inChar = (char)Serial.read();
    //Serial.println(inChar);
    // add it to the inputString:
    
    if (inChar == 'L') 
    {
      servoComplete = true;
       //Serial.println("true");
    }
   
    if (servoComplete==false)
      inputServo += inChar;
    else 
      inputLight += inChar;

     if (inChar == 'S') 
     {
      inputServo="";
      inputLight="";
      servoComplete = false;
       //Serial.println("true");
     }
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if ((inChar == '\r')&&(servoComplete == true))
    {
      stringComplete = true;
       //Serial.println("true");
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////
/*
 * attach an interrupt to a specific pin using pin change interrupts.
 */
 void PCattachInterrupt(uint8_t pin, void (*userFunc)(void), int mode) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t slot;
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

// -- Fix by Baziki. In the original sources it was a little bug, which cause analog ports to work incorrectly.
  if (port == 1) {
     slot = port * 8 + (pin - 14);
  }
  else {
     slot = port * 8 + (pin % 8);
  }
// --Fix end
  PCintMode[slot] = mode;
  PCintFunc[slot] = userFunc;
  // set the mask
  *pcmask |= bit;
  // enable the interrupt
  PCICR |= 0x01 << port;
}

void PCdetachInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

  // disable the mask.
  *pcmask &= ~bit;
  // if that's the last one, disable the interrupt.
  if (*pcmask == 0) {
    PCICR &= ~(0x01 << port);
  }
}

// common code for isr handler. "port" is the PCINT number.
// there isn't really a good way to back-map ports and masks to pins.
static void PCint(uint8_t port) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;

  // get the pin states for the indicated port.
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;
  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }
  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = port * 8 + i;
      // Trigger interrupt if mode is CHANGE, or if mode is RISING and
      // the bit is currently high, or if mode is FALLING and bit is low.
      if ((PCintMode[pin] == CHANGE
          || ((PCintMode[pin] == RISING) && (curr & bit))
          || ((PCintMode[pin] == FALLING) && !(curr & bit)))
          && (PCintFunc[pin] != NULL)) {
        PCintFunc[pin]();
      }
    }
  }
}


SIGNAL(PCINT0_vect) {
  PCint(0);
}
SIGNAL(PCINT1_vect) {
  PCint(1);
}
SIGNAL(PCINT2_vect) {
  PCint(2);
}
