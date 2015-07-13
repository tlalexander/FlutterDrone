#ifndef QUADARDU
#define QUADARDU

#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

#include "Flutter.h"

boolean _running = false;
Flutter flutter;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define GYRO_INTERRUPT 4 //gyro interrupt pin
bool dmpReady = false;  // set true if DMP init was successful

//#define OUTPUT_READABLE_YAWPITCHROLL

#define DEBUGG


/*  Arduino Pin configuration
 *  
 */

#define ESC_A 6
#define ESC_B 7
#define ESC_C 8
#define ESC_D 9


/* ESC configuration
 *
 */

#define ESC_MIN 0
#define ESC_MAX 800
#define ESC_TAKEOFF_OFFSET 30
#define ESC_ARM_DELAY 5000

/* RC configuration
 * 
 */

#define RC_HIGH_CH 0
#define RC_LOW_CH 255

#define RC_ROUNDING_BASE 5

/*  PID configuration
 *  
 */

#define PITCH_P_VAL 0.5
#define PITCH_I_VAL 0
#define PITCH_D_VAL 1

#define ROLL_P_VAL 2
#define ROLL_I_VAL 5
#define ROLL_D_VAL 1

#define YAW_P_VAL 2
#define YAW_I_VAL 5
#define YAW_D_VAL 1


/* Flight parameters
 *
 */

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 20
#define PID_ROLL_INFLUENCE 20
#define PID_YAW_INFLUENCE 20


/*  MPU variables
 *
 */

MPU6050 mpu;                           // mpu interface object


uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 

Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};

volatile bool mpuInterrupt = false;    //interrupt flag

/* Interrupt lock
 *
 */
 
boolean interruptLock = false;

/*  RC variables
 *
 */

float ch1, ch2, ch3, ch4, ch5 = 1000;         // RC channel inputs


/*  Motor controll variables
 *
 */

int velocity;                          // global velocity

float bal_ac, bal_bd;                 // motor balances can vary between -100 & 100
float bal_axes;                       // throttle balance between axes -100:ac , +100:bd

int va, vb, vc, vd;                    //velocities
int v_ac, v_bd;                        // velocity of axes

//Servo a,b,c,d;

/*  PID variables
 *
 */

PID pitchReg(&ypr[1], &bal_bd, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg(&ypr[2], &bal_ac, &ch1, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawReg(&ypr[0], &bal_axes, &ch4, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);


/*  Filter variables
 *
 */
 
float ch1Last, ch2Last, ch4Last, velocityLast;

/*  Setup function
 *
 */

void setup(){
 
  Serial.begin(115200);
  SerialUSB.begin(115200);            
  //while(!SerialUSB);
 // delay(8000);
 //SerialUSB.flush();
  
  flutter.band = BAND;
	flutter.setNetworkName("Home network");
	SerialUSB.println("Initializing...");
        Serial.println("Initializing...");

	if (flutter.init() == true)
	{
		SerialUSB.println("Init success.");
                Serial.println("Init success.");
		flutter.ledLightShow();
		//delay(500);
		//analogWrite(LED_R, 128);
	}
	else
	{
		flutter.setLED(RED);
		SerialUSB.println("Init failed.");

		while (true);
	}

	flutter.setAddress(2);
	flutter.connect(1); //form a network with this and one other device


                          
  initMPU();
  initESCs();
  initBalancing();
  initRegulators();
  
}

/* loop function
 *
 */

void loop(){
  
  		if(flutter.getState()!=NORMAL_OPERATION) //if we aren't synchronized with another radio, just loop and blink lights.
		{
			if(millis()%400<200)
			{
				flutter.setLED(RED);
			}
			
			else
			{
				flutter.setLED(BLUE);
			}
			
		}
  
  
  
 //read the radio to see if there is data available
	if (flutter.dataAvailable() > 0)
	{
		int packetSize = flutter.nextPacketLength();
		byte array[packetSize];
		flutter.readBytes(array, packetSize);
		byte steer = array[6];
		byte throttle = array[7];
		/*Serial.print("Steer: 0x");
		Serial.print(steer,HEX);
		Serial.print(", Throttle: 0x");
		Serial.println(throttle,HEX);*/
		
		
		byte j1 = steer;
		byte j2 = throttle;

                if(throttle<128) throttle=128;
                
                throttle-=128;

                ch1=array[8];
                ch4=array[6];
                ch2=array[9];
                
                
                ch3=throttle; //throttle
                
		//make our LED do nice colors
		j1 = abs((int)j1 - 128) * 1.9f;

		if (j1 < 5)
		{
			j1 = 0;
		}

		if (j2 < 128)
		{
			flutter.setLED(128 - j2, 0, j1);
		}
		else
		{
			flutter.setLED(0, j2 - 128, j1);
		}

		//delay(10);
		flutter.nextPacket();
	}

  while(flutter.getsystemBusActive()==true);
  getYPR();                          
  computePID();
  calculateVelocities();
  updateMotors();
  while(flutter.getsystemBusActive()!=true);
  //delay(5);
  
  
  
}

/*  computePID function
 *
 *  formats data for use with PIDLib
 *  and computes PID output
 */

void computePID(){

 // ch1=ch2=ch4=1500;
  
  ch1 = floor(ch1/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch2 = floor(ch2/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch4 = floor(ch4/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  
    
    

  ch2 = map(ch2, RC_LOW_CH, RC_HIGH_CH, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, RC_LOW_CH, RC_HIGH_CH, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, RC_LOW_CH, RC_HIGH_CH, YAW_MIN, YAW_MAX);
  

  
  if((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;
  
  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;
  

  
   
  
  ypr[0] = ypr[0] * 180/M_PI;
  ypr[1] = ypr[1] * 180/M_PI;
  ypr[2] = ypr[2] * 180/M_PI;
  
  
  
  if(abs(ypr[0]-yprLast[0])>30) ypr[0] = yprLast[0];
  if(abs(ypr[1]-yprLast[1])>30) ypr[1] = yprLast[1];
  if(abs(ypr[2]-yprLast[2])>30) ypr[2] = yprLast[2];
  
  yprLast[0] = ypr[0];
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];
  
    /* SerialUSB.print("Millis:,");
    SerialUSB.print(millis());
    
     SerialUSB.print(",Ay:,");
    SerialUSB.print(ay);
  
    SerialUSB.print(",  YPR 0:, ");
  SerialUSB.print(ypr[0]);
  
  SerialUSB.print(",  YPR 1:, ");
  SerialUSB.print(ypr[1]);
  
  SerialUSB.print(",  YPR 2:, ");
  SerialUSB.println(ypr[2]);*/
  

  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();
  
  

}

/*  getYPR function
 *
 *  gets data from MPU and
 *  computes pitch, roll, yaw on the MPU's DMP
 */

void getYPR(){
  
  // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    while (!mpuInterrupt && fifoCount < packetSize) {
        }
  
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
     // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //SerialUSB.println("FIFO overflow!");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            
            SerialUSB.print("ypr\t");
            SerialUSB.print(ypr[0] * 180/M_PI);
            SerialUSB.print("\t");
            SerialUSB.print(ypr[1] * 180/M_PI);
            SerialUSB.print("\t");
            SerialUSB.println(ypr[2] * 180/M_PI);
        #endif
      

    
    }

}

/*  calculateVelocities function
 *  
 *  calculates the velocities of every motor
 *  using the PID output
 */

void calculateVelocities(){

  //ch3=1000;

  ch3 = floor(ch3/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  velocity = map(ch3, 0, 255, ESC_MIN, ESC_MAX);
  


  if((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;
  
  velocityLast = velocity;
  
  v_ac = (abs(-100+bal_axes)/100)*velocity;
  v_bd = ((100+bal_axes)/100)*velocity;
  
  va = ((100+bal_ac)/100)*v_ac;
  vb = ((100+bal_bd)/100)*v_bd;
  
  vc = (abs((-100+bal_ac)/100))*v_ac;
  vd = (abs((-100+bal_bd)/100))*v_bd;
  
  SerialUSB.print("Velocity: ");
  SerialUSB.print(velocity);
  
  SerialUSB.print(" bal_axes: ");
  SerialUSB.print(bal_axes);
  
  SerialUSB.print(" bal_ac: ");
  SerialUSB.print(bal_ac);
  
  SerialUSB.print(" bal_bd: ");
  SerialUSB.print(bal_bd);
  
  SerialUSB.print(" va: ");
  SerialUSB.print(va);
  
  SerialUSB.print(" vb: ");
  SerialUSB.print(vb);
  
  SerialUSB.print(" vc: ");
  SerialUSB.print(vc);
  
  SerialUSB.print(" vd: ");
  SerialUSB.println(vd);
  
//  SerialUSB.println(bal_bd);
  
  if(velocity < ESC_TAKEOFF_OFFSET){
  
    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;
  
  }
  
}

inline void updateMotors(){

  //a.write(va);
  //c.write(vc);
  //b.write(vb);
  //d.write(vd);
  
    digitalWrite(ESC_A, HIGH);
  delayMicroseconds(1000+va);
  digitalWrite(ESC_A, LOW);
  
  digitalWrite(ESC_B, HIGH);
  delayMicroseconds(1000+vb);
  digitalWrite(ESC_B, LOW);
  
  digitalWrite(ESC_C, HIGH);
  delayMicroseconds(1000+vc);
  digitalWrite(ESC_C, LOW);
  
  digitalWrite(ESC_D, HIGH);
  delayMicroseconds(1000+vd);
  digitalWrite(ESC_D, LOW);

}

inline void arm(){

int targetTime = millis()+ESC_ARM_DELAY;
 
 while(millis()<targetTime)
 {
  digitalWrite(ESC_A, HIGH);
  delayMicroseconds(1000);
  digitalWrite(ESC_A, LOW);
  digitalWrite(ESC_B, HIGH);
  delayMicroseconds(1000);
  digitalWrite(ESC_B, LOW);
  digitalWrite(ESC_C, HIGH);
  delayMicroseconds(1000);
  digitalWrite(ESC_C, LOW);
  digitalWrite(ESC_D, HIGH);
  delayMicroseconds(1000);
  digitalWrite(ESC_D, LOW);
 }
  
  
  


}

inline void dmpDataReady() {
    mpuInterrupt = true;
}


void initMPU(){
  
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    pinMode(GYRO_INTERRUPT,INPUT);
    attachInterrupt(GYRO_INTERRUPT, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    SerialUSB.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    
  } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        SerialUSB.print(F("DMP Initialization failed (code "));
        SerialUSB.print(devStatus);
        SerialUSB.println(F(")"));
    }
  
    mpu.setXAccelOffset(-879);
  mpu.setYAccelOffset(-1470);
  mpu.setZAccelOffset(951);
  mpu.setXGyroOffset(118);
  mpu.setYGyroOffset(12);
  mpu.setZGyroOffset(16);
}

inline void initESCs(){

  pinMode(ESC_A, OUTPUT);
  pinMode(ESC_B, OUTPUT);
  pinMode(ESC_C, OUTPUT);
  pinMode(ESC_D, OUTPUT);
  
  //delay(100);
  
  arm();

}

inline void initBalancing(){

  bal_axes = 0;
  bal_ac = 0;
  bal_bd = 0;

}

inline void initRegulators(){

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  
  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

}



void button1()
{
	interrupts();
	int val = digitalRead(BUTTON1); //top button

	if (val == HIGH)
	{
		// _button1=255;
	}
	else
	{
		//  _button1=0;
	}

// buttonsChanged=true;
}

void button2()
{
	interrupts();
	int val = digitalRead(BUTTON2);
#ifdef FLUTTER_R2

	if (val == HIGH)
#else
	if (val == LOW)
#endif
	{
		//_button2=255;
	}
	else
	{
		//_button2=0;
              systemReset();
	}

// buttonsChanged=true;
}

void systemReset()
{
	flutter.setLED(0, 0, 255);
	delayMicroseconds(16000);
	delayMicroseconds(16000);
	flutter.setLED(0, 0, 0);
	delayMicroseconds(16000);
	delayMicroseconds(16000);
	flutter.setLED(0, 255, 0);
	delayMicroseconds(16000);
	delayMicroseconds(16000);
	flutter.setLED(0, 0, 0);
	delayMicroseconds(16000);
	delayMicroseconds(16000);
	flutter.setLED(255, 0, 0);
	delayMicroseconds(16000);
	delayMicroseconds(16000);
	flutter.setLED(0, 0, 0);
	initiateReset(1);
	tickReset();
}



void radioInterrupt()
{
	flutter.interrupt();
}
void softInt()
{
	flutter.processSoftInt();
}

extern boolean tickInterrupt()
{
	if (!flutter.initialized)
	{
		return true;
	}

	return flutter.tickInt();
}



#endif
