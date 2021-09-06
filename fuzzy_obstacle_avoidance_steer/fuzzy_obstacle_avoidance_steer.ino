/************************************************************
    This Arduino code would be uploaded to the Arduino Mega
    Fuzzy Logic Obstacle Avoidance for Steerable Control
*************************************************************/

#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38
typedef FIS_TYPE(*_FIS_MF)(FIS_TYPE, FIS_TYPE*);
typedef FIS_TYPE(*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE(*_FIS_ARR)(FIS_TYPE*, int, _FIS_ARR_OP);

#include <AFMotor.h>
#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"
#include "NewPing.h"
#include "TimerOne.h"
#include <Servo.h>
#include <math.h>

#define tanf tan

// Number of inputs to the fuzzy inference system
const int fis_gcI = 3;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 2;
// Number of rules to the fuzzy inference system
const int fis_gcR = 27;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

/************************* 
  ROBOT DATA STRUCTURE 
**************************/
struct Data {
    int direction = 0; // -1: left, 0: straight, 1: right
    float left_speed = 0;
    float right_speed = 0;
    float left_distance = 0;
    float middle_distance = 0;
    float right_distance = 0;
};
typedef struct Data Robot_Data;
Robot_Data robot_data;

/**************************
  ULTRASONIC SENSOR SETUP 
***************************/
#define MAX_DISTANCE 100
#define Echo1 31 // LEFT_SENSOR ECHO
#define Trig1 33 // LEFT_SENSOR TRIG
#define Echo2 39 // MID_SENSOR ECHO
#define Trig2 41 // MID_SENSOR TRIG
#define Echo3 47 // RIGHT_SENSOR ECHO
#define Trig3 49 // RIGHT_SENSOR TRIG

NewPing Left_Sensor(Trig1, Echo1, MAX_DISTANCE);
NewPing Middle_Sensor(Trig2, Echo2, MAX_DISTANCE);
NewPing Right_Sensor(Trig3, Echo3, MAX_DISTANCE);
float left_distance, middle_distance, right_distance;

/*****************
  NRF24L01 SETUP 
******************/
RF24 radio(22, 24); // CE, CSN
const byte address[][6] = {"T", "G"};
void init_radio_com(){
    Serial.println("Initializing Radio Com...");
    radio.begin();
    radio.setChannel(100);
    radio.openWritingPipe(address[0]);
    radio.setPALevel(RF24_PA_MIN);
}

/**************
  MOTOR SETUP 
***************/
// Pin definition
AF_DCMotor right_motor(3);
AF_DCMotor left_motor(4);
Servo servo_motor;
int current_angle, new_angle;

/**************** 
  ENCODER SETUP 
*****************/
const byte left_encoder_pin = 20;
const byte right_encoder_pin = 21;
// number of pulses
volatile int left_pulses = 0; 
volatile int right_pulses = 0;
unsigned long timeold;
const byte disk_slots = 20;
const int wheel_radius = 3;
const int length = 12;
const int pi = 3.14;

void left_counter() {
   //Update count
   left_pulses++;
}

void right_counter() {
   //Update count
   right_pulses++;
}

void ISR_timerone()
{
    Timer1.detachInterrupt();  // Stop the timer
    float rotation1 = (left_pulses / disk_slots) * 60.00;  // calculate RPM for Motor 1
    float speed1 = rotation1 * 2 * pi * wheel_radius / 60; // calculate speed for Motor 1: left motor
    robot_data.left_speed = speed1;
    left_pulses = 0;                                        //  reset counter to zero
    float rotation2 = (right_pulses / disk_slots) * 60.00;  // calculate RPM for Motor 2
    float speed2 = rotation2 * 2 * pi * wheel_radius / 60; // calculate speed for Motor 2: right motor
    robot_data.right_speed = speed2;
    right_pulses = 0; //  reset counter to zero
    float angle = current_angle * (pi/180); // convert degrees to radian
    int theta = (speed1 / length) * (tan(angle) * (180 / pi));
    if (theta < 0) {
      // direction is left
        robot_data.direction = -1;
    } else if (theta > 0){
      // direction is right
        robot_data.direction = 1;
    } else {
      // direction is straight
        robot_data.direction = 0;
  }

    Timer1.attachInterrupt(ISR_timerone); // Enable the timer
}

void init_encoder(){
    Serial.println("Initializing Encoder...");
  // Attach an interrupt to each encoder pins
    Timer1.initialize(1000000); // set timer for 1sec
    attachInterrupt(digitalPinToInterrupt(left_encoder_pin), left_counter, RISING);
    attachInterrupt(digitalPinToInterrupt(right_encoder_pin), right_counter, RISING);
    Timer1.attachInterrupt( ISR_timerone ); // Enable the timer
}

void init_motors(){
    Serial.println("Initializing DC motors...");
    left_motor.setSpeed(200);
    right_motor.setSpeed(200);

    left_motor.run(BACKWARD);
    right_motor.run(BACKWARD);

    delay(3000);

    left_motor.run(RELEASE);
    right_motor.run(RELEASE);

}

void init_servo(){
    Serial.println("Initializing Servo...");
    servo_motor.attach(10);
    current_angle = 0;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Initiating Sequence...");  
    init_encoder();
    init_radio_com();
    delay(1000);
    init_motors();
    init_servo();
}

// Loop routine runs over and over again forever:
void loop()
{
    // Read Input: leftDistance
    left_distance = Left_Sensor.ping_cm();
    g_fisInput[0] = (int) left_distance;
    // Read Input: middleDistance
    middle_distance = Middle_Sensor.ping_cm();
    g_fisInput[1] = middle_distance;
    // Read Input: rightDistance
    right_distance = Right_Sensor.ping_cm();
    g_fisInput[2] = right_distance;

    g_fisOutput[0] = 0;
    g_fisOutput[1] = 0;

    fis_evaluate();

    // Set output value: steerAngle
    new_angle = (int) g_fisOutput[0];

    if (new_angle > current_angle){
        for (int pos = current_angle; pos <= new_angle; pos +=1){
            servo_motor.write(pos);
            delay(15);
        }

    } else if (new_angle < current_angle){
        for (int pos = current_angle; pos >= new_angle; pos -=1){
            servo_motor.write(pos);
            delay(15);
        }

    }
    current_angle = new_angle;
    // servo_motor.write(new_angle);
    // TODO: IMPLEMENT INCREMENTAL SPEED (Probably)
    // Set output value: motorSpeed
    left_motor.setSpeed((uint8_t)g_fisOutput[1]);
    right_motor.setSpeed((uint8_t)g_fisOutput[1]);
    left_motor.run(FORWARD);
    right_motor.run(FORWARD);

}

//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Trapezoidal Member Function
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
    FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
    FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
    return (FIS_TYPE) min(t1, t2);
}

// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0);
}

// Sigmoid Member Function
FIS_TYPE fis_sigmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], c = p[1];
    return (FIS_TYPE) (1.0 / (1.0 + exp(-a *(x - c))));
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trapmf, fis_trimf, fis_sigmf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 3, 3, 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 2, 3 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { 0, 0, 25, 50 };
FIS_TYPE fis_gMFI0Coeff2[] = { 25, 50, 75 };
FIS_TYPE fis_gMFI0Coeff3[] = { 50, 75, 100, 100 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3 };
FIS_TYPE fis_gMFI1Coeff1[] = { 0, 0, 25, 50 };
FIS_TYPE fis_gMFI1Coeff2[] = { 25, 50, 75 };
FIS_TYPE fis_gMFI1Coeff3[] = { 50, 75, 100, 100 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3 };
FIS_TYPE fis_gMFI2Coeff1[] = { 0, 0, 25, 50 };
FIS_TYPE fis_gMFI2Coeff2[] = { 25, 50, 75 };
FIS_TYPE fis_gMFI2Coeff3[] = { 50, 75, 100, 100 };
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2, fis_gMFI2Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { -0.192, -27.73 };
FIS_TYPE fis_gMFO0Coeff2[] = { 0.192, 27.7333333333333 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2 };
FIS_TYPE fis_gMFO1Coeff1[] = { 0, 0, 50, 125 };
FIS_TYPE fis_gMFO1Coeff2[] = { 50, 125, 200 };
FIS_TYPE fis_gMFO1Coeff3[] = { 125, 200, 255, 255 };
FIS_TYPE* fis_gMFO1Coeff[] = { fis_gMFO1Coeff1, fis_gMFO1Coeff2, fis_gMFO1Coeff3 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff, fis_gMFO1Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 1, 0 };
int fis_gMFI1[] = { 0, 1, 0 };
int fis_gMFI2[] = { 0, 1, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2};

// Output membership function set
int fis_gMFO0[] = { 2, 2 };
int fis_gMFO1[] = { 0, 1, 0 };
int* fis_gMFO[] = { fis_gMFO0, fis_gMFO1};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1, 1, 1 };
int fis_gRI1[] = { 1, 1, 2 };
int fis_gRI2[] = { 1, 1, 3 };
int fis_gRI3[] = { 1, 2, 1 };
int fis_gRI4[] = { 1, 2, 2 };
int fis_gRI5[] = { 1, 2, 3 };
int fis_gRI6[] = { 1, 3, 1 };
int fis_gRI7[] = { 1, 3, 2 };
int fis_gRI8[] = { 1, 3, 3 };
int fis_gRI9[] = { 2, 1, 1 };
int fis_gRI10[] = { 2, 1, 2 };
int fis_gRI11[] = { 2, 1, 3 };
int fis_gRI12[] = { 2, 2, 1 };
int fis_gRI13[] = { 2, 2, 2 };
int fis_gRI14[] = { 2, 2, 3 };
int fis_gRI15[] = { 2, 3, 1 };
int fis_gRI16[] = { 2, 3, 2 };
int fis_gRI17[] = { 2, 3, 3 };
int fis_gRI18[] = { 3, 1, 1 };
int fis_gRI19[] = { 3, 1, 2 };
int fis_gRI20[] = { 3, 1, 3 };
int fis_gRI21[] = { 3, 2, 1 };
int fis_gRI22[] = { 3, 2, 2 };
int fis_gRI23[] = { 3, 2, 3 };
int fis_gRI24[] = { 3, 3, 1 };
int fis_gRI25[] = { 3, 3, 2 };
int fis_gRI26[] = { 3, 3, 3 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11, fis_gRI12, fis_gRI13, fis_gRI14, fis_gRI15, fis_gRI16, fis_gRI17, fis_gRI18, fis_gRI19, fis_gRI20, fis_gRI21, fis_gRI22, fis_gRI23, fis_gRI24, fis_gRI25, fis_gRI26 };

// Rule Outputs
int fis_gRO0[] = { 0, 1 };
int fis_gRO1[] = { 1, 2 };
int fis_gRO2[] = { 1, 3 };
int fis_gRO3[] = { 0, 1 };
int fis_gRO4[] = { 1, 2 };
int fis_gRO5[] = { 1, 3 };
int fis_gRO6[] = { 0, 1 };
int fis_gRO7[] = { 1, 2 };
int fis_gRO8[] = { 1, 3 };
int fis_gRO9[] = { 2, 2 };
int fis_gRO10[] = { 2, 1 };
int fis_gRO11[] = { 1, 3 };
int fis_gRO12[] = { 2, 2 };
int fis_gRO13[] = { 0, 2 };
int fis_gRO14[] = { 1, 2 };
int fis_gRO15[] = { 2, 2 };
int fis_gRO16[] = { 0, 2 };
int fis_gRO17[] = { 1, 3 };
int fis_gRO18[] = { 2, 3 };
int fis_gRO19[] = { 2, 3 };
int fis_gRO20[] = { 1, 3 };
int fis_gRO21[] = { 2, 3 };
int fis_gRO22[] = { 2, 3 };
int fis_gRO23[] = { 2, 3 };
int fis_gRO24[] = { 2, 3 };
int fis_gRO25[] = { 2, 3 };
int fis_gRO26[] = { 0, 3 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11, fis_gRO12, fis_gRO13, fis_gRO14, fis_gRO15, fis_gRO16, fis_gRO17, fis_gRO18, fis_gRO19, fis_gRO20, fis_gRO21, fis_gRO22, fis_gRO23, fis_gRO24, fis_gRO25, fis_gRO26 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 100, 100, 100 };

// Output range Min
FIS_TYPE fis_gOMin[] = { -45, 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 45, 255 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    // calculate the area under the curve formed by the MF outputs
    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0 };
    FIS_TYPE fuzzyOutput1[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, fuzzyOutput1, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
}
