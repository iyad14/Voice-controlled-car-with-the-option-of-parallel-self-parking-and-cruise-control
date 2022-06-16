#include <Stepper.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <LiquidCrystal.h>

// ************************
// bluetooth voice control

int TxD = 49;
int RxD = 51;
String value;
SoftwareSerial bluetooth(TxD, RxD);

// ****************

LiquidCrystal lcd(41, 39, 37, 35, 33, 31);

const int stepsPerRevolution = 4096;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

#define STEPPER_PIN_1 8
#define STEPPER_PIN_2 9
#define STEPPER_PIN_3 10
#define STEPPER_PIN_4 11
int step_number = 0;


//Ultrasonic sensor 
#define ECHOPIN 12
#define TRIGPIN 13

////connect CLK to 2, DT to 3,  SW to 2 digital, GND to GND, and + to 5V
//#define encoder0PinA  2
//#define encoder0PinB  4

#define enA 5
#define in1 6
#define in2 7

#define enB 4
#define in3 3
#define in4 2



volatile long encoder0Pos=0;
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
long vel;

Adafruit_MPU6050 mpu;

#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38
typedef FIS_TYPE(*_FIS_MF)(FIS_TYPE, FIS_TYPE*);
typedef FIS_TYPE(*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE(*_FIS_ARR)(FIS_TYPE*, int, _FIS_ARR_OP);

// Number of inputs to the fuzzy inference system
const int fis_gcI = 2;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 1;
// Number of rules to the fuzzy inference system
const int fis_gcR = 15;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

// Motor speed control
int rotDirection = 0;
int pressed = false;

double acceleration_bias = 0;

sensors_event_t a, g, temp;

double Required_distance = 20;



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
    fis_trapmf, fis_trimf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 5, 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 5 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { -Required_distance, -Required_distance, -0.6*Required_distance, -0.3*Required_distance};
FIS_TYPE fis_gMFI0Coeff2[] = { -0.3*Required_distance, 0, 0.3*Required_distance} ;
FIS_TYPE fis_gMFI0Coeff3[] = { 0.3*Required_distance, 0.6*Required_distance, 500, 500};
FIS_TYPE fis_gMFI0Coeff4[] = { -0.6*Required_distance, -0.3*Required_distance, 0 } ;
FIS_TYPE fis_gMFI0Coeff5[] = { 0, 0.3*Required_distance, 0.6*Required_distance} ;


FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3, fis_gMFI0Coeff4, fis_gMFI0Coeff5 };
FIS_TYPE fis_gMFI1Coeff1[] = { -20, -20, -13.33, 0 };
FIS_TYPE fis_gMFI1Coeff2[] = { -13.33, 0, 13.33 };
FIS_TYPE fis_gMFI1Coeff3[] = { 0, 13.33, 20, 20 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { -20, -20, -13.33, -6.668 };
FIS_TYPE fis_gMFO0Coeff2[] = { -6.668, 0, 6.668 };
FIS_TYPE fis_gMFO0Coeff3[] = { 6.668, 13.33, 20, 20 };
FIS_TYPE fis_gMFO0Coeff4[] = { -13.33, -6.668, 0 };
FIS_TYPE fis_gMFO0Coeff5[] = { 0, 6.668, 13.33 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4, fis_gMFO0Coeff5 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 1, 0, 1, 1 };
int fis_gMFI1[] = { 0, 1, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1};

// Output membership function set
int fis_gMFO0[] = { 0, 1, 0, 1, 1 };
int* fis_gMFO[] = { fis_gMFO0};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1, 1 };
int fis_gRI1[] = { 1, 2 };
int fis_gRI2[] = { 1, 3 };
int fis_gRI3[] = { 2, 1 };
int fis_gRI4[] = { 2, 2 };
int fis_gRI5[] = { 2, 3 };
int fis_gRI6[] = { 4, 3 };
int fis_gRI7[] = { 4, 2 };
int fis_gRI8[] = { 4, 1 };
int fis_gRI9[] = { 5, 1 };
int fis_gRI10[] = { 5, 2 };
int fis_gRI11[] = { 3, 1 };
int fis_gRI12[] = { 3, 2 };
int fis_gRI13[] = { 3, 3 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11, fis_gRI12, fis_gRI13 };

// Rule Outputs
int fis_gRO0[] = { 1 };
int fis_gRO1[] = { 1 };
int fis_gRO2[] = { 1 };
int fis_gRO3[] = { 5 };
int fis_gRO4[] = { 2 };
int fis_gRO5[] = { 4 };
int fis_gRO6[] = { 4 };
int fis_gRO7[] = { 2 };
int fis_gRO8[] = { 5 };
int fis_gRO9[] = { 5 };
int fis_gRO10[] = { 5 };
int fis_gRO11[] = { 3 };
int fis_gRO12[] = { 3 };
int fis_gRO13[] = { 5 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11, fis_gRO12, fis_gRO13 };

// Input range Min
FIS_TYPE fis_gIMin[] = { -Required_distance, -20 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 500, 20 };

// Output range Min
FIS_TYPE fis_gOMin[] = { -20 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 20 };

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
double fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, };
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

    return g_fisOutput[0];
}



void Control_Distance() { 
 // Read Input: Distance
    g_fisInput[0] = ultra_measure(TRIGPIN,ECHOPIN) - Required_distance;
    // Read Input: input_acceleration
    g_fisInput[1] = measure_acceleration() - acceleration_bias;

    g_fisOutput[0] = 0;
    Serial.print(g_fisInput[0]);
    Serial.print(" ");
    Serial.print( g_fisInput[1]);
    Serial.print(" ");
    Serial.println(fis_evaluate());
    
    
    if(ultra_measure(TRIGPIN,ECHOPIN) > 0.2*Required_distance){
      motor_control(true, fis_evaluate());
    }else {
      digitalWrite(enA, LOW);
    }
}

void motor_control(bool dir, double acceleration) {
  int potValue = acceleration; // Read potentiometer value
  int pwmOutput = map(potValue, -20, 20, 0 , 255); // Map the potentiometer value from 0 to 255
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin

  Serial.println(pwmOutput);
  // go forward
  if (dir == true  & rotDirection == 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    rotDirection = 1;
    delay(20);
  }
  // go backward
  if (dir == false & rotDirection == 1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    rotDirection = 0;
    delay(20);
  }
}

double measure_acceleration() {
  mpu.getEvent(&a, &g, &temp);
  delay(200);
  return a.acceleration.y;
}

//void measure_speed() {
//  pinMode(encoder0PinA, INPUT);
//  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
//  pinMode(encoder0PinB, INPUT);
//  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
//  attachInterrupt(0, doEncoder, RISING);  // encoDER ON PIN 2
//  newposition = encoder0Pos;
//  newtime = millis();
//  vel = (newposition-oldposition) * 1000 /(newtime-oldtime);
//  Serial.print ("speed = ");
//  Serial.print (vel);
//  oldposition = newposition;
//  oldtime = newtime;
//  delay(250);
//}

//void doEncoder(){
//  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
//    encoder0Pos++;
//  } else {
//    encoder0Pos--;
//  }
//}

double ultra_measure(int trig, int echo) {
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  
  float distance = pulseIn(echo, HIGH);
  distance = distance/58;
  // in cm
  delay(200);
  return distance;
}


void StepperCW(int steps) {
  for(int i = 0; i < steps; i++) {
  switch(step_number){
        case 0:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
        case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
        case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
        case 3:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
      } 
      step_number++;
      if(step_number > 3){
        step_number = 0;
      }
      delay(2);
  }
}


void StepperCCW(int steps) {
  for(int i = 0; i < steps; i++) {
    switch(step_number){
          case 0:
          digitalWrite(STEPPER_PIN_1, LOW);
          digitalWrite(STEPPER_PIN_2, LOW);
          digitalWrite(STEPPER_PIN_3, LOW);
          digitalWrite(STEPPER_PIN_4, HIGH);
          break;
          case 1:
          digitalWrite(STEPPER_PIN_1, LOW);
          digitalWrite(STEPPER_PIN_2, LOW);
          digitalWrite(STEPPER_PIN_3, HIGH);
          digitalWrite(STEPPER_PIN_4, LOW);
          break;
          case 2:
          digitalWrite(STEPPER_PIN_1, LOW);
          digitalWrite(STEPPER_PIN_2, HIGH);
          digitalWrite(STEPPER_PIN_3, LOW);
          digitalWrite(STEPPER_PIN_4, LOW);
          break;
          case 3:
          digitalWrite(STEPPER_PIN_1, HIGH);
          digitalWrite(STEPPER_PIN_2, LOW);
          digitalWrite(STEPPER_PIN_3, LOW);
          digitalWrite(STEPPER_PIN_4, LOW);   
      } 
        step_number++;
        if(step_number > 3){
          step_number = 0;
        }
        delay(2);
    }
}





void steer(bool rot, double time, bool dir) {
  int pwmOutput = 200; // Map the potentiometer value from 0 to 255
  analogWrite(enB, pwmOutput); // Send PWM signal to L298N Enable pin

  //true for right / false for left
  analogWrite(enB, HIGH); 
  if(rot) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    motor_control(dir, 5);
    delay(time);
  }else {
    digitalWrite(in4, HIGH);
    digitalWrite(in3, LOW);
    motor_control(dir, 5);
    delay(time);
  }
    digitalWrite(in4, LOW);
    digitalWrite(in3, LOW);
}

void display(double distance, double acceleration) {
  String Distance = "Dist: ";
  Distance.concat(distance);
  Distance.concat("cm");
  lcd.print(Distance);
  lcd.setCursor(0,1);
  String Acc = "Accel: ";
  Acc.concat(acceleration);
  Acc.concat("m/s2");
  lcd.print(Acc);
  delay(200);
  lcd.clear();
}

//void voice_control() {
//  while (Serial.available())   //Check if there is an available byte to read
//  {                            
//  delay(10);                   //Delay added to make thing stable
//  char c = Serial.read();      //Conduct a serial read
//  if (c == '#') {break;}       //Exit the loop when the # is detected after the word
//  voice += c;                  //Shorthand for voice = voice + c
//  } 
//
//  if (voice.length() > 0) {
//    Serial.println(voice);
//  //----------Control Multiple Pins/ LEDs----------// 
//
//       if(voice == "forward")//                                FOR RED COLOUR OF THE LED 
//     {
//     motor_control(true,12)
//     }  
//  else if(voice == "back")//                              FOR GREEN COLOUR OF THE LED !
//     {
//    motor_control(false,12);
//     }
//  else if(voice == "Adjust distance")//                                FOR BLUE COLOUR OF THE LED !
//     {
//    
//     }
//  voice="";                                                       //Reset the variable after initiating
//  }
//}

void setup(){

  
  
  lcd.begin(16, 2);
  Serial.begin(9600);
  bluetooth.begin(9600);
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);


//accelerometer
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
 

   // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  //digitalWrite(in3, HIGH);
  //digitalWrite(in4, HIGH);

  acceleration_bias = measure_acceleration();
}

void loop()
{

  Serial.println(value);
  if(bluetooth.available()) {
     value = bluetooth.readString();
     if(value.substring(0,16) == "Distance Control"){
      
      Required_distance = value.substring(20,value.length());
     }
     value = bluetooth.readString();
      if (value == "drive") {
        double distance = ultra_measure(TRIGPIN,ECHOPIN);
        double acceleration = measure_acceleration()-acceleration_bias;
        display(distance, acceleration);
        
        Control_Distance();
      }

    if (value == "park"){
      bool check = check_park();
      park(check);
    }

    if (value == "forward"){
      motor_control(true, 5); 
    }

     if (value == "backward"){
      motor_control(false, 5); 
    }
     if(value == "stop"){
      motor_control(true, -20);
     }
  }
}

// self parking car code 

// setup the new pins
// for parking we have 3 ultrasonic sensors:

// define the pins used for ultrasonic sensors used for self parking
#define EchoBack 22;
#define TrigBack 24;


// method to check if space is enough to park
bool check_park()
{
  bool check = false;

  // let the car move forward initially with a relatively slow speed
  // steer the ultrasonic to the side
  StepperCW(50);
  StepperCCW_B(50);
  motor_control(true,0);

  double distF = ultra_measure(TRIGPIN, ECHOPIN);
  double distB = ultra_measure(TrigBack, EchoBack);
  while (true){
     distF = ultra_measure(TRIGPIN, ECHOPIN);
     distB = ultra_measure(TrigBack, EchoBack);
     if(distF>9 && distB>9) {
      motor_control(true, -20);
      break;
     }
  }

  StepperCCW(44);

  int count = 0;
  int min_count;
  double current, next, d;
  next = ultra_measure(TRIGPIN, ECHOPIN);
  d = next;
  
  while(count<19){
    next = ultra_measure(TRIGPIN, ECHOPIN);
    if(next < d) {
      d = next;
      min_count = count;
    }
    count = count + 1;
  }

  double angle = (min_count + 6) * 1.8;
  double l = 32; //car length

  if(d*cos(angle) > 0.5*l) {
    check = true;
  }

  StepperCCW(25); //retrun stepper to initial position
  return check;
  
}

// display message that the parking space is not enough
void display_error() {
  lcd.clear();
  String error = "NO PARK";
  lcd.print(error);
  delay(1000);
  lcd.clear();
}


// self parking method
void park(bool check){
  if (check){
    
  double distB = ultra_measure(TrigBack, EchoBack);
  motor_control(true, 0);
  while(distB > 9){
    distB = ultra_measure(TrigBack, EchoBack);
    if(distB < 9) { //stop when back of the car coincides with the back of the front obstacle
      motor_control(true, -20);
      motor_control(false, 0); //get to a suitable position to start parking
      delay(100);
      motor_control(false, -20);
    }
  }
  
 
  //steer the back ultra sonic to measure the rear back-right
  StepperCCW_B(25);
  // let the car move backwards, to the right to park
 
  rear_back = ultra_measure(TrigBack, EchoBack);
   while(rear_back>4){
       steer(true, 100, false);
       rear_back = ultra_measure(TrigBack, EchoBack);
   }
      delay(500);
      motor_control(true,-20);

  // steer wheel back when close to the wall
  // steer the rear ultrasonic back

  StepperCW_B(25);
  rear_back = ultra_measure(TrigBack,EchoBack);
  while(rear_back>4){
     steer(false, 100, false);
     rear_back = ultra_measure(TrigBack, EchoBack); 
  }
    delay(500);
    motor_control(true,-20);
    
  }
  else {
    display_error();
  }
}


// back stepper method and defining the pins


#define BSTEPPER_PIN_1 8
#define BSTEPPER_PIN_2 9
#define BSTEPPER_PIN_3 10
#define BSTEPPER_PIN_4 11
int step_number = 0;

void StepperCCW_B(int steps) {
  for(int i = 0; i < steps; i++) {
    switch(step_number){
          case 0:
          digitalWrite(BSTEPPER_PIN_1, LOW);
          digitalWrite(BSTEPPER_PIN_2, LOW);
          digitalWrite(BSTEPPER_PIN_3, LOW);
          digitalWrite(BSTEPPER_PIN_4, HIGH);
          break;
          case 1:
          digitalWrite(BSTEPPER_PIN_1, LOW);
          digitalWrite(BSTEPPER_PIN_2, LOW);
          digitalWrite(BSTEPPER_PIN_3, HIGH);
          digitalWrite(BSTEPPER_PIN_4, LOW);
          break;
          case 2:
          digitalWrite(BSTEPPER_PIN_1, LOW);
          digitalWrite(BSTEPPER_PIN_2, HIGH);
          digitalWrite(BSTEPPER_PIN_3, LOW);
          digitalWrite(BSTEPPER_PIN_4, LOW);
          break;
          case 3:
          digitalWrite(BSTEPPER_PIN_1, HIGH);
          digitalWrite(BSTEPPER_PIN_2, LOW);
          digitalWrite(BSTEPPER_PIN_3, LOW);
          digitalWrite(BSTEPPER_PIN_4, LOW);   
      } 
        step_number++;
        if(step_number > 3){
          step_number = 0;
        }
        delay(2);
    }
}

void StepperCW_B(int steps) {
  for(int i = 0; i < steps; i++) {
  switch(step_number){
        case 0:
        digitalWrite(BSTEPPER_PIN_1, HIGH);
        digitalWrite(BSTEPPER_PIN_2, LOW);
        digitalWrite(BSTEPPER_PIN_3, LOW);
        digitalWrite(BSTEPPER_PIN_4, LOW);
        break;
        case 1:
        digitalWrite(BSTEPPER_PIN_1, LOW);
        digitalWrite(BSTEPPER_PIN_2, HIGH);
        digitalWrite(BSTEPPER_PIN_3, LOW);
        digitalWrite(BSTEPPER_PIN_4, LOW);
        break;
        case 2:
        digitalWrite(BSTEPPER_PIN_1, LOW);
        digitalWrite(BSTEPPER_PIN_2, LOW);
        digitalWrite(BSTEPPER_PIN_3, HIGH);
        digitalWrite(BSTEPPER_PIN_4, LOW);
        break;
        case 3:
        digitalWrite(BSTEPPER_PIN_1, LOW);
        digitalWrite(BSTEPPER_PIN_2, LOW);
        digitalWrite(BSTEPPER_PIN_3, LOW);
        digitalWrite(BSTEPPER_PIN_4, HIGH);
        break;
      } 
      step_number++;
      if(step_number > 3){
        step_number = 0;
      }
      delay(2);
  }
}
