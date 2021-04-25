// setup for the gryo sensor ///////////////////////////////
#include <Adafruit_MLX90614.h>
#include <Wire.h>      // allows communication to i2c devices connected to arduino
#include<MPU6050.h>
#include <LiquidCrystal_I2C.h>

// setup for the h-bridge pins ///////////////////////////////////
int ENApin_a = 10; //set enable pin A_A
int IN1pin_a = 9; //set to In1 A
int IN2pin_a = 24; //set to In2 A

int ENBpin_a = 5; //set enable pin B_A
int IN3pin_a = 22;//set to In3 A
int IN4pin_a = 4; //set to In4 A

int ENApin_b = 6; //set enable pin A_B
int IN1pin_b = 13; //set to In1 B
int IN2pin_b = 12; //set to In2 B

int ENBpin_b = 11; //set enable pin B_B
int IN3pin_b = 8; //set to In3 B
int IN4pin_b = 7; //set to In4 B


//set up for motor speed //

int max_speed=150;
int min_speed=100;

// setup for the encoder pins ////////////////////////////////
// leg A_A
volatile long motorPositionAA = 0;
volatile int encoderStatusAA = 0;
const int ENC_AA_W        = 18;  // no change
const int ENC_AA_Y       = 19;  // no change

// leg A_B
volatile long motorPositionAB = 0;
volatile int encoderStatusAB = 0;
const int ENC_AB_W        = 2;  // no change
const int ENC_AB_Y       = 3;  // no change


// gyro pins /////////////////////////////////////////////////////

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
int16_t ax, ay, az;  // x y z orientation values from accelerometer
int16_t gx, gy, gz;  // x y z orientation values from gyrscope
MPU6050 mpu;
int initial_ax = 0;

// buzzer pins /////////////////////////////////////////////////////
const int buzzer = 50;

// echolocation pins //////////////////////////////////////////////////

const int trigPin1 = 46;
const int echoPin1 = 47;
long duration1;
int distance1;
const int trigPin2 = 48;
const int echoPin2 = 49;
long duration2;
int distance2;

volatile int input1 = HIGH;
volatile int input2 = LOW;

// motor encoder position functions /////////////////////////////////////////////////////
void updateMotorPositionAA() {
  encoderStatusAA <<= 1;
  encoderStatusAA |= digitalRead(18);
  encoderStatusAA <<= 1;
  encoderStatusAA |= digitalRead(19);
  encoderStatusAA &= 15;
  if (encoderStatusAA == 2 || encoderStatusAA == 4 || encoderStatusAA == 11 || encoderStatusAA == 13 ) {
    motorPositionAA++;
  } else {
    motorPositionAA--;
  }
}

void updateMotorPositionAB() {
  encoderStatusAB <<= 1;
  encoderStatusAB |= digitalRead(2);
  encoderStatusAB <<= 1;
  encoderStatusAB |= digitalRead(3);
  encoderStatusAB &= 15;
  if (encoderStatusAB == 2 || encoderStatusAB == 4 || encoderStatusAB == 11 || encoderStatusAB == 13 ) {
    motorPositionAB++;
  } else {
    motorPositionAB--;
  }
}

void FORWARD_SWITCH() {
  //Serial.println("moving forward");
  input1 = HIGH;
  input2 = LOW;
}


void BACKWARD_SWITCH() {
  // Serial.println("moving backward");
  input1 = LOW;
  input2 = HIGH;
}

// Setup /////////////////////////////////////////////////////
void setup () {
  //put setup code here to run once
  pinMode(ENApin_a, OUTPUT); //set all pins going to Hbridge to output
  pinMode(IN1pin_a, OUTPUT);
  pinMode(IN2pin_a, OUTPUT);
  pinMode(ENBpin_a, OUTPUT); //set all pins going to Hbridge to output
  pinMode(IN3pin_a, OUTPUT);
  pinMode(IN4pin_a, OUTPUT);
  pinMode(ENApin_b, OUTPUT); //set all pins going to Hbridge to output
  pinMode(IN1pin_b, OUTPUT);
  pinMode(IN2pin_b, OUTPUT);
  pinMode(ENBpin_b, OUTPUT); //set all pins going to Hbridge to output
  pinMode(IN3pin_b, OUTPUT);
  pinMode(IN4pin_b, OUTPUT);
  pinMode(ENC_AA_W, INPUT);
  pinMode(ENC_AA_Y, INPUT);
  pinMode(ENC_AB_W, INPUT);
  pinMode(ENC_AB_Y, INPUT);

  // Turn on the encoder channels
  digitalWrite(ENC_AA_W, HIGH);
  digitalWrite(ENC_AA_Y, HIGH);
  digitalWrite(ENC_AB_W, HIGH);
  digitalWrite(ENC_AB_Y, HIGH);

  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updateMotorPosition' is called:
  attachInterrupt(digitalPinToInterrupt(18), updateMotorPositionAA, CHANGE);  // Interrupt digital pin 18
  attachInterrupt(digitalPinToInterrupt(19), updateMotorPositionAA, CHANGE);  // Interrupt  digital pin 19
  attachInterrupt(digitalPinToInterrupt(2), updateMotorPositionAB, CHANGE);  // Interrupt digital pin 2
  attachInterrupt(digitalPinToInterrupt(3), updateMotorPositionAB, CHANGE);  // Interrupt to digital pin 3


  // setup for temp sensor
  mlx.begin();

  // buzzer
  pinMode(buzzer, OUTPUT);


  // echolocation
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input

  // Begin serial communication for monitoring.
  Serial.begin(115200); //begin serial communication


  // gyroooo
  Wire.begin();
  mpu.initialize();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  initial_ax = ax;
  Serial.begin(115200);

}

// loop /////////////////////////////////////////////////////

void loop() {
  //main code here to run repeatedly

  // temp code
  //Serial.print("ambient");
  //Serial.println(mlx.readAmbientTempC());
  //Serial.print("Target");
  //Serial.println(mlx.readObjectTempC());
  //delay(50);

  // gyro code
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // buzzer code
  if ((mlx.readAmbientTempC() + 3) < mlx.readObjectTempC()) {
    tone(buzzer, 1000);
    // Serial.println("BUZZZZZ");
  }
  else {
    noTone(buzzer);
    // Serial.println("no buzzzz");
  }

  // echolocation code
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(2);
  digitalWrite(trigPin1, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration1 = pulseIn(echoPin1, HIGH);
  // Calculating the distance
  distance1 = duration1 * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance1: ");
  Serial.println(distance1);

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(2);
  digitalWrite(trigPin2, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration2 = pulseIn(echoPin2, HIGH);
  // Calculating the distance
  distance2 = duration2 * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance2: ");
  Serial.println(distance2);

  // direction changes code
  if (distance2 < 50) {
    BACKWARD_SWITCH();
  }
  if (distance1 < 50) {
    FORWARD_SWITCH();
  }

  // motor code with gyro corrections
  if (ax > initial_ax + 500) {
    analogWrite(ENApin_a, min_speed); // turns motor on
    digitalWrite(IN1pin_a, input2); //set IN1pin to low
    digitalWrite(IN2pin_a, input1);

    analogWrite(ENBpin_a, min_speed); // turns motor on
    digitalWrite(IN3pin_a, input1); //set IN1pin to low
    digitalWrite(IN4pin_a, input2);

    analogWrite(ENApin_b, max_speed); // turns motor on
    digitalWrite(IN1pin_b, input2); //set IN1pin to low
    digitalWrite(IN2pin_b, input1);

    analogWrite(ENBpin_b, max_speed); // turns motor on
    digitalWrite(IN3pin_b, input1); //set IN1pin to low
    digitalWrite(IN4pin_b, input2);
    Serial.println("Off axis---LEFT");
  }
  if (ax < initial_ax - 500) {
    analogWrite(ENApin_a, max_speed); // turns motor on
    digitalWrite(IN1pin_a, input2); //set IN1pin to low
    digitalWrite(IN2pin_a, input1);

    analogWrite(ENBpin_a, max_speed); // turns motor on
    digitalWrite(IN3pin_a, input1); //set IN1pin to low
    digitalWrite(IN4pin_a, input2);

    analogWrite(ENApin_b, min_speed); // turns motor on
    digitalWrite(IN1pin_b, input2); //set IN1pin to low
    digitalWrite(IN2pin_b, input1);

    analogWrite(ENBpin_b, min_speed); // turns motor on
    digitalWrite(IN3pin_b, input1); //set IN1pin to low
    digitalWrite(IN4pin_b, input2);
    Serial.println("off axis--RIGHT");

    if (initial_ax - 500 <= ax <= initial_ax + 500) {
      analogWrite(ENApin_a, max_speed); // turns motor on
      digitalWrite(IN1pin_a, input2); //set IN1pin to low
      digitalWrite(IN2pin_a, input1);

      analogWrite(ENBpin_a, max_speed); // turns motor on
      digitalWrite(IN3pin_a, input1); //set IN1pin to low
      digitalWrite(IN4pin_a, input2);

      analogWrite(ENApin_b, max_speed); // turns motor on
      digitalWrite(IN1pin_b, input2); //set IN1pin to low
      digitalWrite(IN2pin_b, input1);

      analogWrite(ENBpin_b, max_speed); // turns motor on
      digitalWrite(IN3pin_b, input1); //set IN1pin to low
      digitalWrite(IN4pin_b, input2);
    }
  }

  delay(50);


  // encoder check
  //Serial.print("Motor AA");
  //Serial.println(motorPositionAA);
  //Serial.print("Motor AB");
  //Serial.println(motorPositionAB);



}




// End of function updateMotorPosition()
