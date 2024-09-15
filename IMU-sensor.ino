#include <Wire.h> //library allows communication with I2C / TWI devices
#include <math.h> //library includes mathematical functions

const int MPU_ADDRESS=0x68; //I2C address of the MPU-6050

const int TILT_LIMIT = 10000; // values for filtering IMU acceleration values
const int MOVE_LIMIT = 1000;

// LED pin numbers
const int BLUE_LED = 49;
const int GREEN_LED = 45;
const int YELLOW_LED = 41;
const int RED_LED = 37;

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //16-bit integers
double t,tx,tf,pitch,roll;
int acc_x, acc_y, acc_z, temp, gy_x, gy_y, gy_z;

// direction the IMU is facing
int orientation;
bool move_flag;

// calibration constants
const int ACX = -950;
const int ACY = -300;
const int ACZ = 0;
const int TCAL = -1600;
const int GX = 480;
const int GY = 170;
const int GZ = 210;

// for determining if the IMU has moved
bool left_flag, right_flag, down_flag, up_flag;
bool left_back, right_back, down_back, up_back;
int prev_orientation, curr_orientation;

int cntr;
bool haveMoved;

// way to move
String input_dir;
bool currMoving;



void setup()
{
    Wire.begin(); //initiate wire library and I2C
    Wire.beginTransmission(MPU_ADDRESS); //begin transmission to I2C slave device
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)  
    Wire.endTransmission(true); //ends transmission to I2C slave device
    Serial.begin(9600); //serial communication at 9600 bauds
    
    // movement
    curr_orientation = 0;
    haveMoved = false;
    currMoving = false;

    // LEDs
    pinMode(BLUE_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
}

void loop()
{
  if (currMoving == false && haveMoved == false) {
    // prompt user for direction to move (eventually read in from serial port)
    //Serial.print("\nwhich direction to move? [left, right, down, up]\n");
    while (Serial.available() == 0) {
    }

    input_dir = Serial.readString();
    Serial.print("moving "); Serial.print(input_dir);
    //delay(1000);

    currMoving = true;
  } /*else if (currMoving == true && haveMoved == true) {
    Serial.print("wrong direction! try moving "); Serial.println (input_dir);
  } */

  // light up appropriate LED
  if (input_dir == "left\n") {
    digitalWrite(BLUE_LED, HIGH);
  } else if (input_dir == "right\n") {
    digitalWrite(GREEN_LED, HIGH);
  } else if (input_dir == "down\n") {
    digitalWrite(YELLOW_LED, HIGH);
  } else if (input_dir == "up\n") {
    digitalWrite(RED_LED, HIGH);
  }
  

    Wire.beginTransmission(MPU_ADDRESS); //begin transmission to I2C slave device
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); //restarts transmission to I2C slave device
    Wire.requestFrom(MPU_ADDRESS,14,true); //request 14 registers in total  


    //read accelerometer data
    AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)  
    AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L) 
    AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)
  
    //read temperature data 
    Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L) 
  
    //read gyroscope data
    GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L) 

    //temperature calculation
    tx = Tmp + TCAL;
    t = tx/340 + 36.53; //equation for temperature in degrees C from datasheet
    tf = (t * 9/5) + 32; //fahrenheit

    //get pitch/roll
    getAngle(AcX,AcY,AcZ);

    // add calibrations
    acc_x = AcX + ACX;
    acc_y = AcY + ACY;
    acc_z = AcZ + ACZ;

    gy_x = GyX + GX;
    gy_y = GyY + GY;
    gy_z = GyZ + GZ;

    // get orientation and moving state
    orientation = getDirection(acc_x, acc_y, acc_z);
    move_flag = isMoving(gy_x, gy_y, gy_z);

    // update "states"
    prev_orientation = curr_orientation;
    curr_orientation = orientation;

    // sanity check
    // Serial.print("states: "); Serial.print(prev_orientation); Serial.print(" "); Serial.println(curr_orientation);

    // Serial.println(orientation);  // sanity check
    // Serial.println(move_flag);

    // set flags
    stateMachine(prev_orientation, curr_orientation, &left_flag, &right_flag, &down_flag, &up_flag, &left_back, &right_back, &down_back, &up_back, &cntr);

    if (left_flag && left_back) {
      Serial.println("tilted left!"); 
      if (input_dir == "left\n") {
        digitalWrite(BLUE_LED, LOW);
        currMoving = false;
      }
      //Serial.print("time taken: "); Serial.print(cntr); Serial.print("\n");
      haveMoved = true;
    } else if (right_flag && right_back) {
      Serial.println("tilted right!");
      if (input_dir == "right\n") {
        digitalWrite(GREEN_LED, LOW);
        currMoving = false;
      }
      //Serial.print("time taken: "); Serial.print(cntr); Serial.print("\n");
      haveMoved = true;
    }
    else if (down_flag && down_back) {
      Serial.println("tilted down!");
      if (input_dir == "down\n") {
        digitalWrite(YELLOW_LED, LOW);
        currMoving = false;
      }
      //Serial.print("time taken: "); Serial.print(cntr); Serial.print("\n");
      haveMoved = true;
    }
    else if (up_flag && up_back) {
      Serial.println("tilted up!");
      if (input_dir == "up\n") {
        digitalWrite(RED_LED, LOW);
        currMoving = false;
      }
      //Serial.print("time taken: "); Serial.print(cntr); Serial.print("\n");
      haveMoved = true;
    } 

    if (haveMoved) {
      left_flag = false;
      right_flag = false;
      up_flag = false;
      down_flag = false;

      left_back = false;
      right_back = false;
      up_back = false;
      down_back = false;
      haveMoved = false;

      cntr = 0;
    }

 
    /*
    //printing values to serial port
    Serial.print("Angle: ");
    Serial.print("Pitch = "); Serial.print(pitch);
    Serial.print(" Roll = "); Serial.println(roll);

    Serial.print("Temperature in celsius = "); Serial.print(t);  
    Serial.print(" fahrenheit = "); Serial.println(tf);  
    */
    
    /*
    Serial.print("Gyroscope: ");
    Serial.print("X = "); Serial.print(gy_x);
    Serial.print(" Y = "); Serial.print(gy_y);
    Serial.print(" Z = "); Serial.println(gy_z);
    */
    
    /*
    Serial.print("Accelerometer: ");
    Serial.print("X = "); Serial.print(acc_x); // chip tilt left > 0; chip tilt right < 0
    Serial.print(" Y = "); Serial.print(acc_y); // chip face me > 0; chip face away < 0
    Serial.print(" Z = "); Serial.println(acc_z); // chip face down < 0; chip face up > 0
    */
  
    delay(500);
}

//function to convert accelerometer values into pitch and roll
void getAngle(int Ax, int Ay, int Az) 
{
    double x = Ax;
    double y = Ay;
    double z = Az;

    pitch = atan(x/sqrt((y*y) + (z*z))); //pitch calculation
    roll = atan(y/sqrt((x*x) + (z*z))); //roll calculation

    //converting radians into degrees
    pitch = pitch * (180.0/3.14);
    roll = roll * (180.0/3.14) ;
}

// function to figure out direction faced
int getDirection(int acc_x, int acc_y, int acc_z) {
  int orientation;

  if ((acc_x) < (0 -TILT_LIMIT)) {
    //Serial.print("Tilting right, "); Serial.println(acc_x);
    orientation = 2;
  } else if ((acc_x) > TILT_LIMIT) {
    //Serial.print("Tilting left, "); Serial.println(acc_x);
    orientation = 1;
  } else if ((acc_y) < (0 -TILT_LIMIT)) {
    //Serial.print("Facing away, "); Serial.println(acc_y);
    orientation = 4;
  } else if ((acc_y) > TILT_LIMIT) {
    //Serial.print("Facing me, "); Serial.println(acc_y);
    orientation = 3;
  } else if ((acc_z) < (0 -TILT_LIMIT)) {
    //Serial.print("Facing down, "); Serial.println(acc_z);
    orientation = 6;
  } else if ((acc_z) > TILT_LIMIT) {
    //Serial.print("Facing up, "); Serial.println(acc_z);
    orientation = 5;
  } else {
    orientation = 0;
  }

  return orientation;
}

// function to figure out moving or not
bool isMoving(int gy_x, int gy_y, int gy_z) {
  bool move_flag;
  if ((abs(gy_x) <= MOVE_LIMIT) && (abs(gy_y) <= MOVE_LIMIT) && (abs(gy_z) <= MOVE_LIMIT)) {
    //Serial.println("Stationary");
    move_flag = false;
  } else {
    //Serial.println("Moving");
    move_flag = true;
  }

  return move_flag;
}

// state machine (?)
void stateMachine(int prev_orientation, int curr_orientation, bool *left_flag, bool *right_flag, bool *down_flag, bool *up_flag, bool *left_back, bool *right_back, bool *down_back, bool *up_back, int *cntr) {
  if (prev_orientation == 5 && curr_orientation == 1) {
    *left_flag = true;
    //digitalWrite(BLUE_LED, HIGH);
  } else if (prev_orientation == 1 && curr_orientation == 5) {
    *left_back = true;
  } else if (prev_orientation == 5 && curr_orientation == 2) {
    *right_flag = true;
    //digitalWrite(GREEN_LED, HIGH);
  } else if (prev_orientation == 2 && curr_orientation == 5) {
    *right_back = true;
  } else if (prev_orientation == 5 && curr_orientation == 3) {
    *down_flag = true;
    //digitalWrite(YELLOW_LED, HIGH);
  } else if (prev_orientation == 3 && curr_orientation == 5) {
    *down_back = true;
  } else if (prev_orientation == 5 && curr_orientation == 4) {
    *up_flag = true;
    //digitalWrite(RED_LED, HIGH);
  } else if (prev_orientation == 4 && curr_orientation == 5) {
    *up_back = true;
  } else if (prev_orientation == curr_orientation) {
    *cntr++;
  } 
}
