//Goes around the cans using circle, NavX MXP, encoders, and servomotor/hall effect (worked at states)
#include <Servo.h>
#include <ADC.h>
#include <SPI.h>
#include "IMURegisters.h"

#define TARGET_DISTANCE (10 + 0.02)
#define CAN_DISTANCE 0.01  //0.2
#define BIAS 0    // -0.035

//geometry
#define ROBOT_WHEELBASE 0.21 //meters
#define ROBOT_LENGTH 0.295

//encoder constant values
#define ENCODER_FRONT_RESOLUTION 1440 //ticks
#define WHEEL_FRONT_DIAMETER 0.08 //meters
#define WHEEL_FRONT_CIRCUMFERENCE  PI*WHEEL_FRONT_DIAMETER //meters
#define METERS_PER_TICK_FRONT WHEEL_FRONT_CIRCUMFERENCE / ENCODER_FRONT_RESOLUTION //meters/tick

#define ENCODER_BACK_RESOLUTION 240 //ticks
#define WHEEL_BACK_DIAMETER 0.09 //meters
#define WHEEL_BACK_CIRCUMFERENCE  PI*WHEEL_BACK_DIAMETER //meters
#define METERS_PER_TICK_BACK 0.001152  //WHEEL_BACK_CIRCUMFERENCE / ENCODER_BACK_RESOLUTION //meters/tick

//hall sensor
#define HALL_PIN A10
#define HALL_VAL_MAX 65535
#define HALL_VAL_MIN 4005

//steering
//min -> PI/6
//max -> -PI/6
#define STEERING_ANGLE(measure) -STEERING_SWEEP/(HALL_VAL_MAX - HALL_VAL_MIN)*(measure - HALL_VAL_MIN) + PI/6

//circle setup
#define SIDE (1 - CAN_DISTANCE/2 + BIAS)
#define RADIUS_OF_CIRCLE ((pow(TARGET_DISTANCE, 2) + 4*pow(SIDE, 2))/(8*SIDE))
#define SERVO_BIAS_ANGLE (asin(ROBOT_WHEELBASE/RADIUS_OF_CIRCLE) + 0.0397)
#define STARTING_ROBOT_ANGLE (PI/2 - asin(TARGET_DISTANCE/2/RADIUS_OF_CIRCLE))

//system pins
#define SWITCH_PIN 22
#define LED_PIN 14

//motor
#define RELAY_PIN 2
#define MOTOR_PIN 3

//laser offsets
#define LASER_OFFSET_THETA 0.00690

//loop timing
elapsedMicros loopTime;
elapsedMillis timeFromStart;
#define LOOPS_PER_SEC 200
#define SLOWER_LOOPS_PER_SEC 5
int loopCount = 0;

//velocity control
#define END_TARGET_VELOCITY 0.5

#define K_P_VEL 120
#define LOWER_CONST_SPEED 40
#define MAX_MOTOR_ACCEL 150
#define MAX_SPEED 2
#define HIGHER_CONST_SPEED 60

//for recalibrating the measures
#define NO_RECALIBRATION -42

class Motor{
  public:
    Motor(void);
    void mBreak(void);
    void mSpeed(int PWM);//0 to 255
};

Motor::Motor(void){
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  analogWriteResolution(8);
}

void Motor::mBreak(void){
  analogWrite(MOTOR_PIN, 0);
  digitalWriteFast(RELAY_PIN, 1);
}

void Motor::mSpeed(int PWM){
  if (digitalReadFast(RELAY_PIN)){
    digitalWriteFast(RELAY_PIN, 0);
    delay(3);
  }
  analogWrite(MOTOR_PIN, PWM);
}

Motor *motor;

//servo
#define SERVO_PIN 6

#define SERVO_PWM_MIN 0
#define SERVO_PWM_MAX 180

#define STEERING_SWEEP PI/3//radians
#define PWM_ANGLE_TO_MEASURE(measure) -(SERVO_PWM_MAX - SERVO_PWM_MIN)/(STEERING_SWEEP)*(measure - PI/6) + SERVO_PWM_MIN

#define SERVO_OFFSET 21

class servo{
  public:
    servo(void);
    void angle(float ang);
  private:
    Servo *servoobj;
};

servo::servo(void){
  this->servoobj = new Servo();
  this->servoobj->attach(SERVO_PIN);  // attaches the servo to the servo object
  this->servoobj->attach(SERVO_PIN, 1000, 2000); // some motors need min/max setting
}

void servo::angle(float ang){
  this->servoobj->write((int)(PWM_ANGLE_TO_MEASURE(ang) - SERVO_OFFSET));
}

servo *front_servo;

class Hall{
  public:
    Hall(void);
    int getAdc(void);
    double getPhi(void);
  private:
    ADC *_adcobj;
    int _val;
    int _pin;
};

Hall::Hall(void){
  this->_adcobj = new ADC();
  pinMode(HALL_PIN, INPUT);
  this->_adcobj->setResolution(16);//16-bit resolution
  this->_adcobj->setAveraging(32);//TODO: see if this should be reduced
  this->_adcobj->setConversionSpeed(ADC_LOW_SPEED);
  this->_adcobj->setSamplingSpeed(ADC_LOW_SPEED);
  this->_val = this->_adcobj->analogRead(HALL_PIN, ADC_0);
}

int Hall::getAdc(void){
  this->_val = this->_adcobj->analogRead(HALL_PIN, ADC_0);
  return this->_val;
}

double Hall::getPhi(void){
  return STEERING_ANGLE(this->getAdc());
    
}

Hall *front_hall;

//encoder
#define FRONT_ENC_PIN_A  15
#define FRONT_ENC_PIN_B  16

#define R_ENC_PIN_A  18
#define R_ENC_PIN_B  17

#define L_ENC_PIN_A  20
#define L_ENC_PIN_B  19

struct encoder{
  volatile bool prevA;
  volatile bool prevB;
  volatile bool currA;
  volatile bool currB;
  volatile long int ticks;
  elapsedMicros sinceCalc;
  long int prevTicks;
  int vel;//tickspersecond
  double filteredVel;//tickspersecond
  double metersPerTick;
};

void checkEncoderZeroVelocities(struct encoder *enc, int tickTolerance);

void genericInterrupt(struct encoder *enc, int pinA, int pinB);

int setupEncoder(struct encoder *enc, int pinA, int pinB, void (*interrupt)(void), double metersPerTick);

void interruptFrontEnc(void);
void interruptREnc(void);
void interruptLEnc(void);

struct encoder front_enc;
struct encoder r_enc;
struct encoder l_enc;



#define SENSOR_WEIGHT_BACK_ENCODER_L 0.50
#define SENSOR_WEIGHT_BACK_ENCODER_R 0.50
#define SENSOR_WEIGHT_FRONT_ENCODER 0.00
#define SENSOR_VEL_WEIGHT_BACK_ENCODER_L 0.1
#define SENSOR_VEL_WEIGHT_BACK_ENCODER_R 0.1
#define SENSOR_VEL_WEIGHT_FRONT_ENCODER 0.8
#define ROBOT_VELOCITY_FILTER 0.05
#define ROBOT_X_VELOCITY_FILTER 0.08
#define ROBOT_Y_VELOCITY_FILTER 0.08

//SPI
#define CS_PIN 10

//robot
class Robot{
  public:
    Robot(void);
    void setupNavX(void);
    void pollNavX(void);
    int getNavXOpStatus(void);
    int getNavXStatus(void);
    void makeUpdateRate200Hz(void);
    double getU(void);
    double getX(void);
    double getY(void);
    double getTheta(void);//angle to floor
    double getPhi(void);//steering angle
    
    double getTargetVelocity(double y_position);
    double getEstimatedCoastingDistance(double robot_velocity);
    
    long int getFrontEnc(void);//ticks
    long int getREnc(void);//ticks
    long int getLEnc(void);//ticks
    
    double getFrontEncVel(void);//ticks/sec
    double getREncVel(void);//ticks/sec
    double getLEncVel(void);//ticks/sec
    double getRobotVel(void);//meters/sec
    double filtered_robot_vel;  //meters/sec
    double filtered_robot_x_vel; //meters/sec
    double filtered_robot_y_vel;  //meters/sec
    
    
    void recalibrateMeasures(double x, double y, double theta, double phi, double u, int long l_enc_ticks, int long r_enc_ticks);
    void updatePose(void);
    void logPose(double wanted_x);
    void logEncoders(void);
    void logEncoderVelocities(void);
    
  private:
    byte _spi_data[512];
    double _u;//total distance
    double _x;
    double _y;
    double _theta;
    double _phi;

    double _phi_offset;
    double _theta_offset;

    int long _prev_front_enc_ti; 
    int long _prev_l_enc_ti;
    int long _prev_r_enc_ti;

    struct encoder *_front_enc;
    struct encoder *_r_enc;
    struct encoder *_l_enc;
};

bool first_finished_flag = false;
void finished(void);


Robot::Robot(void){

  front_hall = new Hall();
  motor = new Motor();
  front_servo = new servo();

  this->_front_enc = &front_enc;
  this->_r_enc = &r_enc;
  this->_l_enc = &l_enc;

  setupEncoder(&front_enc, FRONT_ENC_PIN_A, FRONT_ENC_PIN_B, interruptFrontEnc, METERS_PER_TICK_FRONT);
  setupEncoder(&r_enc, R_ENC_PIN_A, R_ENC_PIN_B, interruptREnc, METERS_PER_TICK_BACK);
  setupEncoder(&l_enc, L_ENC_PIN_A, L_ENC_PIN_B, interruptLEnc, METERS_PER_TICK_BACK);

  this->_theta_offset = 0;
  this->_phi_offset = 0;

  //this->recalibrateMeasures(0, 0, PI/2, 0, 0, 0, 0);

  this->setupNavX();
 
}

void Robot::setupNavX(void){
  pinMode(CS_PIN,OUTPUT);
  digitalWriteFast(SS, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(8); /* 16Mhz/32 = 500kHz; /16=1Mhz; /8=2Mhz */ 
}

/*void Robot::pollNavX(void){
  uint8_t spi_crc;
  uint8_t spi_data[3];

  // Transmit SPI data
  do{
    spi_data[0] = 0;   // Start register address (high bit clear == read)
    spi_data[1] = NAVX_REG_YAW_H + 1; // Number of bytes to read
    spi_data[2] = IMURegisters::getCRC(spi_data,2);
    
    digitalWriteFast(SS, LOW);
    
    for ( int spi_data_index = 0; spi_data_index < 3; spi_data_index++ ) {
        SPI.transfer(spi_data[spi_data_index]);
    }
    
    digitalWriteFast(SS, HIGH);
    
    delayMicroseconds(200); // Read 0xFFs until ready
    digitalWriteFast(SS, LOW);
    for ( int x = 0; x <= NAVX_REG_YAW_H + 1; x++ ) {
        this->_spi_data[x] = SPI.transfer((byte)0xFF);
    }
    digitalWriteFast(SS, HIGH);  
    spi_crc = IMURegisters::getCRC(this->_spi_data, NAVX_REG_YAW_H + 1);

    //SPI checksum
    if (spi_crc != this->_spi_data[NAVX_REG_YAW_H + 1]){
      Serial.print("SPI CRC ERROR!  ");
      Serial.println(spi_crc);
    }
  }while(spi_crc != this->_spi_data[NAVX_REG_YAW_H + 1]);

}*/
void Robot::pollNavX(void){
  uint8_t spi_crc;
  uint8_t spi_data[3];

  // Transmit SPI data
  do{
    spi_data[0] = 0;   // Start register address (high bit clear == read)
    spi_data[1] = 32; // Number of bytes to read
    spi_data[2] = IMURegisters::getCRC(spi_data,2);
    
    digitalWriteFast(SS, LOW);
    
    for ( int spi_data_index = 0; spi_data_index < 3; spi_data_index++ ) {
        SPI.transfer(spi_data[spi_data_index]);
    }
    
    digitalWriteFast(SS, HIGH);
    
    delayMicroseconds(200); // Read 0xFFs until ready
    digitalWriteFast(SS, LOW);
    for ( int x = 0; x <= 32; x++ ) {
        this->_spi_data[x] = SPI.transfer((byte)0xFF);
    }
    digitalWriteFast(SS, HIGH);  
    spi_crc = IMURegisters::getCRC(this->_spi_data,32);

    //SPI checksum
    if (spi_crc != this->_spi_data[32]){
      Serial.print("SPI CRC ERROR!  ");
      Serial.println(spi_crc);
    }
  }while(spi_crc != this->_spi_data[32]);

}



int Robot::getNavXStatus(void){
  return this->_spi_data[NAVX_REG_CAL_STATUS];
}

int Robot::getNavXOpStatus(void){
  return this->_spi_data[NAVX_REG_OP_STATUS];
}

void Robot::makeUpdateRate200Hz(void){
    uint8_t spi_data[3];
    
  /* Transmit SPI data */
  spi_data[0] = 0x80 | NAVX_REG_UPDATE_RATE_HZ; // Starting Register Address + Write Request
  spi_data[1] = 200;
  spi_data[2] = IMURegisters::getCRC(spi_data,2);
  digitalWriteFast(SS, LOW);
  for ( int i = 0; i < 3; i++ ) {
      SPI.transfer(spi_data[i]);
  }
  digitalWriteFast(SS, HIGH);
}

void Robot::recalibrateMeasures(double x, double y, double theta, double phi, double u, int long l_enc_ticks, int long r_enc_ticks){
  
  if(theta != NO_RECALIBRATION){
    this->_theta = theta;
    this->_theta_offset = this->getTheta() - theta;
  } 
  this->_x = x - ROBOT_LENGTH * cos(this->getTheta());
  this->_y = y - ROBOT_LENGTH * sin(this->getTheta());
  /*Serial.print("RESET [offset = ");
  Serial.print(this->getTheta());
  Serial.print(" - ");
  Serial.print(theta);
  Serial.println("]");*/
  //this->_phi = phi;
  this->_phi_offset = 0.06;//front_hall->getPhi() - phi;
  this->_u = u;

  this->_prev_front_enc_ti = 0;//TODO
  this->_prev_l_enc_ti = l_enc_ticks;
  this->_prev_r_enc_ti = r_enc_ticks;

  this->_front_enc->ticks = 0;
  this->_r_enc->ticks = 0;
  this->_l_enc->ticks = 0;
}

double Robot::getU(void){
  return this->_u;
}

double Robot::getX(void){
  return this->_x;
}
double Robot::getY(void){
  return this->_y;
}
double Robot::getTheta(void){
  char *angle = new char[2];
  *angle = this->_spi_data[NAVX_REG_YAW_L];
  *(angle+1) = this->_spi_data[NAVX_REG_YAW_H];
  this->_theta = -PI/180*(double)IMURegisters::decodeProtocolSignedHundredthsFloat(angle);//pointer of lower of two bytes

  free(angle);
  return this->_theta - this->_theta_offset;
}

double Robot::getPhi(void){
  this->_phi = front_hall->getPhi();
  return this->_phi - this->_phi_offset;
}

double Robot::getTargetVelocity(double y_position){
  return END_TARGET_VELOCITY;
}

double Robot::getEstimatedCoastingDistance(double robot_velocity){
  return 0.1*robot_velocity*robot_velocity + 0.212*robot_velocity;
}

long int Robot::getFrontEnc(void){//ticks
  return this->_front_enc->ticks;
}

long int Robot::getREnc(void){//ticks
  return this->_r_enc->ticks;
}
long int Robot::getLEnc(void){//ticks
  return this->_l_enc->ticks;
}


double Robot::getFrontEncVel(void){//ticks/sec
  return this->_front_enc->filteredVel;
}

double Robot::getREncVel(void){//ticks/sec
  return this->_r_enc->filteredVel;
}
double Robot::getLEncVel(void){//ticks/sec
  return this->_l_enc->filteredVel;
}

double Robot::getRobotVel(void){//meters/sec
  return SENSOR_VEL_WEIGHT_BACK_ENCODER_L*this->_l_enc->filteredVel*METERS_PER_TICK_BACK + SENSOR_VEL_WEIGHT_BACK_ENCODER_R*this->_r_enc->filteredVel*METERS_PER_TICK_BACK + SENSOR_VEL_WEIGHT_FRONT_ENCODER*this->_front_enc->filteredVel*METERS_PER_TICK_FRONT*cos(this->getPhi()); 
}

void Robot::updatePose(void){
  //this->_phi = front_hall->getPhi();

  int dFenc = (int)(this->_front_enc->ticks - this->_prev_front_enc_ti);
  int dLenc = (int)(this->_l_enc->ticks - this->_prev_l_enc_ti);
  int dRenc = (int)(this->_r_enc->ticks - this->_prev_r_enc_ti); 
  this->_prev_front_enc_ti = this->_front_enc->ticks;
  this->_prev_l_enc_ti = this->_l_enc->ticks;
  this->_prev_r_enc_ti = this->_r_enc->ticks;

  //TODO: write slipping analysis and navigation correction
  double du = SENSOR_WEIGHT_BACK_ENCODER_L*dLenc * METERS_PER_TICK_BACK +SENSOR_WEIGHT_BACK_ENCODER_R*dRenc * METERS_PER_TICK_BACK + SENSOR_WEIGHT_FRONT_ENCODER*dFenc*METERS_PER_TICK_FRONT;
  this->_u += du;

  this->pollNavX();
  
  // -PI to +PI
  this->_theta = this->getTheta();
  
  
  double dx = du*cos(this->_theta);
  double dy = du*sin(this->_theta);
  
  this->filtered_robot_x_vel += ROBOT_X_VELOCITY_FILTER * (dx * LOOPS_PER_SEC - this->filtered_robot_x_vel);
  this->filtered_robot_y_vel += ROBOT_Y_VELOCITY_FILTER * (dy * LOOPS_PER_SEC - this->filtered_robot_y_vel);

  this->_x += dx;
  this->_y += dy; 
}

void Robot::logPose(double error_x){
  Serial.print("pose( x (cm) = ");
  Serial.print(100*(this->getX() + ROBOT_LENGTH * cos(this->getTheta())));
  Serial.print(", x-err (cm) = ");
  Serial.print(error_x*100);  
  Serial.print(", y (m) = ");
  Serial.print(this->getY() + ROBOT_LENGTH * sin(this->getTheta()), 3);
  Serial.print(", theta (deg)= ");
  Serial.print(this->getTheta()*180/PI);
  Serial.print(", phi (deg)= ");
  Serial.print(this->getPhi()*180/PI);
  Serial.print(", vel (m/s)= ");
  Serial.print(this->getRobotVel());
  Serial.print(", time (s)= ");
  Serial.print(timeFromStart/1000.0, 3);
  Serial.println(")");
}

void Robot::logEncoders(void){
  
  static long prev_ticks_FrontEnc = 0;
  static long prev_ticks_REnc = 0;
  static long prev_ticks_LEnc = 0;

  Serial.print("encoders( front = ");
  Serial.print(this->getFrontEnc()*METERS_PER_TICK_FRONT, 3);
  Serial.print(", right = ");
  Serial.print(this->getREnc()*METERS_PER_TICK_BACK, 3);
  Serial.print(", left = ");
  Serial.print(this->getLEnc()*METERS_PER_TICK_BACK, 3);

  Serial.print(" dFront (cm)= ");
  Serial.print((this->getFrontEnc() - prev_ticks_FrontEnc)*METERS_PER_TICK_FRONT*100);
  Serial.print(", dRight (cm)= ");
  Serial.print((this->getREnc() - prev_ticks_REnc)*METERS_PER_TICK_BACK*100);
  Serial.print(", dLeft (cm)= ");
  Serial.print((this->getLEnc() - prev_ticks_LEnc)*METERS_PER_TICK_BACK*100);

  prev_ticks_FrontEnc = this->getFrontEnc();
  prev_ticks_REnc = this->getREnc();
  prev_ticks_LEnc = this->getLEnc();
  
  Serial.println(")");  
}

void Robot::logEncoderVelocities(void){
  Serial.print("encoder vel( front = ");
  Serial.print(this->getFrontEncVel());
  Serial.print(", right = ");
  Serial.print(this->getREncVel());
  Serial.print(", left = ");
  Serial.print(this->getLEncVel());
  Serial.println(")");  
}

Robot *vehicle;

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  
  delay(10000);

  //front_hall = new Hall();
  //motor = new Motor();
  //front_servo = new servo();
  
  vehicle = new Robot();
  front_servo->angle(0);

  //Wait for NavX to Calibrate
  Serial.println("Waiting for NavX...");
  
  vehicle->pollNavX();
  while(vehicle->getNavXOpStatus()!=NAVX_OP_STATUS_NORMAL){
    delay(300);
    vehicle->pollNavX();
    switch(vehicle->getNavXStatus()){
      case(NAVX_OP_STATUS_INITIALIZING):
        Serial.println("NAVX_OP_STATUS_INITIALIZING");
        break;
      case(NAVX_OP_STATUS_SELFTEST_IN_PROGRESS):
        Serial.println("NAVX_OP_STATUS_SELFTEST_IN_PROGRESS");
        break;
      case(NAVX_OP_STATUS_ERROR):
        Serial.println("NAVX_OP_STATUS_ERROR");
        break;
      case(NAVX_OP_STATUS_IMU_AUTOCAL_IN_PROGRESS):
        Serial.println("NAVX_OP_STATUS_IMU_AUTOCAL_IN_PROGRESS");
        break;
      default:
        Serial.println("NavX Unknown Status...");
        break;
    };
  }  
  Serial.println("NAVX_OP_STATUS_NORMAL");
  
  //navx operation normal, wait for human//TODO change this to wait for calibration finish
  
  pinMode(SWITCH_PIN, INPUT); 


  //wait for button to be pressed
  while(!digitalReadFast(SWITCH_PIN)){
    Serial.print("Waiting for User Button... Angle(deg):");
    vehicle->pollNavX();

    Serial.println(vehicle->getTheta()*180.0/PI);

    digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    
    delay(100);
  }
  
  //wait for button to be released
  while(digitalReadFast(SWITCH_PIN)){
    
    delay(50);
    //vehicle->pollNavX();

    digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
  } 
  delay(500);

  vehicle->makeUpdateRate200Hz();
  
  digitalWriteFast(LED_PIN, HIGH);
  
  vehicle->pollNavX();
  vehicle->recalibrateMeasures(0, 0, PI/2 + LASER_OFFSET_THETA, 0, 0, 0, 0);
  
  while(!digitalReadFast(SWITCH_PIN)){   
    delay(5);

    vehicle->pollNavX();

    digitalWriteFast(LED_PIN, abs(vehicle->getTheta() - STARTING_ROBOT_ANGLE) < 0.01);
   } //wait for button to be pressed to signal run start
   
  delay(500);
  vehicle->pollNavX();
  vehicle->recalibrateMeasures(0, 0, NO_RECALIBRATION, 0, 0, 0, 0);
  
  timeFromStart = 0;
}

double prevError=0;
double dError = 0;

//for arc follower
#define  K_P_ARC_FOLLOW  5
#define K_D_ARC_FOLLOW 1.0
#define dERROR_FILTER 0.5

int motorSpeed = MAX_MOTOR_ACCEL;



void loop(){

  loopTime = 0;
  loopCount++;
  
  //very occaisionally get CRC errors
  vehicle->updatePose();

  //abort switch
  if(digitalReadFast(SWITCH_PIN)){
    finished();
    while(digitalReadFast(SWITCH_PIN)); //wait until switch released
    delay(20); //for switch bounce
  }
  
  if(!first_finished_flag){
  
    double theta = vehicle->getTheta();
    
    double x = vehicle->getX() + ROBOT_LENGTH * cos(theta);
    double y = vehicle->getY() + ROBOT_LENGTH * sin(theta);
    
    //velocity controller
    
    double xDesired = sqrt(pow(RADIUS_OF_CIRCLE, 2) - pow((y - TARGET_DISTANCE/2),2)) - RADIUS_OF_CIRCLE + SIDE;

    double errorX = x - xDesired;

    dError += dERROR_FILTER * ((errorX - prevError)*LOOPS_PER_SEC - dError);// derivative of error, meters/sec

    prevError = errorX;
  
    double angleDesired = -(K_P_ARC_FOLLOW*errorX/(vehicle->getRobotVel() + 0.2) + K_D_ARC_FOLLOW*dError)/(vehicle->getRobotVel() + 0.3) - SERVO_BIAS_ANGLE;//atan(-errSERVO_BIAS_ANGLEorX / 0.20) + theta - PI/2;
    //TODO:: INVESTIGATE
    if(y >= TARGET_DISTANCE){
      finished();
    }else if(TARGET_DISTANCE - y < vehicle->getEstimatedCoastingDistance(vehicle->getRobotVel())){//if on last segment and almost reached goal
      motor->mBreak();
    }else if((y > TARGET_DISTANCE/2 - 0.02) and (y < TARGET_DISTANCE/2 + 0.1)){
      motor->mSpeed(0);
    }else if(y < 8.5){
      motor->mSpeed(min(K_P_VEL*(MAX_SPEED - vehicle->getRobotVel()) + HIGHER_CONST_SPEED, MAX_MOTOR_ACCEL));
    }else if(y > 8.8){
      //motorSpeed = (int) (ROBOT_VELOCITY_FILTER*max(K_P_VEL*(END_TARGET_VELOCITY - vehicle->getRobotVel()) + CONST_SPEED, 0) + (1-ROBOT_VELOCITY_FILTER)*motorSpeed);
      motor->mSpeed(K_P_VEL*(vehicle->getTargetVelocity(y) - vehicle->getRobotVel()) + LOWER_CONST_SPEED);
    }else if(y > 8.5){
      motor->mBreak();
    }

    //right positive

    front_servo->angle(angleDesired);
  
    if(vehicle->getRobotVel() != 0){
      vehicle->logPose(errorX);
    }
    
  }else{//finished entire arc
    finished();
    vehicle->logPose(vehicle->getX() + ROBOT_LENGTH * cos(vehicle->getTheta()));

  }

  if (loopCount >= LOOPS_PER_SEC/SLOWER_LOOPS_PER_SEC) {    //Stuff that should run slower than the main loop
    
    checkEncoderZeroVelocities(&front_enc);
    checkEncoderZeroVelocities(&r_enc);
    checkEncoderZeroVelocities(&l_enc);
    
    //vehicle->logPose();
    digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
    loopCount = 0;
  }
  

  while(loopTime < 1e6/LOOPS_PER_SEC);
  
}

void checkEncoderZeroVelocities(struct encoder *enc){
  if(enc->ticks == enc->prevTicks){
    enc->filteredVel = 0;
    enc->vel = 0;
  }
  enc->prevTicks = enc->ticks;
}

void genericInterrupt(struct encoder *enc, int pinA, int pinB){
  enc->currA = digitalReadFast(pinA);
  enc->currB = digitalReadFast(pinB);
  
  enc->vel = 1000000 / enc->sinceCalc;
  enc->sinceCalc = 0;
  if(enc->vel < 10/enc->metersPerTick){
    enc->filteredVel += ROBOT_VELOCITY_FILTER*(enc->vel - enc->filteredVel);
  }
  
  int currA = enc->currA;
  int currB = enc->currB;

  int prevA = enc->prevA;
  int prevB = enc->prevB;

  if(currA && currB){
    if(!prevA && prevB){
      enc->ticks--;
    }else if(prevA && !prevB){
      enc->ticks++;
    }
  }else if(currA && !currB){
    if(prevA && prevB){
      enc->ticks--;
    }else if(!prevA && !prevB){
      enc->ticks++;
    }
  }else if(!currA && !currB){
    if(prevA && !prevB){
      enc->ticks--;
    }else if(!prevA && prevB){
      enc->ticks++;
    }
  }else if(!currA && currB){
    if(!prevA && !prevB){
      enc->ticks--;
    }else if(prevA && prevB){
      enc->ticks++;
    }
  }
  
  enc->prevA = enc->currA;
  enc->prevB = enc->currB;
}

int setupEncoder(struct encoder *enc, int pinA, int pinB, void (*interrupt)(void), double metersPerTick){
  enc->ticks=0;
  enc->vel=0;
  enc->filteredVel = 0;
  enc->sinceCalc=0;
  enc->metersPerTick = metersPerTick;
  
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(pinA), interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), interrupt, CHANGE);
  
  enc->prevA = digitalReadFast(pinA);
  enc->prevB = digitalReadFast(pinB);

  return 0;
}

void interruptFrontEnc(void){
  genericInterrupt(&front_enc, FRONT_ENC_PIN_A, FRONT_ENC_PIN_B);
}
void interruptREnc(void){
  genericInterrupt(&r_enc, R_ENC_PIN_A, R_ENC_PIN_B);
}
void interruptLEnc(void){
  genericInterrupt(&l_enc, L_ENC_PIN_A, L_ENC_PIN_B);
}

void finished(void){
  if(!first_finished_flag){
    motor->mBreak();
    front_servo->angle(-SERVO_BIAS_ANGLE);
    //Serial.println("finished");   
    first_finished_flag = true;
  }  
}
