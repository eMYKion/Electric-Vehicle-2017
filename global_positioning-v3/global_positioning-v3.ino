//Goes around the cans using splines, NavX MXP, encoders, and servomotor/hall effect

//system
#define SWITCH_PIN 22

//motor
#define RELAY_PIN 2
#define MOTOR_PIN 3

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
  delay(5);
}

void Motor::mSpeed(int PWM){
  digitalWriteFast(RELAY_PIN, 0);
  delay(5);
  analogWrite(MOTOR_PIN, PWM);
}

Motor *motor;

//servo
#include <Servo.h>

#define SERVO_PIN 6

#define SERVO_PWM_MIN 0
#define SERVO_PWM_MAX 180

#define STEERING_SWEEP PI/3//radians
#define PWM_ANGLE_TO_MEASURE(measure) -(SERVO_PWM_MAX - SERVO_PWM_MIN)/(STEERING_SWEEP)*(measure - PI/6) + SERVO_PWM_MIN

class servo{
  public:
    servo(void);
    void angle(float ang);
  private:
    Servo *servoobj;
};

servo::servo(void){
  this->servoobj = new Servo();
  this->servoobj->attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  this->servoobj->attach(SERVO_PIN, 1000, 2000); // some motors need min/max setting
}

#define SERVO_OFFSET 21

void servo::angle(float ang){
  this->servoobj->write((int)(PWM_ANGLE_TO_MEASURE(ang) - SERVO_OFFSET));
}

servo *front_servo;


//geometry
#define BACKWHEEL_TO_FRONT_DOWEL 0.265 //meters

#define ROBOT_WHEELBASE 0.2 //meters

#define ENCODER_FRONT_RESOLUTION 1440 //ticks
#define WHEEL_FRONT_DIAMETER 0.08 //meters
#define WHEEL_FRONT_CIRCUMFERENCE  PI*WHEEL_FRONT_DIAMETER //meters
#define METERS_PER_TICK_FRONT WHEEL_FRONT_CIRCUMFERENCE / ENCODER_FRONT_RESOLUTION //meters/tick

#define ENCODER_BACK_RESOLUTION 250 //ticks
#define WHEEL_BACK_DIAMETER 0.09 //meters
#define WHEEL_BACK_CIRCUMFERENCE  PI*WHEEL_BACK_DIAMETER //meters
#define METERS_PER_TICK_BACK 0.001152//WHEEL_BACK_CIRCUMFERENCE / ENCODER_BACK_RESOLUTION //meters/tick

//hall sensor
#include <ADC.h>
#define HALL_PIN A10
#define HALL_VAL_MAX 65535
#define HALL_VAL_MIN 4005

//steering
//min -> PI/6
//max -> -PI/6
#define STEERING_ANGLE(measure) -STEERING_SWEEP/(HALL_VAL_MAX - HALL_VAL_MIN)*(measure - HALL_VAL_MIN) + PI/6

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
  double vel;//tickspersecond
};

void checkEncoderZeroVelocities(struct encoder *enc, int tickTolerance);

void genericInterrupt(struct encoder *enc, int pinA, int pinB);

int setupEncoder(struct encoder *enc, int pinA, int pinB, void (*interrupt)(void));

void interruptFrontEnc(void);
void interruptREnc(void);
void interruptLEnc(void);

struct encoder front_enc;
struct encoder r_enc;
struct encoder l_enc;



#define SENSOR_WEIGHT_BACK_ENCODER_L 0.50
#define SENSOR_WEIGHT_BACK_ENCODER_R 0.50
#define SENSOR_WEIGHT_FRONT_ENCODER 0.00

#include <SPI.h>
#include "IMURegisters.h"

#define CS_PIN 10


//robot
class Robot{
  public:
    Robot(void);
    void setupNavX(void);
    void pollNavX(void);
    int getNavXOpStatus(void);
    int getNavXStatus(void);
    double getU(void);
    double getX(void);
    double getY(void);
    double getTheta(void);//angle to floor
    double getPhi(void);//steering angle
    long int getFrontEnc(void);//ticks
    long int getREnc(void);//ticks
    long int getLEnc(void);//ticks
    double getFrontEncVel(void);//ticks/sec
    double getREncVel(void);//ticks/sec
    double getLEncVel(void);//ticks/sec
    double getRobotVel(void);//meters/sec
    
    void recalibrateMeasures(double x, double y, double theta, double phi, double u, int long l_enc_ticks, int long r_enc_ticks);
    void updatePose(void);
    void logPose(void);
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

  setupEncoder(&front_enc, FRONT_ENC_PIN_A, FRONT_ENC_PIN_B, interruptFrontEnc);
  setupEncoder(&r_enc, R_ENC_PIN_A, R_ENC_PIN_B, interruptREnc);
  setupEncoder(&l_enc, L_ENC_PIN_A, L_ENC_PIN_B, interruptLEnc);

  this->_theta_offset = 0;
  this->_phi_offset = 0;

  //this->recalibrateMeasures(0, 0, PI/2, 0, 0, 0, 0);

  this->setupNavX();
 
}

void Robot::setupNavX(void){
  pinMode(CS_PIN,OUTPUT);
  digitalWrite(SS, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(128); /* 16Mhz/32 = 500kHz; /16=1Mhz; /8=2Mhz */ 
}

void Robot::pollNavX(void){
  int crc_count = 0;
  
  if(crc_count <= 3){
    uint8_t spi_crc;
    uint8_t spi_data[3];
  
    // Transmit SPI data
    spi_data[0] = 0;   // Start register address (high bit clear == read)
    spi_data[1] = 32; // Number of bytes to read
    spi_data[2] = IMURegisters::getCRC(spi_data,2);
    
    digitalWrite(SS, LOW);
    
    for ( int spi_data_index = 0; spi_data_index < 3; spi_data_index++ ) {
        SPI.transfer(spi_data[spi_data_index]);
    }
    
    digitalWrite(SS, HIGH);
    
    delayMicroseconds(200); // Read 0xFFs until ready
    digitalWrite(SS, LOW);
    for ( int x = 0; x <= 32; x++ ) {
        this->_spi_data[x] = SPI.transfer((byte)0xFF);
    }
    digitalWrite(SS, HIGH);  
    spi_crc = IMURegisters::getCRC(this->_spi_data,32);
  
    //SPI checksum
    if ( spi_crc != this->_spi_data[32] ) {
        crc_count++;
        //Serial.print("SPI CRC ERROR!  ");
        //Serial.println(spi_crc);
    }
  }else{
    finished();
  }
}

int Robot::getNavXStatus(void){
  return this->_spi_data[NAVX_REG_CAL_STATUS];
}

int Robot::getNavXOpStatus(void){
  return this->_spi_data[NAVX_REG_OP_STATUS];
}

void Robot::recalibrateMeasures(double x, double y, double theta, double phi, double u, int long l_enc_ticks, int long r_enc_ticks){
  this->_x = x;
  this->_y = y;
  this->_theta = theta;
  this->_theta_offset = this->getTheta() - theta;
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
  /*Serial.print("[theta_f = ");
  Serial.print(this->_theta);
  Serial.print(" - ");
  Serial.print(this->_theta_offset);
  Serial.println("]");*/
  return this->_theta - this->_theta_offset;
}

double Robot::getPhi(void){
  this->_phi = front_hall->getPhi();
  return this->_phi - this->_phi_offset;
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
  return this->_front_enc->vel;
}

double Robot::getREncVel(void){//ticks/sec
  return this->_r_enc->vel;
}
double Robot::getLEncVel(void){//ticks/sec
  return this->_l_enc->vel;
}

double Robot::getRobotVel(void){//meters/sec
  return SENSOR_WEIGHT_BACK_ENCODER_L*this->_l_enc->vel*METERS_PER_TICK_BACK + SENSOR_WEIGHT_BACK_ENCODER_R*this->_r_enc->vel*METERS_PER_TICK_BACK + SENSOR_WEIGHT_FRONT_ENCODER*this->_front_enc->vel*METERS_PER_TICK_FRONT; 
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
  
  char *signed_hundredths_float = new char[2];
  *(signed_hundredths_float) = this->_spi_data[NAVX_REG_YAW_L];
  *(signed_hundredths_float+1) = this->_spi_data[NAVX_REG_YAW_H];
  
  // -PI to +PI
  this->_theta = this->getTheta();
  
  
  double dx = du*cos(this->_theta);
  double dy = du*sin(this->_theta);

  this->_x += dx;
  this->_y += dy;
}

void Robot::logPose(void){
  Serial.print("pose( x = ");
  Serial.print(this->getX());
  Serial.print(", y = ");
  Serial.print(this->getY());
  Serial.print(", theta = ");
  Serial.print(this->getTheta());
  Serial.print(", phi = ");
  Serial.print(this->getPhi());
  Serial.println(")");
}

void Robot::logEncoders(void){
  Serial.print("encoders( front = ");
  Serial.print(this->getFrontEnc());
  Serial.print(", right = ");
  Serial.print(this->getREnc());
  Serial.print(", left = ");
  Serial.print(this->getLEnc());
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


//Splines
#define SPLINE_DISTANCE_TOLERANCE 0.05//5cm of point to point tolerance for splines




#define SEGMENTS 2 //p1 -> p2 -> p3 defined to be (number of points - 1)
#define POINTS 3

//spline data format is x0 , x1, ..., y0, y1, ..., v_x0, v_x1, ..., v_y0, v_y1, ...

#define TARGET_DISTANCE 10.0
#define CAN_DISTANCE 0.3
#define BIAS 0.05

float spline_data[POINTS*4] = {0.0, 1.0-CAN_DISTANCE/2.0 + BIAS, 0.0,
                               0.0, TARGET_DISTANCE/2.0, TARGET_DISTANCE,
                               0.0, 0.0, 0.0,
                               0.0, 2.0, 0.0};

//e.g.  d(4, 1) means d_y at the 4th segment

class Splines{
  public:
    Splines(void);

    void loadSplineData(float *spline_data);

    //n is number of points
    //this defines n-1 SEGMENTS (1,2,...,n-1)
    
    void printData(void);

    float v_f(int seg, int axis);
    float v_i(int seg, int axis);
    float r_f(int seg, int axis);
    float r_i(int seg, int axis);

    float a(int seg, int axis);
    float b(int seg, int axis);
    float c(int seg, int axis);
    float d(int seg, int axis);
    
    float x(int seg, float t);
    float y(int seg, float t);

    bool closeTo(float x0, float y0, float x1, float y1);

    int getCurrentSegment(void);
    void nextSegment(void);

  private:
    float *x_vels;
    float *y_vels;
    float *x_pos;
    float *y_pos;

    int curr_segment;//goes from 1,2,3,...,n-1
    
};

Splines::Splines(void){
  this->x_vels = new float[POINTS];
  this->y_vels = new float[POINTS];
  this->x_pos = new float[POINTS];
  this->y_pos = new float[POINTS];

  this->curr_segment = 1;
}

void Splines::loadSplineData(float *spline_data){
  
  for(int i = 0; i<POINTS; i++){
    this->x_pos[i] = spline_data[i];
    this->y_pos[i] = spline_data[POINTS + i];

    this->x_vels[i] = spline_data[POINTS*2 + i];
    this->y_vels[i] = spline_data[POINTS*3 + i];
  }
}

void Splines::printData(void){
  Serial.println("x_vel[i] | y_vel[i] | x_pos[i] | y_pos[i]");
  for(int i=0;i<POINTS;i++){
    Serial.print(this->x_vels[i]);
    Serial.print(" | ");
    Serial.print(this->y_vels[i]);
    Serial.print(" | ");
    Serial.print(this->x_pos[i]);
    Serial.print(" | ");
    Serial.print(this->y_pos[i]);
    Serial.print("\n");
  }
}

float Splines::v_f(int seg, int axis){
  if(axis==1){
    return this->y_vels[seg];
  }else{
    return this->x_vels[seg]; 
  }
}
float Splines::v_i(int seg, int axis){
  if(axis==1){
    return this->y_vels[seg-1];
  }else{
    return this->x_vels[seg-1]; 
  }
}

float Splines::r_f(int seg, int axis){
  if(axis==1){
    return this->y_pos[seg];
  }else{
    return this->x_pos[seg]; 
  }  
}

float Splines::r_i(int seg, int axis){
  if(axis==1){
    return this->y_pos[seg-1];
  }else{
    return this->x_pos[seg-1]; 
  }
}

float Splines::a(int seg, int axis){
  return this->v_f(seg, axis) + this->v_i(seg,axis) -2*(this->r_f(seg, axis)) + 2*(this->r_i(seg, axis));
}

float Splines::b(int seg, int axis){
  return -(this->v_f(seg,axis)) -2*(this->v_i(seg,axis)) + 3*(this->r_f(seg,axis)) -3*(this->r_i(seg,axis));
}

float Splines::c(int seg, int axis){
  return this->v_i(seg, axis);  
}

float Splines::d(int seg, int axis){
  return this->r_i(seg, axis);
}

//this is what the user cares about
float Splines::x(int seg, float t){//t must be between 0.0 and 1.0
  if((t < 0.0 ) || (t > 1.0)){
    //Serial.println("ACCESSING SPLINE PARAMETER OUT OF BOUNDS");
  }
  return this->a(seg,0)*(t*t*t) + this->b(seg,0)*(t*t) + this->c(seg,0)*(t) + this->d(seg,0);
}

float Splines::y(int seg, float t){//t must be between 0.0 and 1.0
 if((t < 0.0 ) || (t > 1.0)){
  //Serial.println("ACCESSING SPLINE PARAMETER OUT OF BOUNDS");
 }
 return this->a(seg,1)*(t*t*t) + this->b(seg,1)*(t*t) + this->c(seg,1)*(t) + this->d(seg,1);
}

bool Splines::closeTo(float x0, float y0, float x1, float y1){
  return sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0)) < SPLINE_DISTANCE_TOLERANCE;
}

int Splines::getCurrentSegment(void){
  return this->curr_segment;
}

void Splines::nextSegment(void){
  this->curr_segment++;
}

//math

int signum(float x){
  if(x>0){
    return 1;
  }else if(x<0){
    return -1;
  }else{
    return 0;  
  }    
}

Splines *spline;

void setup() {
  //Serial.begin(9600);
  //while(!Serial);
  delay(2000);

  //front_hall = new Hall();
  //motor = new Motor();
  //front_servo = new servo();
  
  vehicle = new Robot();

  spline = new Splines();
  spline->loadSplineData(spline_data);

  
  

  //Wait for NavX to Calibrate
  //Serial.println("Waiting for NavX...");
  
  
  vehicle->pollNavX();
  while(vehicle->getNavXOpStatus()!=NAVX_OP_STATUS_NORMAL){
    delay(100);
    vehicle->pollNavX();
    switch(vehicle->getNavXStatus()){
      case(NAVX_OP_STATUS_INITIALIZING):
        //Serial.println("NAVX_OP_STATUS_INITIALIZING");
        break;
      case(NAVX_OP_STATUS_SELFTEST_IN_PROGRESS):
        //Serial.println("NAVX_OP_STATUS_SELFTEST_IN_PROGRESS");
        break;
      case(NAVX_OP_STATUS_ERROR):
        //Serial.println("NAVX_OP_STATUS_ERROR");
        break;
      case(NAVX_OP_STATUS_IMU_AUTOCAL_IN_PROGRESS):
        //Serial.println("NAVX_OP_STATUS_IMU_AUTOCAL_IN_PROGRESS");
        break;
      default:
        //Serial.println("NavX Unknown Status...");
        break;
    };
  }  
  //Serial.println("NAVX_OP_STATUS_NORMAL");
  
  //navx operation normal, wait for human//TODO change this to wait for calibration finish

  
  pinMode(SWITCH_PIN, INPUT); 


  
  while(!digitalRead(SWITCH_PIN)){
    //Serial.println("Waiting for User Button...");
    delay(100);
  }
  delay(1000);
  //Serial.println("Started");
  
  vehicle->recalibrateMeasures(0, 0, PI/2, 0, 0, 0, 0);
}

//using y as the internal parameter
double startY = 0;

double prevError=0;

const float K_p = 3;
const float K_d = 1;

int motorSpeed = 55;

void loop(){
  
  delay(20);
  
  //very occaisionally get CRC errors

  vehicle->updatePose();
  //vehicle->logPose();

  checkEncoderZeroVelocities(&front_enc, 4);
  checkEncoderZeroVelocities(&r_enc, 4);
  checkEncoderZeroVelocities(&l_enc, 4);

  //spline code goes here

  if(digitalRead(SWITCH_PIN)){
    finished();
  }

  int currSeg = spline->getCurrentSegment();
  
  if(currSeg <= SEGMENTS && !first_finished_flag){
  
    double x = vehicle->getX();
    double y = vehicle->getY();
    double theta = vehicle->getTheta();

    if(currSeg == SEGMENTS && spline->y(currSeg, 1.0) - y < 0.35){//if on last segment and almost reached goal
      motor->mBreak();
    }else{
      motor->mSpeed(55);
    }

    double segmentProgress = (y - startY) / (spline->y(currSeg, 1.0) - startY);
    //Serial.print("segment: ");
    //Serial.println(currSeg);
    
    //Serial.print("segmentProgress: ");
    //Serial.println(segmentProgress);
    double xDesired = spline->x(currSeg, segmentProgress);

    double errorX = x - xDesired;

    double dError = (errorX - prevError)/0.021;//meters/sec

    prevError = errorX;
  
    double angleDesired = -(K_p*errorX + K_d*dError) + (theta - PI/2);//atan(-errorX / 0.20) - theta + PI/2;// switch the axes , looking 0.05 m ahead y direction

    //right positive

    //Serial.print("corrected theta: ");
    //Serial.println(theta - PI/2);
    
   
    //double correctedAngle = signum(angleDesired)*max(abs(angleDesired), STEERING_SWEEP/2-0.1);
    //Serial.print("steer angle(deg): ");
    //Serial.println(angleDesired*180.0/PI);
    front_servo->angle(angleDesired);
  
    //checking
    if( spline->y(currSeg, 1.0) - y <= 0.01){//TODO:: Calibrate this measure...
      spline->nextSegment();
      startY = y;
    }
  }else{//finished entire spline
    finished();
  }
  //vehicle->logEncoderVelocities();
  //Serial.print("Robot Vel: ");
  //Serial.println(vehicle->getRobotVel());
  
}

void checkEncoderZeroVelocities(struct encoder *enc, int tickTolerance){
  if(enc->ticks - enc->prevTicks < 4){
    enc->vel=0;
  }
  enc->prevTicks = enc->ticks;
}

void genericInterrupt(struct encoder *enc, int pinA, int pinB){

  enc->vel = 1000000 / enc->sinceCalc;
  enc->sinceCalc = 0;
  
  enc->currA = digitalReadFast(pinA);
  enc->currB = digitalReadFast(pinB);

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

int setupEncoder(struct encoder *enc, int pinA, int pinB, void (*interrupt)(void)){
  enc->ticks=0;
  enc->vel=0;
  enc->sinceCalc=0;
  
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
    motor->mSpeed(0);
    motor->mBreak();
    //Serial.println("finished spline!");   
    first_finished_flag = true;
  }  
}
