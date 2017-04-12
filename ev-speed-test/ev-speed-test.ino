//Goes around the cans using splines, NavX MXP, encoders, and servomotor/hall effect (worked at states)

#define LAST_COASTING_DISTANCE 0.005
#define TARGET_DISTANCE 10

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
#define LOOPS_PER_SEC 200
#define SLOWER_LOOPS_PER_SEC 5
int loopCount = 0;

//velocity control

#define END_TARGET_MAX_VELOCITY 0.8  //meters/sec
#define END_TARGET_MIN_VELOCITY 0.3 //meters/sec
#define TARGET_SLOW_DOWN_DISTANCE 0.5 //meters, the distance where vehicle slows down from END_TARGET_MAX_VELOCITY to END_TARGET_MIN_VELOCITY
#define VELOCITY_TOLERANCE 0.5  //meters/sec

#define END_TARGET_VELOCITY 0.5

#define K_P_VEL 120
#define CONST_SPEED 40

//for recalibrating the measures
#define NO_RECALIBRATION -42

//Splines
#define SPLINE_DISTANCE_TOLERANCE 0.05//5cm of point to point tolerance for splines

#define SEGMENTS 2 //p1 -> p2 -> p3 defined to be (number of points - 1)
#define POINTS 3

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
  this->servoobj->attach(SERVO_PIN);  // attaches the servo to the servo object
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
#define ROBOT_LENGTH 0.295


#define ENCODER_FRONT_RESOLUTION 1440 //ticks
#define WHEEL_FRONT_DIAMETER 0.08 //meters
#define WHEEL_FRONT_CIRCUMFERENCE  PI*WHEEL_FRONT_DIAMETER //meters
#define METERS_PER_TICK_FRONT WHEEL_FRONT_CIRCUMFERENCE / ENCODER_FRONT_RESOLUTION //meters/tick

#define ENCODER_BACK_RESOLUTION 240 //ticks
#define WHEEL_BACK_DIAMETER 0.09 //meters
#define WHEEL_BACK_CIRCUMFERENCE  PI*WHEEL_BACK_DIAMETER //meters
#define METERS_PER_TICK_BACK 0.001152  //WHEEL_BACK_CIRCUMFERENCE / ENCODER_BACK_RESOLUTION //meters/tick

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
  Serial.print(this->getY() + ROBOT_LENGTH * sin(this->getTheta()));
  Serial.print(", theta (deg)= ");
  Serial.print(this->getTheta()*180/PI);
  Serial.print(", phi (deg)= ");
  Serial.print(this->getPhi()*180/PI);
  Serial.print(", vel (m/s)= ");
  Serial.print(this->getRobotVel());
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
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  
  delay(10000);

  //front_hall = new Hall();
  //motor = new Motor();
  //front_servo = new servo();
  
  vehicle = new Robot();
 /* 
  //test if it's really 200 Hz
  vehicle->pollNavX();
  float starting_theta = vehicle->getTheta();

  //wait, polling at 200 Hz, until theta changes, to ensure we start at the beginning of a NavX cycle
  do{
    loopTime = 0;
    vehicle->pollNavX();
    while(loopTime < 1e6/200);
  }while(vehicle->getTheta() == starting_theta);
  
  delayMicroseconds(1e6/200 * 3); //wait out the rest of that cycle
  
  //prints out values at 200 hz so we can see if the update rate is right
  for(int loopCounter = 0; loopCounter < 50; loopCounter++){
    for(int loopCounter2 = 0; loopCounter2 < 4; loopCounter2++){
      loopTime = 0;
      vehicle->pollNavX();
      Serial.print(vehicle->getTheta()/PI*180);
      Serial.print(" ");
      while(loopTime < 1e6/200);
    }
    Serial.println(" ");
  }
  */
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
    Serial.println("Waiting for User Button...");
    vehicle->pollNavX();
    /*
    Serial.print("Angle: ");
    Serial.println(vehicle->getTheta()*180.0/PI);
    */
    delay(100);
  }
  //wait for button to be released
  while(digitalReadFast(SWITCH_PIN)){delay(50);} 
  delay(500);

  vehicle->makeUpdateRate200Hz();
  
  digitalWriteFast(LED_PIN, HIGH);
  vehicle->pollNavX();
  vehicle->recalibrateMeasures(0, 0, PI/2 + LASER_OFFSET_THETA, 0, 0, 0, 0);
  
  while(!digitalReadFast(SWITCH_PIN)){delay(50);} //wait for button to be pressed to signal run start
  delay(500);
  vehicle->pollNavX();
  vehicle->recalibrateMeasures(0, 0, NO_RECALIBRATION, 0, 0, 0, 0);
  /*
  Serial.print("Starting angle (degrees): ");
  Serial.println(vehicle->getTheta()*180.0/PI)
  */
  
  //spline data format is x0 , x1, ..., y0, y1, ..., v_x0, v_x1, ..., v_y0, v_y1, ...
  /* format:
  float spline_data[POINTS*4] = {x_point0, x_point1, ...
                                 y_point0, y_point1, ...
                                 velocity_x_point0, ...
                                 velocity_y_point0, ...};*/

  float spline_data[POINTS*4] = {0.0, 0.0, 0.0,
                                 0.0, TARGET_DISTANCE/2.0, TARGET_DISTANCE,
                                 0.01*sin(vehicle->getTheta()), 0.0, -0.01*sin(vehicle->getTheta()),
                                 0.01*cos(vehicle->getTheta()), 2.0, 0.01*cos(vehicle->getTheta())};
  //e.g.  d(4, 1) means d_y at the 4th segment
  //use of small velocities at beginning and end is to get the spline pointing in the right direction
  
  spline = new Splines();
  spline->loadSplineData(spline_data);
  
}

//using y as the internal parameter
double startY = 0;

double prevError=0;
double dError = 0;

//for spline follower
#define  K_P_SPLINE_FOLLOW  2.8
#define K_D_SPLINE_FOLLOW 1.0
#define dERROR_FILTER 0.5

#define MAX_MOTOR_ACCEL 150  //PWM 0-256
#define ROBOT_TEST_VELOCITY 1.0
int motorSpeed = MAX_MOTOR_ACCEL;

bool started_breaking = false;

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

  //splines 
  int currSeg = spline->getCurrentSegment();


  
  if(currSeg <= SEGMENTS && !first_finished_flag){
  
    double theta = vehicle->getTheta();
    
    double x = vehicle->getX() + ROBOT_LENGTH * cos(theta);
    double y = vehicle->getY() + ROBOT_LENGTH * sin(theta);
    
    //velocity controller
    
    double segmentProgress = (y - startY) / (spline->y(currSeg, 1.0) - startY);
    //Serial.print("segment: ");
    //Serial.println(currSeg);
    
    //Serial.print("segmentProgress: ");
    //Serial.println(segmentProgress);
    double xDesired = spline->x(currSeg, segmentProgress);

    double errorX = x - xDesired;

    dError += dERROR_FILTER * ((errorX - prevError)*LOOPS_PER_SEC - dError);// derivative of error, meters/sec

    prevError = errorX;
  
    double angleDesired = -(K_P_SPLINE_FOLLOW*errorX/(vehicle->getRobotVel() + 0.2) + K_D_SPLINE_FOLLOW*dError)/(vehicle->getRobotVel() + 0.2);//atan(-errorX / 0.20) + theta - PI/2;
    //TODO:: INVESTIGATE
    if(started_breaking || vehicle->getRobotVel() > ROBOT_TEST_VELOCITY){
      if(!started_breaking){
        Serial.println("Brakes, ");
        started_breaking = true;
      }
      motor->mBreak();
      if (vehicle->getRobotVel() > 0){
        vehicle->logPose(errorX);
      }
    }else{
      motor->mSpeed(MAX_MOTOR_ACCEL);
      vehicle->logPose(errorX);
    }

    //right positive

    //Serial.print("corrected theta: ");
    //Serial.println(theta - PI/2);
    
   
    //double correctedAngle = signum(angleDesired)*max(abs(angleDesired), STEERING_SWEEP/2-0.1);
    //Serial.print("steer angle(deg): ");
    //Serial.println(angleDesired*180.0/PI);
    front_servo->angle(angleDesired);
  
    //checking for finishing spline
    if( spline->y(currSeg, 1.0) - y <= 0.01){//TODO:: Calibrate this measure...
      spline->nextSegment();
      startY = y;
    }
  }else{//finished entire spline
    finished();
    vehicle->logPose(vehicle->getX() + ROBOT_LENGTH * cos(vehicle->getTheta()));

  }
  //vehicle->logEncoderVelocities();
  //Serial.print("Robot Vel: ");
  //Serial.println(vehicle->getRobotVel());

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
    //Serial.println("finished");   
    first_finished_flag = true;
  }  
}
