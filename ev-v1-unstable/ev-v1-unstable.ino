//system
#define LED_PIN 13
#define SWITCH_PIN 22

//motor
#define RELAY_PIN 2
#define MOTOR_PIN 3

class Motor{
  public:
    Motor(void);
    void mBreak(void);
    void mSpeed(float percent);
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

void Motor::mSpeed(float percent){
  digitalWriteFast(RELAY_PIN, 0);
  delay(5);
  int spd  =(int)((percent/100.0)*(255));
  analogWrite(MOTOR_PIN, spd);
  //Serial.println(spd);
}

Motor *motor;

//servo
#include <PWMServo.h>

PWMServo myservo;  // create servo object to control a servo
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
    PWMServo *servoobj;
};

servo::servo(void){
  this->servoobj = new PWMServo();
  this->servoobj->attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  this->servoobj->attach(SERVO_PIN, 1000, 2000); // some motors need min/max setting
}

void servo::angle(float ang){
  this->servoobj->write((int)PWM_ANGLE_TO_MEASURE(ang));
}

servo *front_servo;


//geometry
#define BACKWHEEL_TO_FRONT_DOWEL 0.265 //meters

#define ROBOT_WHEELBASE 0.2 //meters

#define ENCODER_FRONT_RESOLUTION 1440 //ticks
#define WHEEL_FRONT_DIAMETER 0.08 //meters
#define WHEEL_FRONT_CIRCUMFERENCE  PI*WHEEL_FRONT_DIAMETER //meters
#define METERS_PER_TICK_FRONT WHEEL_FRONT_CIRCUMFERENCE / ENCODER_FRONT_RESOLUTION //meters/tick

#define ENCODER_BACK_RESOLUTION 240 //ticks
#define WHEEL_BACK_DIAMETER 0.09 //meters
#define WHEEL_BACK_CIRCUMFERENCE  PI*WHEEL_BACK_DIAMETER //meters
#define METERS_PER_TICK_BACK WHEEL_BACK_CIRCUMFERENCE / ENCODER_BACK_RESOLUTION //meters/tick

//trapezoidal velocity
class trapVelProfile{
  public:
    trapVelProfile(double x1, double x2, double x3, double x4, int maxPWM, int minPWM);
    int getPWM(long int ticks);
  private:
    double _x1;
    double _x2;
    double _x3;
    double _x4;
    int _maxPWM;
    int _minPWM;
    
};

trapVelProfile::trapVelProfile(double x1, double x2, double x3, double x4, int maxPWM, int minPWM){
  this->_x1 = x1;
  this->_x2 = x2;
  this->_x3 = x3;
  this->_x4 = x4;

  this->_maxPWM = maxPWM;
  this->_minPWM = minPWM;
}

int trapVelProfile::getPWM(long int ticks){
  
  double meters = ((double)ticks) * METERS_PER_TICK_FRONT;
  
  if(meters <= this->_x1){
    return (int)((this->_maxPWM - this->_minPWM)  / this->_x1 * meters + this->_minPWM);
  }else if(meters <= this->_x2){
    return this->_maxPWM;
  }else if(meters <= this->_x3){
    return (int)((-this->_maxPWM)  / (this->_x3 - this->_x2) * meters + this->_maxPWM);
  }else if(meters <= this->_x4){
    return 0;
  }else{
    return 25;
  }
}

trapVelProfile *velProfile;

//hall sensor
#include <ADC.h>
#define HALL_PIN A10
#define HALL_VAL_MAX 65535
#define HALL_VAL_MIN 4036

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
};

void genericInterrupt(struct encoder *enc, int pinA, int pinB);

int setupEncoder(struct encoder *enc, int pinA, int pinB, void (*interrupt)(void));

void interruptFrontEnc(void);
void interruptREnc(void);
void interruptLEnc(void);

struct encoder front_enc;
struct encoder r_enc;
struct encoder l_enc;

//robot
class Robot{
  public:
    Robot(void);
    double getX(void);
    double getY(void);
    double getTheta(void);//angle to floor
    double getPhi(void);//steering angle
    long int getFrontEnc(void);//ticks
    long int getREnc(void);//ticks
    long int getLEnc(void);//ticks
    
    void recalibrateMeasures(double x, double y, double theta, double phi, double u, int long l_enc_ticks, int long r_enc_ticks);
    void updatePose(void);
    void logPose(void);
    void logEncoders(void);
    
  private:
    double _u;//total distance
    double _x;
    double _y;
    double _theta;
    double _phi;

    double _phi_offset;
    
    int long _prev_l_enc_ti;
    int long _prev_r_enc_ti;

    struct encoder *_front_enc;
    struct encoder *_r_enc;
    struct encoder *_l_enc;
};
Robot::Robot(void){

  this->_front_enc = &front_enc;
  this->_r_enc = &r_enc;
  this->_l_enc = &l_enc;

  setupEncoder(&front_enc, FRONT_ENC_PIN_A, FRONT_ENC_PIN_B, interruptFrontEnc);
  setupEncoder(&r_enc, R_ENC_PIN_A, R_ENC_PIN_B, interruptREnc);
  setupEncoder(&l_enc, L_ENC_PIN_A, L_ENC_PIN_B, interruptLEnc);
  
  recalibrateMeasures(0, 0, PI/2, 0, 0, 0, 0);
 
}

void Robot::recalibrateMeasures(double x, double y, double theta, double phi, double u, int long l_enc_ticks, int long r_enc_ticks){
  this->_x = x;
  this->_y = y;
  this->_theta = theta;
  this->_phi = phi;
  this->_phi_offset = front_hall->getPhi() - phi;
  this->_u = u;
  
  this->_prev_l_enc_ti = l_enc_ticks;
  this->_prev_r_enc_ti = r_enc_ticks;
}

double Robot::getX(void){
  return this->_x;
}
double Robot::getY(void){
  return this->_y;
}
double Robot::getTheta(void){
  return this->_theta;
}
double Robot::getPhi(void){
  this->_phi = front_hall->getPhi();
  return this->_phi;
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

void Robot::updatePose(void){
  this->_phi = front_hall->getPhi();
  
  //int dTi = (int)(front_enc.ticks - this->_sum_encoder_ticks);//assume that difference of these longs can fit in an int
  int dLenc = (int)(this->_l_enc->ticks - this->_prev_l_enc_ti);
  int dRenc = (int)(this->_r_enc->ticks - this->_prev_r_enc_ti); 
  
  double du = ((double)(dLenc + dRenc))/2.0 * METERS_PER_TICK_BACK;
  this->_u += du;

  double dTheta = du / ROBOT_WHEELBASE * tan(this->_phi);
  this->_theta += dTheta;  //TODO: change to NavX data
  
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

Robot *vehicle;

#define TRAP_END_POINT 1.5
#define MOTOR_TOP_SPEED 5.5
#define DECEL_DISTANCE 0.3
#define COASTING_PERIOD 0
#define TARGET_POINT 10 //78cm l 17cm b


//

void setup() {
  Serial.begin(9600);
  //while(!Serial);
  pinMode(LED_PIN, OUTPUT);
  
  front_hall = new Hall();
  
  vehicle = new Robot();

  motor = new Motor();

  front_servo = new servo();

  velProfile = new trapVelProfile(TRAP_END_POINT, MOTOR_TOP_SPEED + TRAP_END_POINT, MOTOR_TOP_SPEED + TRAP_END_POINT + DECEL_DISTANCE, MOTOR_TOP_SPEED + TRAP_END_POINT + DECEL_DISTANCE + COASTING_PERIOD, 50, 50);

  pinMode(SWITCH_PIN, INPUT); 
  Serial.println("Waiting...");
  while(!digitalRead(SWITCH_PIN)){
    delay(50);
  }
  delay(1000);
  Serial.println("Started");
  vehicle->recalibrateMeasures(0, 0, PI/2, 0, 0, 0, 0);
}



void loop() {
  
  delay(1);
  digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
  
  

  if(vehicle->getY() <= TRAP_END_POINT + MOTOR_TOP_SPEED + DECEL_DISTANCE + COASTING_PERIOD){
    //motor->mSpeed(velProfile->getPWM(front_enc.ticks)/ 255.0 * 100.0);
  }else{
    motor->mBreak();
    digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
    delay(1500);
    while(vehicle->getY() < TARGET_POINT){
       //motor->mSpeed(50.0 / 255.0 * 100.0);
       vehicle->updatePose();
       //vehicle->logPose();
       vehicle->logEncoders();
       delay(1);
    }
    motor->mBreak();
    while(true){
      delay(500);
      vehicle->updatePose();
      //vehicle->logPose();
      vehicle->logEncoders();
      digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
      //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    }
  }
  
  //front_servo->angle(PI/6);
  
  

  vehicle->updatePose();
  //vehicle->logPose();
  vehicle->logEncoders();
    
}

void genericInterrupt(struct encoder *enc, int pinA, int pinB){
  
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
