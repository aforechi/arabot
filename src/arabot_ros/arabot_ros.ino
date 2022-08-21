#include <AFMotor.h>
#include <Servo.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include <geometry_msgs/Twist.h>
 
 
//baseline é o tamanho do eixo
const float BASELINE = 0.134;
const float CONTROL_GAIN = 1.0;
const float CONTROL_TRIM = 0.0;
const int MIN_SPEED_PWM = 80;
const int MAX_SPEED_PWM = 255;
const float SPEED_TOLERANCE = 0.01;
const uint8_t NUM_SERVOS = 4;

Servo servos[NUM_SERVOS];
uint8_t angles[NUM_SERVOS];
const uint8_t SERVO_PINS[NUM_SERVOS] = {
    3, //forward-backward
    4, //up-down
    9, //rotate
    10, //grab
};

AF_DCMotor leftMotor(3);
AF_DCMotor rightMotor(4);

float leftMotorSpeed=0.0;
float rightMotorSpeed=0.0;
 
ros::NodeHandle nh;

//char message[255];
//std_msgs::String status_message;

void velocity_message_callback( const geometry_msgs::Twist& msg){
  float angularVelocity = msg.angular.z;
  float linearVelocity = msg.linear.x;
  leftMotorSpeed = (CONTROL_GAIN - CONTROL_TRIM) * (linearVelocity + 0.5 * angularVelocity * BASELINE);
  rightMotorSpeed = (CONTROL_GAIN + CONTROL_TRIM) * (linearVelocity - 0.5 * angularVelocity * BASELINE);
  //Inverte sentido das rodas caso esteja andando na direção errada
  leftMotorSpeed *= -1.0;
  rightMotorSpeed *= -1.0;
}

void angle_message_callback(const std_msgs::Int8MultiArray& msg){
    //msg.data.size();
    for(int i=0;i<NUM_SERVOS;i++){
        angles[i] = msg.data[i];
    }
}

//ros::Publisher status_publisher("status", &status_message);
ros::Subscriber<geometry_msgs::Twist> velocity_subscriber("cmd_vel", &velocity_message_callback );
ros::Subscriber<std_msgs::Int8MultiArray> angle_subscriber("arabot_arm", &angle_message_callback);

void setup(){
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);

  for(int i; i < NUM_SERVOS; i++)
  {
    servos[i].attach(SERVO_PINS[i]);
    angles[i] = 0;
  }
 
  nh.initNode();
  nh.subscribe(angle_subscriber);
  nh.subscribe(velocity_subscriber);
  //nh.advertise(status_publisher);
}
 
int fmap(float v, int minPWM, int maxPWM){
  int pwm = 0;
  if (fabs(v) > SPEED_TOLERANCE)
      pwm = (int)(floor(fabs(v) * (maxPWM - minPWM) + minPWM));
  return pwm;  
}

void loop(){
  if (leftMotorSpeed > SPEED_TOLERANCE) 
    leftMotor.run(FORWARD);
  else if (leftMotorSpeed < -SPEED_TOLERANCE)
    leftMotor.run(BACKWARD);
  else
    leftMotor.run(RELEASE);

  if (rightMotorSpeed > SPEED_TOLERANCE) 
    rightMotor.run(FORWARD);
  else if (rightMotorSpeed < -SPEED_TOLERANCE)
    rightMotor.run(BACKWARD);
  else
    rightMotor.run(RELEASE);
  
  int leftMotorPWM = fmap(leftMotorSpeed, MIN_SPEED_PWM, MAX_SPEED_PWM);
  int rightMotorPWM = fmap(rightMotorSpeed, MIN_SPEED_PWM, MAX_SPEED_PWM);
  /*
  sprintf(message, "%d %d", leftMotorPWM, rightMotorPWM);
  status_message.data = message;
  status_publisher.publish( &status_message );
  */
  leftMotor.setSpeed(leftMotorPWM); 
  rightMotor.setSpeed(rightMotorPWM); 

  for(int i=0; i < NUM_SERVOS; i++)
    if (angles[i] >= 0 && angles[i] <= 179)
      servos[i].write(angles[i]);
    
  nh.spinOnce();
  delay(50);
}
