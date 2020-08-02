//#include <NewPing.h>

#define TRIGGER_PIN_L 8
#define ECHO_PIN_L    9  
#define TRIGGER_PIN_R  6
#define ECHO_PIN_R     7  
#define MAX_DISTANCE 200 


//NewPing sonar_L(TRIGGER_PIN_L, ECHO_PIN_L, MAX_DISTANCE);
//NewPing sonar_R(TRIGGER_PIN_R, ECHO_PIN_R, MAX_DISTANCE);

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <Servo.h>
#include "ros.h"
#include "ros/time.h"
//header file for publishing velocities for odom
#include "lino_msgs/Velocities.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
#include "lino_msgs/PID.h"
#include "lino_msgs/Imu.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"

#include "lino_base_config.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"
#include "Imu.h"

#define ENCODER_OPTIMIZE_INTERRUPTS 
#include "Encoder.h"

#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV); 
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV); 
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV); 

Servo steering_servo;


Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 
Controller motor3_controller(Controller::MOTOR_DRIVER, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Controller motor4_controller(Controller::MOTOR_DRIVER, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;


unsigned long g_prev_command_time = 0;

//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const lino_msgs::PID& pid);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);

lino_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

sensor_msgs::Range range_msg_L;
sensor_msgs::Range range_msg_R;
ros::Publisher pub_range1( "/ultrasound_R", &range_msg_R);
ros::Publisher pub_range2( "/ultrasound_L", &range_msg_L);

long range_time;
char frameid[] = "/ultrasound";
float leftdistance;
float rightdistance;


void setup()
{
    steering_servo.attach(STEERING_PIN);
    steering_servo.write(90);
    
    nh.initNode();
    nh.getHardware()->setBaud(57600);
   // Serial2.begin(57600);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
    
    nh.advertise(pub_range1);
    nh.advertise(pub_range2);
    //tambahan buat baca ultrasonic
    range_msg_L.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg_L.header.frame_id = frameid;
    range_msg_L.field_of_view = 0.1; // fake
    range_msg_L.min_range = 0.0;
    range_msg_L.max_range = 6.47;

    range_msg_R.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg_R.header.frame_id = frameid;
    range_msg_R.field_of_view = 0.1; // fake
    range_msg_R.min_range = 0.0;
    range_msg_R.max_range = 6.47;
    

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("LINOBASE CONNECTED");
    delay(1);
}

void loop()
{
    static unsigned long prev_control_time = 0;
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_debug_time = 0;
    static bool imu_is_initialized;

      range_msg_L.range = getleftRange();
      leftdistance = range_msg_L.range;
      range_msg_L.header.stamp = nh.now();
      pub_range2.publish(&range_msg_L);
              
      range_msg_R.range = getrightRange();
      rightdistance = range_msg_R.range;
      range_msg_R.header.stamp = nh.now();
      pub_range1.publish(&range_msg_R);
              

    //this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        moveBase();
        prev_control_time = millis();
    }

    //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
    }
    if( (leftdistance <= 15.0) || (rightdistance <= 15.0) )
    {
        stopBase();
        nh.loginfo("Safety sequence: Stopping velocity command");
    }

    //this block publishes the IMU data based on defined rate
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        //sanity check if the IMU is connected
        if (!imu_is_initialized)
        {
            imu_is_initialized = initIMU();

            if(imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            publishIMU();
        }
        prev_imu_time = millis();
    }
    
    
    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            printDebug();
            prev_debug_time = millis();
        }
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
}


void PIDCallback(const lino_msgs::PID& pid)
{
    //callback function every time PID constants are received from lino_pid for tuning
    //this callback receives pid object where P,I, and D constants are stored
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
    motor4_pid.updateConstants(pid.p, pid.i, pid.d);
}


void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    if( (leftdistance <= 15.0) || (rightdistance <= 15.0) )
    {
      g_req_linear_vel_x = 0;
      g_req_linear_vel_y = 0;
      g_req_angular_vel_z = 0;
    }
    else
    {
      g_req_linear_vel_x = cmd_msg.linear.x;
      g_req_linear_vel_y = cmd_msg.linear.y;
      g_req_angular_vel_z = cmd_msg.angular.z;
    }
    
    g_prev_command_time = millis();
    
}


void moveBase()
{
    //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    //get the current speed of each motor
    int current_rpm1 = motor1_encoder.getRPM();
    int current_rpm2 = motor2_encoder.getRPM();
    int current_rpm3 = motor3_encoder.getRPM();
    int current_rpm4 = motor4_encoder.getRPM();

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));  
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));    

    Kinematics::velocities current_vel;

    if(kinematics.base_platform == Kinematics::ACKERMANN || kinematics.base_platform == Kinematics::ACKERMANN1)
    {
        float current_steering_angle;
        
        current_steering_angle = steer(g_req_angular_vel_z);
        current_vel = kinematics.getVelocities(current_steering_angle, current_rpm1, current_rpm2);
    }
    else
    {
        current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    }
    
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void safetyBase()
{
    raw_vel_msg.linear_x = 0;
    raw_vel_msg.linear_y = 0;
    raw_vel_msg.angular_z = 0;
    raw_vel_pub.publish(&raw_vel_msg);
}
void publishIMU()
{
    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readAccelerometer();
    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readGyroscope();
    //pass accelerometer data to imu object
    raw_imu_msg.magnetic_field = readMagnetometer();
    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
}



float steer(float steering_angle)
{
    //steering function for ACKERMANN base
    float servo_steering_angle;

    steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    servo_steering_angle = mapFloat(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE, PI, 0) * (180 / PI);

    steering_servo.write(servo_steering_angle);

    return steering_angle;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void printDebug()
{
    char buffer[50];
    sprintf (buffer, "Encoder RearLeft : %ld", motor1_encoder.read());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder RearRight : %ld", motor2_encoder.read());
    nh.loginfo(buffer);
}

long microsecondsToCentimeters(long microseconds)
{
    // The speed of sound is 340 m/s or 29 microseconds per centimeter.
    // The ping travels out and back, so to find the distance of the
    // object we take half of the distance travelled.
    return microseconds / 29.1 / 2;
}

float getleftRange()
{
    // establish variables for duration of the ping,
    // and the distance result in inches and centimeters:
    long lduration;
    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    pinMode(TRIGGER_PIN_L, OUTPUT);
    digitalWrite(TRIGGER_PIN_L, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN_L, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN_L, LOW);
    // The same pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(ECHO_PIN_L, INPUT);
    lduration = pulseIn(ECHO_PIN_L, HIGH);
    // convert the time into a distance
    return microsecondsToCentimeters(lduration);
}


float getrightRange()
{
    // establish variables for duration of the ping,
    // and the distance result in inches and centimeters:
    long rduration;
    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    pinMode(TRIGGER_PIN_R, OUTPUT);
    digitalWrite(TRIGGER_PIN_R, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN_R, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN_R, LOW);
    // The same pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(ECHO_PIN_R , INPUT);
    rduration = pulseIn(ECHO_PIN_R, HIGH);
    // convert the time into a distance
    return microsecondsToCentimeters(rduration);
}
