#include <MPU6050_light.h>
#include <ArduinoQueue.h>
#include <NewPing.h>
#include <Servo.h>

////////////////////////////////////////////////////////////////////////////// Here is variables definition and functions declaration //////////////////////////////////////////////////////////////////////////////

#define TRIGGER_PIN_L1  34
#define ECHO_PIN_L1  35

#define TRIGGER_PIN_L2  38
#define ECHO_PIN_L2  39

#define TRIGGER_PIN_R1  23
#define ECHO_PIN_R1  22

#define TRIGGER_PIN_R2  44
#define ECHO_PIN_R2  45

#define TRIGGER_PIN_F 42
#define ECHO_PIN_F  43

#define D2R_trigger 52
#define D2R_echo 53

#define D2L_trigger 24
#define D2L_echo 25

#define B_trigger 13
#define B_echo 12

#define BL_trigger 3
#define BL_echo 2

#define BR_trigger 27
#define BR_echo 26

#define MAX_DISTANCE 300

#define R_EN 10
#define L_EN 8
#define R_PWM 11
#define L_PWM 9

#define Switch 50
MPU6050 mpu(Wire);



bool last_turn_to_R = false;
bool last_turn_to_L = false;
bool start_G =false;
bool Ana_Last = false;
int prev_section  = 2;
const double pi = acos(-1);
int pos_R = 90, pos_L = 85, pos_L_Back = 95 , pos_follow_R = 90, turn_pos = 45, turn_pos2 = 45;// TestV3
int Delay_Turn = 550;
int counter = 0;
int dis_narrow_F = 50;
int speed = 60, speed_turn = 60;
double Position_L = pos_R - turn_pos , Position_R = pos_R + turn_pos2;
double position_angle = 0;
double distance = 0, real_distance = 0 , Real_dis_L = 0, Real_dis_R = 0, dis_L1 = 0, dis_L2 = 0, dis_R1 = 0, dis_R2 = 0, dis_F = 0, dis_D2L = 0 , dis_D2R = 0 , dis_B = 0, dis_BL = 0 , dis_BR = 0  ;
double angle = 0, Angle = 0, Aangle = 0;
double conskp_R = 2.7, kp_right = 1;
double conskp = 2.2, kp_left = 1;
double Setpoint = 15;
double highest_angle = 0.27;
double pos_distance;
double jbes = 0;
double duration_F = 0, duration_L1 = 0, duration_L2 = 0, duration_R1 = 0, duration_R2 = 0 , duration_D2L = 0, duration_D2R = 0 , duration_B = 0, duration_BL = 0, duration_BR = 0;
double theta = 0, theta_mpu = 0, ref_mpu = 0; //radian
double d_pillar = 70;
double min_angle = -37 - max(pos_L, pos_R) , max_angle = 37 + max(pos_L, pos_R);
double Setpoint_Red_R = 10, Setpoint_Red_R_2 = 15;
double Setpoint_Green_L = 10, Setpoint_Green_L_2 = 15;
double counter_angle_Right = 0, counter_angle_Left = 0, counter_1 = 0;///////////123
int detected_first =  0;
char detected_cube = 'N';
char detected_P, detected_ser = 'N';
double aAngle = 0, y = 0, desired_angle = 0, Desired_angle = 0, start_angle = 0;
////////////////PID__C
double currentTime_angle = 0, dtc = 0 , lastTime_angle = 0 , current_angle = 0 , error_c = 0, output_c = 0  , last_angle_error = 0;
double kpc = 1, kdc = 1;
double reference = 0;
double reference_back = 0;
double t_make_straight;
double endReTurn = 0;
String data = "N",ser = "N" ;
int start = 0;
int i = 0;/// $£
double t_ = 0;//$£
double correction_it = 0;
double error = 0;
Servo myservo;
NewPing sonar_L1(TRIGGER_PIN_L1, ECHO_PIN_L1, MAX_DISTANCE);
NewPing sonar_L2(TRIGGER_PIN_L2, ECHO_PIN_L2, MAX_DISTANCE);
NewPing sonar_R1(TRIGGER_PIN_R1, ECHO_PIN_R1, MAX_DISTANCE);
NewPing sonar_R2(TRIGGER_PIN_R2, ECHO_PIN_R2, MAX_DISTANCE);
NewPing sonar_F(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);
NewPing sonar_D2R(D2R_trigger, D2R_echo, MAX_DISTANCE);
NewPing sonar_D2L(D2L_trigger, D2L_echo, MAX_DISTANCE);
NewPing sonar_B(B_trigger, B_echo, MAX_DISTANCE);
NewPing sonar_BL(BL_trigger, BL_echo, MAX_DISTANCE);
NewPing sonar_BR(BR_trigger, BR_echo, MAX_DISTANCE);

int Filter_Size = 5;
double mean_bl = 0;
ArduinoQueue <double> Means_BL;
double mean_fl = 0;
ArduinoQueue <double> Means_FL;
double mean_br = 0;
ArduinoQueue <double> Means_BR;
double mean_fr = 0;
ArduinoQueue <double> Means_FR;
double mean_f = 0;
ArduinoQueue <double> Means_F;
double mean_b = 0;
ArduinoQueue<double> Means_B;

double f , fl, bl , fr , br, b;

void avoid_G(double dis = 15);
void avoid_R(double dis = 17);

void avoid_RG(double dis = 15);
void avoid_GR(double dis = 19);

////////////////////////////////////////////////////////////////////////////// End of variables definition and functions declaration //////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////// Motors controlling functions are here //////////////////////////////////////////////////////////////////////////////

void Run_Sumo (int pwm)// Move forward with "pwm" speed.
{
  digitalWrite (R_EN, HIGH);
  digitalWrite (L_EN, HIGH);
  analogWrite (R_PWM, pwm);
  analogWrite (L_PWM, 0);
}

void Stop_Sumo ()// Move backward with "pwm" speed.
{
  digitalWrite (R_EN, HIGH);
  digitalWrite (L_EN, HIGH);
  analogWrite (R_PWM, 0);
  analogWrite (L_PWM, 0);

}

void Back_Sumo (int pwm=0)//Stop the motor
{
  digitalWrite (R_EN, HIGH);
  digitalWrite (L_EN, HIGH);
  analogWrite (R_PWM, 0);
  analogWrite (L_PWM, pwm);

}

void servo_write(double ang) {// Move the servo motor "ang" degrees, set upper and lower limit for the angle.
  if (ang > max_angle) {
    ang = max_angle;
  }
  else if (angle < min_angle) {
    ang = min_angle;
  }
  myservo.write(ang);
}

////////////////////////////////////////////////////////////////////////////// End of motors controlling functions //////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////// Sensors reading functions are here //////////////////////////////////////////////////////////////////////////////
//- "Filter": This function the current reading, readings queue and last reading as input, it take the average of the last 5 readings and reutrn it as the sensor reading.

double Filter(double read, double &last_mean, ArduinoQueue <double> &Means)
{
  if (Means.isEmpty())
  {
    last_mean = read;
    for (int i = 0; i < Filter_Size; i++)
    {
      Means.enqueue(read);
    }
  }
  else
  {
    double temp = Means.dequeue();
    last_mean -= (double(temp / Filter_Size));
    last_mean += (double(read / Filter_Size));
    Means.enqueue(read);
  }
  return last_mean;
}
////////////////////////////////////////////////////////////////////////////// End of distance and angle calculation functions //////////////////////////////////////////////////////////////////////////////

double get_distance_F ()
{
  duration_F = sonar_F.ping();
  f = (duration_F / 2) * 0.0343;// Calculate the distance based on utlra sonic wave speed and time taken by the wave.
  return f;
}
double get_distance_B() {
  duration_B = sonar_B.ping();
  b = ((duration_B / 2) * 0.0343);
  return b;
}

double get_distance_BL ()
{
  duration_BL = sonar_BL.ping();
  return ((duration_BL / 2) * 0.0343);
}
double get_distance_BR ()
{
  duration_BR = sonar_BR.ping();
  return ((duration_BR / 2) * 0.0343);
}

double get_distance_L1 ()
{
  duration_L1 = sonar_L1.ping();
  fl = ((duration_L1 / 2) * 0.0343) - 4 ;
  mean_fl = Filter(fl, mean_fl, Means_FL);// Take the filtered value
  return mean_fl ;
}

double get_distance_L2()
{
  duration_L2 = sonar_L2.ping();
  bl = ((duration_L2 / 2) * 0.0343) - 4 ;
  mean_bl = Filter(bl, mean_bl, Means_BL);
  return mean_bl;
}

double get_distance_R1 ()
{
  duration_R1 = sonar_R1.ping();
  fr = ((duration_R1 / 2) * 0.0343) - 4;
  mean_fr = Filter(fr, mean_fr, Means_FR);
  return mean_fr;
}

double get_distance_R2()
{
  duration_R2 = sonar_R2.ping();
  br = ((duration_R2 / 2) * 0.0343) - 4;
  mean_br = Filter(br, mean_br, Means_BR);
  return mean_br;
}


double get_distance_D2R ()
{
  duration_D2R = sonar_D2R.ping();
  if (((duration_D2R / 2) * 0.0343) == 0)
    return 100;
  return ((duration_D2R / 2) * 0.0343);

}

double get_distance_D2L()
{
  duration_D2L = sonar_D2L.ping();
  if (((duration_D2L / 2) * 0.0343) == 0)
    return 100;
  return ((duration_D2L / 2) * 0.0343) - 3 ;
}

////////////////////////////////////////////////////////////////////////////// End of sensors reading functions //////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////// Distance and angle calculation functions are here //////////////////////////////////////////////////////////////////////////////
/*
The provided functions below are responsible for all mathematical calculations (angle, prependicular distance,filtering):
- "Distance": Takes distance sensors readings as input, and returns the prependicular distance.
- "angle_cal": Takes sensor readings as input, and returns the angle between the robot and the wall.
- "correction_L"/"correction_R": Those functions are responsible for correcting the MPU6050 input reference, they work by taking the average of 5 angle calculations and substracting the value from the calculated reference.
- "reset_means": Resets the sensors readings queues, this is necessary when moving from corner section to straight forward section (Values may some time to be updated due to the nature of filtering, thus, resetting them is beneficial).
*/

  double angle_cal (double dis_1 , double dis_2)
{
  Angle =  atan ((dis_1 - dis_2 ) / 7.5);
  if (Angle > highest_angle)
  {
    Angle =  highest_angle;
  }
  if (Angle < ((-1)*highest_angle))
  {
    Angle = (-1) * highest_angle;
  }
  return Angle;
  //Radian
}
double Distance(double L1 , double L2)
{
  distance = (L1 + L2) / 2;
  angle = angle_cal (L1, L2);
  real_distance = distance * cos(angle);
  return real_distance;
}
void correction_L(){
   if(correction_it == 4){
      reference -= error/5;
      error = 0;
    }
    else {
      error += angle_cal(dis_L1,dis_L2)*180/acos(-1);
      correction_it++;
    }
}
void correction_R(){
   if(correction_it == 4){
      reference -= error/5;
      error = 0;
    }
    else {
      error += angle_cal(dis_R1,dis_R2)*180/acos(-1);
      correction_it++;
    }
}
void reset_means()
{
 while (!Means_B.isEmpty())
 {
   Means_B.dequeue();
   Means_BL.dequeue();
   Means_BR.dequeue();
   Means_FL.dequeue();
   Means_FR.dequeue();
   Means_F.dequeue();

 }
}


double angleR = 0, angleL = 0;
void sensor() {
  dis_R1 = get_distance_R1();
  delay(10);
  dis_L2 = get_distance_L2();
  delay(10);
  dis_B = get_distance_B();
  delay(10);
  dis_L1 = get_distance_L1();
  delay(15);
  dis_R2 = get_distance_R2();
  delay(15);
  dis_F = get_distance_F();
  delay(10);
  Real_dis_L = Distance(dis_L1, dis_L2);
  Real_dis_R = Distance(dis_R1, dis_R2);

  angleR = angle_cal(dis_R1, dis_R2) * 180 / pi;
  angleL = angle_cal(dis_L1, dis_L2) * 180 / pi;

}
////////////////////////////////////////////////////////////////////////////// Motion in protected area and unprotected area functions are here //////////////////////////////////////////////////////////////////////////////

/*
 The functions that drive the motion in the protected and unprotected areas are presented here:
- ("angle_Left","angle_Left_Back","angle_Right","angle_FollowR"): These functions are repsonsible for PID following with ki and kd constants equal to zero, they use the angle calculated using sensors readings. These functions differ
in the setpoint and used sensors.
- ("P_distanceL","P_distanceR"): These functions work by the same manner, but with a higher kp constant.
- "PID_angle"/"PID_angle_Back": hese functions are repsonsible for PID following with ki constant equal to zero, they use the input angle from the gyroscope sensor. 
*/

double angle_Left(double L1 , double L2) {


  angle =  angle_cal (L1, L2);

  Angle = (angle * 180) / 3.14;

  position_angle = pos_L - Angle;

  myservo.write(position_angle);

}
double angle_Left_Back(double L1 , double L2) {


  angle =  angle_cal (L1, L2);

  Angle = (angle * 180) / 3.14;

  position_angle = pos_L_Back - Angle;

  myservo.write(position_angle);

}

double angle_Right(double R1 , double R2) {

  angle =  angle_cal (R1, R2);

  Angle = (angle * 180) / 3.14;

  position_angle = pos_R + Angle;

  myservo.write(position_angle);

}
double angle_FollowR(double R1 , double R2) {


  angle =  angle_cal (R1, R2);

  Angle = (angle * 180) / 3.14;

  position_angle = pos_follow_R + Angle;

  myservo.write(position_angle);

}

double P_distanceL (double real_distance , double Set_Point)
{
  pos_distance = conskp * (real_distance - Set_Point);
  jbes = pos_L - pos_distance;
  servo_write(jbes);
}
double P_distanceR (double real_distance , double Set_Point)
{
  pos_distance = conskp_R * (real_distance - Set_Point);
  jbes = pos_R + pos_distance;
  servo_write(jbes);
}

void Back_PID_angle(double current_angle) {
  currentTime_angle = (micros()) * (1e-3);
  dtc =  currentTime_angle - lastTime_angle;
  lastTime_angle = currentTime_angle;
  error_c = current_angle - reference_back;
  output_c = kpc * error_c + ((error_c - last_angle_error) / dtc) * kdc ;
  servo_write( pos_R - output_c);
  last_angle_error = error_c;
}
void PID_angle(double current_angle) {
  currentTime_angle = (micros()) * (1e-3);
  dtc =  currentTime_angle - lastTime_angle;
  lastTime_angle = currentTime_angle;
  error_c = current_angle - reference;
  output_c = kpc * error_c + ((error_c - last_angle_error) / dtc) * kdc ;
  servo_write(output_c + pos_R);
  last_angle_error = error_c;
}

////////////////////////////////////////////////////////////////////////////// End of motion in protected area and unprotected area functions //////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////// Following functions are described and explained here //////////////////////////////////////////////////////////////////////////////

/*
Following functions are explained below, each function works in the same manner. we will refer to the unprotected area by "UPA" and protected area by "PA":
1- If the prependicular distance is small (<5cm), this means that the robot is in the UPA. The program calls "P_distanceR"/"P_distanceL" functions depending on which side is the robot. Those functions are responsible for 
keeping the robot away from UPA by using P algorithm with relatively big P constant.
2- If the previous condition is not true for the two sides, this means that the robot is in PA. "PID_angle" function is called in "followmiddle_straight" function, which applies PD algorithm on robot's angle to keep it parallel to the wall.
On the other hand, the rest of following functions call ("angle_Left","angle_Right","angle_FollowR"), this difference comes from the fact that "followmiddle_straight" function is called in the first turn, which means that
the direction of movement is not identified yet, thus applying PD algorithm on one of the algorithms ("angle_Left","angle_Right","angle_FollowR") may result in unstable movement (In case of reading distances from a long distance, the measurments won't be precise).

*/
void followmiddle_straight()
{
   if (Real_dis_R < 5)
  {
    P_distanceR(Real_dis_R, 5);
  }
  else if (Real_dis_R > 90)
  {
    P_distanceR(Real_dis_R, 90);
  }
  else
  {
      if(counter_angle_Right == 1)correction_R();
      if(counter_angle_Left == 1)correction_L();
      mpu.update();
      PID_angle(mpu.getAngleZ());
 }
}
void follow_G_Counter() {
  Ana_Khales();
  while (true) {

    sensor();
 
    if (counter_angle_Right == 1 && fl >= 50)break;
    if(((counter_angle_Left == 0 && counter_angle_Right == 0) || counter_angle_Left == 1)  && (fr > 100 || fl > 100))break;
    if (Real_dis_L < 13)
    {
      Run_Sumo(speed);
      P_distanceL(Real_dis_L, 13);
    }
    else if (Real_dis_L > 16)
    {
      Run_Sumo(speed);
      P_distanceL(Real_dis_L, 16);
    }
    else
    {
      Run_Sumo(70);
      angle_Left(fl, bl);
    }
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
  }

if(counter_angle_Right == 1)
{
  servo_write(pos_R);
  mpu.update();
  reference = mpu.getAngleZ();
  for(int i=0;i<50;i++)
  {
    delay(10);
    mpu.update();
    PID_angle(mpu.getAngleZ());
  }
}
  detected_cube = 'G';
  detected_ser = 'N';
}
void follow_G_cw()
{

  servo_write(pos_R);
  while (true) {
   
    sensor();
    angle_Left(dis_L1, dis_L2);
    if (fr < 100 && dis_B > 80)break;

  }

  detected_cube = 'G';
  detected_ser = 'N';
}
void follow_G_cww() {
  sensor();
  servo_write(pos_R);
  while (true) {
    
    sensor();
    angle_Right(fr, br);
    if (bl < 40 && dis_B > 80)break;

  }

  detected_cube = 'G';
  detected_ser = 'N';
}
void follow_G_GN() {
  Ana_Khales();

 
  while (true) {

    sensor();
    if (counter_angle_Right == 1 && fl >= 50)break;
    if(((counter_angle_Left == 0 && counter_angle_Right == 0) || counter_angle_Left == 1)  && (fr > 100 || fl > 100))break;    
    
    
    if (Real_dis_L < 7)
    {
      Run_Sumo(speed);
      P_distanceL(Real_dis_L, 7);
    }
    else if (Real_dis_L > 17)//15 $
    {
      Run_Sumo(speed);
      P_distanceL(Real_dis_L, 17);//13 $
    }
    else
    {
      Run_Sumo(70);
      angle_Left(fl, bl);
    }
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
  }
  if(counter_angle_Right == 1)
  {
  servo_write(pos_R);
  mpu.update();
  reference = mpu.getAngleZ();
  for(int i=0;i<50;i++)
  {
    delay(10);
    mpu.update();
    PID_angle(mpu.getAngleZ());
  }
  }
         
  detected_cube = 'G';
  detected_ser = 'N';
}
void follow_R_cw()
{  
  servo_write(pos_R);
  while (true) {
    sensor();
    if (br < 40 && dis_B > 80)break;
    angle_Left(fl, bl);
  }
  detected_cube = 'R';
  detected_ser = 'N';
}
void follow_R_Counter() {
  Ana_Khales();
  while (true) {

    sensor();
    if(counter_angle_Left == 1 && fr > 50)break;
    if(((counter_angle_Left == 0 && counter_angle_Right == 0) || counter_angle_Right == 1) && (fr > 100 || fl > 100))break;
    if (Real_dis_R < 7 )
    {
      Run_Sumo(speed);
      P_distanceR(Real_dis_R, 7);
    }
    else if (Real_dis_R > 14)
    {
      Run_Sumo(speed);
      P_distanceR(Real_dis_R, 14);
    }
    else
    {
      Run_Sumo(70);
      angle_FollowR(fr, br); 
    }
  }
  if(counter_angle_Left == 1)
  {
  servo_write(pos_R);
  mpu.update();
  reference = mpu.getAngleZ();
  for(int i=0;i<50;i++)
  {
    delay(10);
    mpu.update();
    PID_angle(mpu.getAngleZ());
  }
  }
  detected_cube = 'R';
  detected_ser = 'N';
}
void follow_R_ccw() {
  servo_write(pos_R);
  while (true) {
    sensor();
    if (fl < 100 && dis_B > 80)break;

    if (Real_dis_R < 7)
    {
      P_distanceR(Real_dis_R, 7);
    }
    else if (Real_dis_R > 13)
    {
      P_distanceR(Real_dis_R,13);
    }
    else
    {
      angle_Right(fr, br);
    }
  }
  detected_cube = 'R';
  detected_ser = 'N';
}
////////////////////////////////////////////////////////////////////////////// End of following functions //////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////// Avoiding functions are described and explained here //////////////////////////////////////////////////////////////////////////////
/*
A typical avoiding functions in the code works as the following:
1- First it checks for input signal from the Raspberry Pi
2- In case the robot is close enough to the wall and to correct side of the pillar, then no need for turning an angle to avoid it.
3- It calculates the angle between the robot and the wall and convert it to radians (An upper limit is set to the angle, this helps in avoiding dangerous angle turn).
4- In case of the need for turning, there is a loop for turning, which only breaks in case the robot reaches the angle or becomes very close to the wall.
5- Keep moving with previous orientation until the robot reaches a specified distance from the wall.
6- Turn back the same angle.
7- Call the proper following function.
8- "avoid_R" and "avoid_G" functions are called one time in the first section only, in case of other avoiding functions, the robot checks the recieved information. This information may indicate that there is
another pillar of different color in the same section, the program calls another avoiding function in that case ("avoid_RG","avoid_GR"). In the other case, which is the existence of another pillar of same color and no pillar, the program
calls one of the following functions ("follow_G_GN", "follow_R_cw", "follow_R_ccw","follow_G_cw", "follow_G_ccw") or ("follow_R_Counter" , "follow_G_Counter") in case of calling "avoid_R"/"avoid_G" functions.
It should be noted that the functions below may have slight differences, this is mainly controlled by calibration and testing conditions. However, the general principle is the same in all of them.
To better understande the code, "avoid_G" and "avoid_G_cw" are explained in details.
*/
void avoid_G(double dis = 15) {
  if (Serial.available() > 0)/// Check for Raspberry Pi recived message
  {
    data = Serial.readStringUntil('#');
  }
  sensor();
  Run_Sumo(speed);
  if (Real_dis_L > 15 ) /// In case the robot is close enough to the wall and to the left of the green pillar, then there is no need to turn an angle to avoid it.
  {
    Real_dis_R = Distance(dis_R1, dis_R2);// Calculate the prependicular distance
    if(counter_angle_Right == 1 || (counter_angle_Right == 0 && counter_angle_Left == 0))// In case that the direction is counter clockwise (CCW) or the direction is unkown, calculate the angle using right sensors.
    {
      theta = abs(atan(1.5 * (100 - Real_dis_R - 10) / 60 ));
      angle = angle_cal(dis_R1,dis_R2);
    }
    if (counter_angle_Left == 1)//In case that the direction is clockwise (CW), calculate the angle using left sensors.
    {
      theta = abs(atan(1.5 * (Real_dis_L - 10) / 60 ));
      angle = angle_cal(dis_L1,dis_L2);
    }
    theta = theta * 180 / pi;// convert the angle from Radian to degrees.
    if(theta > 40) theta = 40;// Upper limit for the angle
    desired_angle = theta;
    angle = angle * 180 / pi;
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    ref_mpu = theta_mpu;
    theta -= angle;
    theta += theta_mpu;// the robot will turn until it reaches this angle
    dis_D2L = get_distance_D2L();
    while (abs(theta_mpu - theta) >= 2 && (dis_D2L > dis || dis_D2L<1))//Turn until reaching the angle or becomeing very close to the wall
    {
      if (Serial.available() > 0)// Check for Raspberry Pi recieved signals
      {
        data = Serial.readStringUntil('#');
      }
      dis_D2L = get_distance_D2L();
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      delay(10);
      servo_write(pos_R - 40);
    }
    //stop summo & delay 500 deleted (edit)

    dis_D2L = get_distance_D2L();
    delay(10);
    //run sumo deleted
    mpu.update();
    reference = mpu.getAngleZ();
    servo_write(pos_L);
   if (abs(desired_angle) > 5 )// In case the calculated angle is big enough
    {
      while (dis_D2L >= dis || dis_D2L<1)// After reaching the angle, keep moving in a straight line until the robot becomes in a specified distance from the wall.
      {
        sensor();
        if (Serial.available() > 0)
        {
          data = Serial.readStringUntil('#');
        }
        mpu.update();
        theta_mpu = mpu.getAngleZ();
        PID_angle(theta_mpu);
        dis_D2L = get_distance_D2L();
        delay(10);
      }
    }
    Run_Sumo(speed);

    mpu.update();
    theta_mpu = mpu.getAngleZ();

    while (abs(theta_mpu - ref_mpu) >= 2)// Turn back to the original angle (Paralell to the wall)
    {
      if (Serial.available() > 0)
      {
        data = Serial.readStringUntil('#');
      }
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      delay(10);
      servo_write(pos_R + 40);
    }

  }

  servo_write(pos_R);// Reset the servo's angle
  follow_G_Counter();//Follow on a straight line
  theta = 0;// reset the values
  angle = 0;// reseet the values

}
void avoid_G_cw(String info = "N" ,double dis = 13)
{
  sensor();
  Run_Sumo(speed);
  Real_dis_L = Distance(dis_L1, dis_L2);// calculate the prependicular distance from the wall
  if (Real_dis_L > 15 )/// In case the robot is close enough to the wall and to the left of the green pillar, then there is no need to turn an angle to avoid it.
  {
    theta = abs(atan(2 * (Real_dis_L - 10) / 60 ));// calculate the angle between the robot and the wall
    theta = theta * 180 / pi;// convert the angle from Radian to degrees.
    if(theta > 40) theta = 40;//Set upper limit for the angle 
    desired_angle = theta;
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    ref_mpu = theta_mpu;
    theta += theta_mpu;
    dis_D2L = get_distance_D2L();
    while (abs(theta_mpu - theta) >= 2 && dis_D2L > dis )//Turn until reaching the angle or becomeing very close to the wall
    {
      if (Serial.available() > 0)
      {
        data = Serial.readStringUntil('#');
      }
      dis_D2L = get_distance_D2L();
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      delay(10); 
      servo_write(pos_R - 40);
     
    }
    dis_D2L = get_distance_D2L();
    delay(10);
    mpu.update();
    reference = mpu.getAngleZ();
    servo_write(pos_L);
    if (abs(desired_angle) > 5)
    {
    while (dis_D2L >= dis)// After reaching the angle, keep moving in a straight line until the robot becomes in a specified distance from the wall.
    {
      sensor();
      if (Serial.available() > 0)
      {
        data = Serial.readStringUntil('#');
      }
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      PID_angle(theta_mpu);
      dis_D2L = get_distance_D2L();
      delay(10);
    }
    }
    Run_Sumo(speed);

    mpu.update();
    theta_mpu = mpu.getAngleZ();

    while (abs(theta_mpu - ref_mpu) >= 2 )// Turn back to the original angle (Paralell to the wall)
    {
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      delay(10);
      servo_write(pos_R + 40);
    }
  }
  follow_G_cw();// Keep following while moving until reaching the current section
  theta = 0;
  angle = 0;
  Run_Sumo(speed);
 
    if (info == "G" || info == "GG") {// In case the next pillar is green "Same color", then keep following until reaching a turn.

      follow_G_GN();

    }
   
    if (info == "GR") {// In case the next pillar is red "Different color",  then call another avoiding function.

      avoid_GR();
    }
  detected_first = 0;
  detected_ser = 'N';
}
void avoid_G_ccw( String info="N", double dis = 59) {
  if (Serial.available() > 0)
  {
    data = Serial.readStringUntil('#');
  }

  sensor();
  Run_Sumo(speed);
  theta = 35;

  mpu.update();
  theta_mpu = mpu.getAngleZ();
  ref_mpu = theta_mpu;
  theta += theta_mpu;
  while (abs(theta_mpu - theta) >= 2)
  {
    if (Serial.available() > 0)
    {
      data = Serial.readStringUntil('#');
    }
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_R - 40);
  }
  Run_Sumo(speed);
  mpu.update();
  reference = mpu.getAngleZ();
  dis_BR = get_distance_BR();
  delay(10);
  while (dis_BR < dis)
  {
    if (Serial.available() > 0)
    {
      data = Serial.readStringUntil('#');
    }
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    PID_angle(theta_mpu);
    dis_BR = get_distance_BR();
    delay(10);
  }

  Run_Sumo(speed);
  sensor();
  angle  = angle * 180 / pi;
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  while (abs(theta_mpu - ref_mpu) >= 10)//ammar
  {

    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_R + 40);
  }
  
  Run_Sumo(speed);

  follow_G_cww();
  theta = 0;
  angle = 0; 

  Run_Sumo(40);
  
    if (info == "G" || info == "GG") {
      follow_G_GN();
    }
   
    if (info == "GR") {
      avoid_GR();
    }
  
 
  detected_first = 0;
  detected_ser = 'N';
}

void avoid_R_cw(String info = "N", double dis = 62)
{

  sensor();
  Run_Sumo(speed);
  theta = 35;
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  ref_mpu = theta_mpu;
  theta -= theta_mpu;
  servo_write(pos_R);
  while ((theta_mpu + theta) >= 0)
  {
    if (Serial.available() > 0)
    {
      data = Serial.readStringUntil('#');
    }
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_R + 40);
  }
  dis_BL = get_distance_BL();

  delay(10);
  mpu.update();
  reference = mpu.getAngleZ();
  servo_write(pos_R);
  while (dis_BL < dis)
  {
    if (Serial.available() > 0)
    {
      data = Serial.readStringUntil('#');
    }
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    PID_angle(theta_mpu);
    dis_BL = get_distance_BL();
    delay(10);
  }
  mpu.update();
  theta_mpu = mpu.getAngleZ();

  while (abs(theta_mpu - ref_mpu) >= 2)
  {


    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_L - 40);
  }

  follow_R_cw();
  theta = 0;
  angle = 0;


  Run_Sumo(speed);


    if (info == "R" || info == "RR") {

      follow_R_Counter();

    }
    else if (info == "RG") {

      avoid_RG();
    }
   
  detected_first = 0;
  detected_ser = 'N';
}
void avoid_R(double dis = 17) {
  
  Run_Sumo(speed);

  if (Serial.available() > 0)
  {
    data = Serial.readStringUntil('#');
  }
  sensor();
  if (Real_dis_R > 16) {
    Run_Sumo(speed);
    if (counter_angle_Right == 1 || (counter_angle_Right == 0 && counter_angle_Left == 0))
    {
      theta = abs(atan(2 * (Real_dis_R - 10) / 60)); 
      angle = angle_cal(dis_R1,dis_R2);
    }
    if (counter_angle_Left == 1)
    {
      theta = abs(atan(1.5 * (100 - Real_dis_L - 10) / 60)); 
      angle = angle_cal(dis_L1,dis_L2);
      
    }
    if (theta < 0)
      theta = 0;

    theta = theta * 180 / pi;
    if(theta > 40){theta = 40;}
    desired_angle = theta;

    mpu.update();
    theta_mpu = mpu.getAngleZ();
    ref_mpu = theta_mpu;
    
    angle = angle * 180 / pi;
    theta += angle;
    theta -= theta_mpu;
    servo_write(pos_R);
    while ((theta_mpu + theta) >= 0)
    {
      if (Serial.available() > 0)
      {
        data = Serial.readStringUntil('#');
      }
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      delay(10);
      servo_write(pos_R + 40);
    }
    dis_D2R = get_distance_D2R();
    mpu.update();
    endReTurn = mpu.getAngleZ();

    delay(10);
    get_distance_F();
    delay(10);
    //run sumo deletd (edit)
    mpu.update();
    reference = mpu.getAngleZ();
    servo_write(pos_R);
    if (abs(desired_angle) > 5 )
    {
    while (dis_D2R >= dis)
    {
      if (Serial.available() > 0)
      {
        data = Serial.readStringUntil('#');
      }
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      PID_angle(theta_mpu);
      dis_D2R = get_distance_D2R();
      delay(10);
    }
    }

    mpu.update();
    theta_mpu = mpu.getAngleZ();
    while (abs(theta_mpu - ref_mpu) >= 2)
    {
      if (Serial.available() > 0)
      {
        data = Serial.readStringUntil('#');
      }
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      delay(10);
      servo_write(pos_L - 40);
    }
  }
  servo_write(pos_L);
  follow_R_Counter();
  theta = 0;
  angle = 0;
}
void avoid_R_ccw(String info = "N",double dis = 17) {
  if (Serial.available() > 0)
  {
    data = Serial.readStringUntil('#');
  }
  sensor();
  Run_Sumo(speed);
  Real_dis_R = Distance(dis_R1, dis_R2);
  if (Real_dis_R > 16) {
    theta = abs(atan(2 * (Real_dis_R - 15) / 60));
    if (theta < 0)
      theta = 0;

    theta = theta * 180 / pi;
    if(theta > 40) theta = 40;
    desired_angle = theta;
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    ref_mpu = theta_mpu;
    angle = angle * 180 / pi;
    theta += angle;
    theta -= theta_mpu;
    servo_write(pos_R);
    while ((theta_mpu + theta) >= 0)
    {
      if (Serial.available() > 0)
      {
        data = Serial.readStringUntil('#');
      }
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      delay(10);
      servo_write(pos_R + 40);
    }
    dis_D2R = get_distance_D2R();
    endReTurn = mpu.getAngleZ();
    delay(15);
    get_distance_F();
    delay(10);
    Run_Sumo(speed);
    mpu.update();
    reference = mpu.getAngleZ();
    servo_write(pos_R);
    if (abs(desired_angle) > 5 )
    {
    while (dis_D2R >= dis)
    {
      if (Serial.available() > 0)
      {
        data = Serial.readStringUntil('#');
      }
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      PID_angle(theta_mpu);
      dis_D2R = get_distance_D2R();
      delay(10);
      get_distance_F();
      delay(10);
    }
    }
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    ref_mpu -= 10;
    while (abs(theta_mpu - ref_mpu) >= 2)
    {
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      delay(10);
      servo_write(pos_L - 40);
    }
  }
  follow_R_ccw();
  theta = 0;
  angle = 0;
  Run_Sumo(speed);
    if (info == "R" || info == "RR") {

      follow_R_Counter();
  
    }
    if (info == "RG") {
      avoid_RG(13);
    }
   

  detected_first = 0;
  detected_ser = 'N';
}

void avoid_GR(double dis = 19) {
  if (counter_1 == 12)
  {
    Ana_Khales();
  }
  if (Serial.available() > 0)
  {
    data = Serial.readStringUntil('#');
  }

  Run_Sumo(speed);
  sensor();
  if (counter_angle_Left) {
    theta = 35;
  }
  else if (counter_angle_Right) {
    theta = 35; 
  }
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  delay(10);
  ref_mpu = theta_mpu;
  
  theta = theta - ref_mpu;
 
  while (theta_mpu + theta >= 0)
  {
    if (Serial.available() > 0)
    {
      data = Serial.readStringUntil('#');
    }
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_R + 40);
    Run_Sumo(speed);
  }
  

  dis_D2R = get_distance_D2R();
  
  mpu.update();
  reference = mpu.getAngleZ();
  endReTurn = mpu.getAngleZ();
  delay(10);
  get_distance_F();
  delay(10);
  servo_write(pos_R);
  Run_Sumo(speed);
  while (dis_D2R >= dis)
  {
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    PID_angle(theta_mpu);
    dis_D2R = get_distance_D2R();
    delay(10);
    if (Serial.available() > 0)
    {
      data = Serial.readStringUntil('#');
    }
    get_distance_F();
    delay(10);
  }
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  while (abs(theta_mpu - ref_mpu) >= 2)
  {
    if (Serial.available() > 0)
    {
      data = Serial.readStringUntil('#');
    }
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_L - 40);
  }

  follow_R_Counter();
  theta = 0;
  angle = 0;
}

void avoid_RG(double dis = 15) {
  if (counter_1 == 12)
  {
    Ana_Khales();
  }
  if (Serial.available() > 0)
  {
    data = Serial.readStringUntil('#');
  }

  sensor();
  Run_Sumo(speed);
  if (counter_angle_Left) {
    theta = 40;
  }
  else if (counter_angle_Right) {
    theta = 45;
  }
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  ref_mpu = theta_mpu -7;//
  theta += theta_mpu;
  while (abs(theta_mpu - theta) >= 2)
  {
    if (Serial.available() > 0)
    {
      data = Serial.readStringUntil('#');
    }
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_R - 40);
  }
  dis_D2L = get_distance_D2L();
  delay(20);
  mpu.update();
  reference = mpu.getAngleZ();
  servo_write(pos_R);
  Run_Sumo(80);
  while (dis_D2L >= dis)
  {
    if (Serial.available() > 0)
    {
      data = Serial.readStringUntil('#');
    }
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    PID_angle(theta_mpu);
    dis_D2L = get_distance_D2L();
    delay(10);
  }
  Run_Sumo(speed);

  mpu.update();
  theta_mpu = mpu.getAngleZ();
  while (abs(theta_mpu - ref_mpu) >= 2)
  {
    if (Serial.available() > 0)
    {
      data = Serial.readStringUntil('#');
    }
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_R + 40);
  }

  follow_G_Counter();
  theta = 0;
  angle = 0;
}
////////////////////////////////////////////////////////////////////////////// End of avoiding functions //////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////Turn right and turn left functions are here //////////////////////////////////////////////////////////////////////////////
/*
Most of the work is done here, the turn function is responsible for turn 90 degrees and put the robot in the specified position, in which the robot is parallel to the wall and facing the next section. This ensures that the camera can view all of the next section in addition to 
the correct movement while entering the next section.
"Turn_L_Back" function working steps are explained below. For "Turn_R_Back" function, the same steps are applied but with differences in direction:
Check whether the direction of rotation was reversed or not:
- if it is, then look directly for the recived data (This is because in that case, the program will call the opposite direction function directly after reversing the direction of rotation. This means that the robot is in the correct position).
- if it is not, then increment the number of turns by one, and check for the need of reversing the direction of rotation. There are also two cases here:
  - In case the last detected cube was red and the robot has completed 9 turns, then it decreases the numbers of turns and calls the rotation reversing function "turn_last_to_cw", the correct function for turning now is "Turn_R_Back".
  - In the other case, the program will check the last detcted pillar again:
    - The robot will keep moving until reaching a specified area depending on the last detected pillar.
    - If the last detected pillar was green or no pillar was detected, then the robot will have to turn 90 degress backwards, but before turning, the robot moves forward in a small angle of 35 degrees in the counter clockwise direction CCW. This step is necessary to avoid colliding with the pillars in the next section will moving backward.
      Afterthat, the robot will keep moving backwards until it becomes on a certian distance from the wall in the back.
    - If the last detected pillar was red, this means that the robot does not have enough distance to move 90 degrees backwards. Instead, it moves 90 degrees in the front direction, then it moves backward until it reaches a certian distance from the wall in the back.
  - After completing the turn, the robot is in the proper position and ready to move to the next section. The program will send a signal to the Raspberry Pi to begin image processing. Then it recieves the decision made by Raspberry Pi. Based on this decision, the robot will move to the next seciton.
  - After completing the section, the program goes back to the beginning of the "Turn_L_Back" function.
*/
void Turn_L_Back() {
turnL:
  if (!last_turn_to_L)// Check if the robot has reversed its motion direction
  { 
    counter_angle_Right = 1;// The direction of motion is counter clockwise CCW
    counter_1 = counter_1 + 1;// Increment the number of encountered corner sections by one
    if (counter_1 == 9 && detected_cube == 'R' && !Ana_Last && !start_G)//check whether there is a need for reversing the direction of movement
    {
      Ana_Last = true;
      turn_last_to_cw();// Reverse the direction of rotation
      counter_angle_Left = 1;// CW true
      counter_angle_Right = 0;// CCW false
      counter_1 = counter_1 - 1;// decrease the corner
      last_turn_to_R = true;// Reversed the direction true
      Turn_R_Back();// Use the other direction turn function
      return ;// skip the rest of the code in the function
    }

    Run_Sumo(speed);

    if (detected_cube == 'G') {// if the last detected pillar was green
      sensor();
      while (f > 30) {// keep moving until reaching a specific distance from the wall
        sensor();
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        angle_Right(fr, br);
      }

    }
    if (detected_cube == 'N' || detected_cube == 'G') {// if the last detected pillar was green or nothing
      if (detected_cube == 'N')// if there was no pillar detected
      {
        servo_write(pos_R);
          while (true) {// keep f
          if (Serial.available() > 0) {// check for Raspberry Pi recieved message
            data = Serial.readStringUntil('#');
          }
          sensor();
          if (f < 42)break;// break when reaching a specified distance from the front wall
          if (Real_dis_R < 10)// UPA
          {
            P_distanceR(Real_dis_R, 10);
          }
          else if (Real_dis_R > 70)
          {
            P_distanceR(Real_dis_R, 70);// UPA
          }
          else
          {
            angle_Right(fr, br);//PA
            Run_Sumo(speed);

         }
        }
         Run_Sumo(speed);
        
      }

      Real_dis_R = Distance(fr, br);
      theta = 35;
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta += theta_mpu;
      servo_write(pos_L);
      f = get_distance_F();
      delay(10);
      dis_D2R = get_distance_D2R();
      delay(10);
      while (abs(theta_mpu - theta) >= 2 )// turn a small angle to avoid colliding with pillars while moving backward.
      {
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }

        dis_D2R = get_distance_D2R();
        f = get_distance_F();
        delay(10);
        mpu.update();
        if (dis_D2R > 1 && dis_D2R < 7)break;// If the robot is very close to the wall, then break
        if (f > 1 && f < 7)break;
        theta_mpu = mpu.getAngleZ();
        delay(10);
        servo_write(pos_R - 40);
      }
      Stop_Sumo();
      delay(500);
      theta = 70;
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      theta += ref_mpu;
       sensor();
      Back_Sumo(60);
      while ((theta_mpu - theta) <= 0)// move backward until becoming parallel to the wall
      {
        if (Serial.available() > 0)
        {
          data = Serial.readStringUntil('#');
        }
        if (dis_B < 7 && dis_B >= 1)break;
       
        mpu.update();
        theta_mpu = mpu.getAngleZ();
         sensor();
        servo_write(pos_R + 50);
      }
      dis_B = get_distance_B();
      delay(10);
      servo_write(pos_R);
     
      sensor();
      Back_Sumo(60);
      while (b > 16 || b < 1) {// keep moving backwards  until reaching a specified distance from the wall
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        sensor();
        angle_Right(dis_R2, dis_R1);
      }
    }

    if (detected_cube == 'R') {// In case the last pillar was red
      servo_write(pos_R);      
      sensor();
      Run_Sumo(speed);
      while (dis_F > 60) {// Keep moving until reaching a specified distance from the front wall
        sensor();
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        angle_Right(fr, br);

      }

      Run_Sumo(60);
      theta = 90;
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta += ref_mpu;

      while (abs(theta_mpu - theta) >= 2)// turn 90 degrees in the front direction
      {
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        mpu.update();
        theta_mpu = mpu.getAngleZ();
        delay(10);
        servo_write(pos_R - 40);
      }
      servo_write(pos_R);
     
 
      sensor();
      while (dis_B > 16 || b < 1) {// Go backward until reaching a specified distance from the wall
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        sensor();
        angle_Right(dis_R2, dis_R1);
              Back_Sumo(60);
      }
    }
  
  }
  Stop_Sumo();
  delay(200);
      Serial.println("L");  // Send a signal to the Raspberry Pi to begin image processing and send the decision
      delay(200);
           while (Serial.available() > 0)// read the incoming data from Raspberry Pi
            {
             
                 ser = Serial.readStringUntil('#');
              if(ser == "RR" || ser == "R" || ser == "RG" || ser == "GR" || ser == "GG" || ser == "G" )
                {
                  data = ser;
               
                }
            }
  last_turn_to_L = false ;
  sensor();
  Run_Sumo(speed);
  servo_write(pos_R);// reset the servo motor's angle
  
    if (data == "G" || data == "GR" || data == "GG") {// if the next pillar is green, then call "avoid_G_ccw" function and go the beginning of "Turn_L_Back" function

        avoid_G_ccw(data);
        goto turnL;

    }
    else if (data == "R" || data == "RG" || data == "RR") {// if the next pillar is red, then call "avoid_R_ccw" function and go the beginning of "Turn_L_Back" function
     
        avoid_R_ccw(data); 
        goto turnL;
    }
  

  detected_first = 0;// reset the values
  detected_cube = 'N';
  error = 0;
  correction_it = 0;
  mpu.update();
  reference = mpu.getAngleZ();
  Run_Sumo(speed);

}

void Turn_R_Back() {
turnR:

  if (!last_turn_to_R)
  {
    
    
    counter_angle_Left = 1;
    counter_1 = counter_1 + 1;
    if (counter_1 == 9 && detected_cube == 'R' && !Ana_Last  && !start_G)
    {
      counter_1 = counter_1 - 1;
      Ana_Last = true;
      turn_last_to_ccw();
      counter_angle_Left = 0;
      counter_angle_Right = 1;
      last_turn_to_L = true;
      Turn_L_Back();
      return ;


    }
    Run_Sumo(speed);

    if (detected_cube == 'R') {    
      sensor();
      while (f > 25) {
        sensor();
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        angle_Left(fl, bl);
      }

    }

    if (detected_cube == 'N' || detected_cube == 'R') {
      if (detected_cube == 'N')
      {
        servo_write(pos_R);
        while (true) {
          if (Serial.available() > 0) {
            data = Serial.readStringUntil('#');
          }

       
          sensor();
     
          if (f < 42 && detected_cube == 'N')break;
          if (Real_dis_L < 10)
          {
            P_distanceL(Real_dis_L, 10);
          }
          else if (Real_dis_L > 70)
          {
            P_distanceL(Real_dis_L, 70);
          }
          else
          {
            angle_Left(fl, bl);
          }
        }
      }

      theta = 35;
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta = ref_mpu - theta;
      servo_write(pos_L);
      f = get_distance_F();
      delay(10);
      dis_D2L = get_distance_D2L();
      delay(10);
      while (abs(theta_mpu - theta) >= 2 )
      {
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        dis_D2L = get_distance_D2L();
        delay(10);
        f = get_distance_F();
        mpu.update();
        if (dis_D2L > 1 && dis_D2L < 7)break;
        if (f > 1 && f < 7)break;
        theta_mpu = mpu.getAngleZ();
        delay(10);
        servo_write(pos_R + 40);
      }
      theta = 70;
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      theta = ref_mpu - theta;
        sensor();
      Back_Sumo(60);
      while ((theta_mpu - theta) >= 0)
      {
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        if (dis_B < 7 && dis_B >= 1)break;
       
        mpu.update();
        theta_mpu = mpu.getAngleZ();
         sensor();
        servo_write(pos_R - 40);
      }
      dis_B = get_distance_B();
      delay(10);
      servo_write(pos_R);
      Back_Sumo(60);
     
     
      while (b > 16 || b < 1) {
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        sensor();
        angle_Left_Back(dis_L2, dis_L1);
      }

    }

    if (detected_cube == 'G') {
      servo_write(pos_R);
    
      sensor();

      while (dis_F > 60) {
        
        if (Serial.available() > 0) {//$$%
          data = Serial.readStringUntil('#');
        }
        sensor();
        angle_Left(fl, bl);
      }
     while (dis_F < 50) {
        
        if (Serial.available() > 0) {//$$%
          data = Serial.readStringUntil('#');
        }
        sensor();
        angle_Left(bl,fl);
        Back_Sumo(speed);
      }
      Run_Sumo(60);
      theta = 90;
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta = ref_mpu - theta;


      while (abs(theta_mpu - theta) >= 2)
      {
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        mpu.update();
        theta_mpu = mpu.getAngleZ();
        delay(10);
        servo_write(pos_R + 40);
      }
      servo_write(pos_R);
     Stop_Sumo();
      delay(150);
      sensor();
      
      while (dis_B > 16|| b < 1) {
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }  
        angle_Left_Back(dis_L2, dis_L1);
           sensor();
            Back_Sumo(60);
      }
    }
  }
    
   Stop_Sumo();
  
      Serial.println("R");  
      delay(200);
           while (Serial.available() > 0)
            {
             
                 ser = Serial.readStringUntil('#');
              if(ser == "RR" || ser == "R" || ser == "RG" || ser == "GR" || ser == "GG" || ser == "G" )
                {
                  data = ser;
               
                }
            }
  last_turn_to_R = false;
  sensor();
  Run_Sumo(speed);
  servo_write(pos_R);
  
 
    if (data == "G" || data == "GG" || data == "GR") {    
        avoid_G_cw(data);
        goto turnR;
    }
    
    else if (data == "R" || data == "RR" || data == "RG") {
        avoid_R_cw(data);
        goto turnR;
    }


  detected_first = 0;
  detected_cube = 'N';
  error = 0;
  correction_it = 0;
  mpu.update();
  reference = mpu.getAngleZ();
  Run_Sumo(speed);

}

////////////////////////////////////////////////////////////////////////////// Direction reversing functions are here //////////////////////////////////////////////////////////////////////////////
/*
We will consider changing direction from CCW to CW,  changing direction from CW to CCW is the same but with opposite movements. The way in which the direction is changed from CCW to CW can be divided to the following steps:

1- Keep moving until the front distance sensor reads a distance less than 75cm.
2- Move 90 in the CCW direction.
3- Move backwards until the back distance sensor reads less than 45cm.
4- Move front in the CCW direction until reaching an angle that is equal to 30 degrees.
5- turn 80 degress and move backwards in the CCW direction.
6- Move backwards until the back distance sensor reads a distance less than 10cm.

*/
void turn_last_to_cw()
{
  sensor();
  servo_write(pos_R);
  Run_Sumo(speed);
  while (dis_F > 75) {
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    sensor();

    angle_Right(dis_R1, dis_R2);

  }
  Stop_Sumo();
  delay(50);
  Run_Sumo(speed);
  theta = 90;
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  ref_mpu = theta_mpu;
  theta += ref_mpu;
  
  while (true)
  {
    if((theta_mpu - theta) >= 0 )break;
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_R - 40);
  }
  servo_write(pos_R);
  sensor();
  Back_Sumo(40);
  while (dis_B > 45) { 
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    sensor();
    angle_Right(dis_R2, dis_R1);
  }
  Stop_Sumo();
  delay(50);
  Run_Sumo(speed);

  theta = 30;
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  dis_F = get_distance_F();
  delay(10);
  dis_D2L = get_distance_D2L();
  delay(10);

  ref_mpu = theta_mpu;
  theta += theta_mpu;
  servo_write(pos_L);
  while (abs(theta_mpu - theta) >= 2 )
  {
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    dis_D2L = get_distance_D2L();
    delay(10);
    dis_F = get_distance_F();
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    if (dis_D2L > 1 && dis_D2L < 7)break;
    if (dis_F > 1 && dis_F < 7)break;
    servo_write(pos_R - 40);
  }
  Stop_Sumo();
  delay(50);
  theta = 80;
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  theta += ref_mpu;
  Back_Sumo(50);
  while (abs(theta_mpu - theta) >= 2)
  {
    dis_B = get_distance_B();
    delay(20);
    if((theta_mpu - theta) >= 0 || (dis_B<7 && dis_B>1))break;
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    if (dis_B < 5 && dis_B >= 1)break;
    sensor();
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_R + 40);
  }
  dis_B = get_distance_B();
  delay(10);
  servo_write(pos_R);
  Back_Sumo(50);
  while (b > 10) {
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    sensor();
    angle_Left_Back(bl, fl);
  }
  Stop_Sumo();
  delay(100);
  Run_Sumo(speed);


}
void turn_last_to_ccw()
{
  sensor();
  servo_write(pos_R);
  Run_Sumo(speed);
  while (dis_F > 60) {
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    sensor();

    angle_Left(dis_L1, dis_L2);

  }
  while(dis_F <60){
    sensor();
    Back_Sumo(speed);
    angle_Left(dis_L2,dis_L1);
  }
  
  Run_Sumo(speed);
  theta = 90;
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  ref_mpu = theta_mpu;
  theta += ref_mpu;
  while (abs(theta_mpu - theta) >= 2)
  {
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_R - 40);
  }
  servo_write(pos_R);
  sensor();
  Back_Sumo(40);
  while (dis_F > 37) {
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    sensor();
    angle_Right(dis_R2, dis_R1);
  }
  Stop_Sumo();
  delay(50);
  Run_Sumo(speed);

  theta = 30;
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  dis_F = get_distance_F();
  delay(10);
  dis_D2L = get_distance_D2L();
  delay(10);

  ref_mpu = theta_mpu;
  theta += theta_mpu;
  servo_write(pos_L);
  while (abs(theta_mpu - theta) >= 2 )
  {
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    dis_D2L = get_distance_D2L();
    delay(10);
    dis_F = get_distance_F();
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    if (dis_D2L > 1 && dis_D2L < 10)break;
    if (dis_F > 1 && dis_F < 10)break;
    servo_write(pos_R - 40);
  }
  Stop_Sumo();
  delay(50);
  theta = 60;
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  theta += ref_mpu;
  Back_Sumo(50);
  while ((theta_mpu - theta) <= 0)
  {
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    if (dis_B < 5 && dis_B >= 1)break;
    sensor();
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_R + 40);
  }
  dis_B = get_distance_B();
  delay(10);
  servo_write(pos_R);
  Back_Sumo(50);
  while (true) {
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    if(b<10 && b>1){break;}
    sensor();
    angle_Right(br, fr);
  }
    Stop_Sumo();
      delay(100);
      Run_Sumo(speed);

}
////////////////////////////////////////////////////////////////////////////// End of direction reversing functions //////////////////////////////////////////////////////////////////////////////

void Ana_Khales()// This function is responsilbe for checking the end of the three laps and stop the robot.
{

  if (counter_1 == 12)
  {
    Run_Sumo(speed);
    for (int i = 0; i < 15; i++)
    { dis_R1 = get_distance_R1();
      delay(10);
      dis_R2 = get_distance_R2();
      delay(10);
      angle_Right(dis_R1, dis_R2);
    }
    Stop_Sumo();
    delay(1000000000);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(Switch, CHANGE);
  pinMode(TRIGGER_PIN_L1, OUTPUT);
  pinMode(ECHO_PIN_L1, INPUT);
  pinMode(TRIGGER_PIN_L2, OUTPUT);
  pinMode(ECHO_PIN_L2, INPUT);
  pinMode(TRIGGER_PIN_R1, OUTPUT);
  pinMode(ECHO_PIN_R1, INPUT);
  pinMode(TRIGGER_PIN_R2, OUTPUT);
  pinMode(ECHO_PIN_R2, INPUT);
  pinMode(TRIGGER_PIN_F, OUTPUT);
  pinMode(ECHO_PIN_F, INPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
    Wire.begin();
    byte status = mpu.begin();
    while (status != 0) {
      
    }
    mpu.calcOffsets();
    mpu.update();
    delay(10);
    myservo.attach (7);
}
bool loop_start_pos = 1;

void loop() {
  if(counter_1 == 0)// This code block is only true when the arduino code begins, it keeps the code in an open loop until start switch is on.
  {
      while (start == 0)
    {
      start = digitalRead(Switch);
      if (Serial.available() > 0) {/// Check whether arduino recieved from Raspberry Pi through serial communication.
        // debug();
        data = Serial.readStringUntil('#');
      }
      delay(1);
    }
  }
  Run_Sumo(speed);
  if (counter_1 >= 12)
  {
    Stop_Sumo();
    delay(100000000);
  }

  sensor();
  if (loop_start_pos) { //This code block also works only at the start of the program, it sets the servo angle to the setpoint and set an mpu reference.
    servo_write(pos_R);
    loop_start_pos = 0;
    mpu.update();
    reference = mpu.getAngleZ();
  }
  if (Serial.available() > 0) {/// Check whether arduino recieved from Raspberry Pi through serial communication.
    data = Serial.readStringUntil('#');
    Run_Sumo(speed);
  }
  if (data == "R" || data == "RG" || data == "RR") {/// Check the stored value in "data" string in the beginning of the round, if the recived data is one of the following characters ("R","RG","RR"), this means that there is a red pillar in the current section to be avoided (ignore the one in the next section in case it is recieved since the algorithm cares about the pillars in the current section only).
    avoid_R();
  }
  else if (data == "G" || data == "GR" || data == "GG") {/// Check the stored value in "data" string in the beginning of the round, if the recived data is one of the following characters ("G","GR","GG"), this means that there is a red pillar in the current section to be avoided (ignore the one in the next section in case it is recieved since the algorithm cares about the pillars in the current section only).
     start_G = true;
    avoid_G();
  }

  else {// Raspberry Pi did not detect any pillar, move straight (followmiddle_straight).
    if (Real_dis_R < 100 && Real_dis_L < 100) {
    
      followmiddle_straight();
    }
  }
  if (counter_angle_Left == 0 && counter_angle_Right == 0 ) {// If the robot did not detect any turn yet (direction of movement is not identified yet).
    if (fl > 100 || bl > 100) {// If one of the left sensors detects a large distance (>100cm), then turn left.

      Turn_L_Back();

    }
    if (fr > 100 || br > 100) {// If one of the right sensors detects a large distance (>100cm), then turn right.
      Turn_R_Back();

    }
  }
  if (counter_angle_Right == 1) {// If the direction of movement is counter clockwise (CCW).
    if ((fl > 100 || bl > 100)&&b>190&&detected_cube=='N') {// If one of the left sensors detects a large distance (>100cm),the robot is in the end of the current section (b>190cm) and no pillars were detected, then turn left.
      Turn_L_Back();
    }
    else if ((fl > 100 || bl > 100)&&(detected_cube == 'R' || detected_cube == 'G'))// If one of the right sensors detects a large distance (>100cm) and the robot avoided one of the pillars, then it is the end of the current section and it has to turn left.
      Turn_L_Back();
    }
  if (counter_angle_Left == 1) {// If the direction of movement is clockwise (CW).
    if ((fr > 100 || br > 100)&&b>190&&detected_cube == 'N') {// If one of the right sensors detects a large distance (>100cm),the robot is in the end of the current section (b>190cm) and no pillars were detected, then turn right.
      Turn_R_Back();
    }
    else if ((fr > 100 || br > 100)&&(detected_cube == 'R' || detected_cube == 'G')) {// If one of the right sensors detects a large distance (>100cm) and the robot avoided one of the pillars, then it is the end of the current section and it has to turn right.
      Turn_R_Back();
    }
  }
}
