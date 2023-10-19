#include <MPU6050_light.h>
#include <ArduinoQueue.h>
#include <NewPing.h>
#include <Servo.h>
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
int pos_R = 90, pos_L = 85, pos_L_Back = 94 , pos_follow_R = 90, turn_pos = 45, turn_pos2 = 45;// TestV3
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
double counter_angle_Right = 0, counter_angle_Left = 0, counter_1 = 0, counter_stop = 0;///////////123
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
/******************Filtering**********************/

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

/******************************************************************/
void avoid_G(double dis = 15);
void avoid_R(double dis = 17);
void read_serial();
void avoid_RG(double dis = 15);
void avoid_GR(double dis = 19);

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

void servo_write(double ang) {
  if (ang > max_angle) {
    ang = max_angle;
  }
  else if (angle < min_angle) {
    ang = min_angle;
  }
  myservo.write(ang);
}

void Ana_Khales()
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
void turn_last_to_cw()
{
  sensor();
  servo_write(pos_R);
  Run_Sumo(speed);
  while (dis_F > 60) {
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
  while (dis_B > 45) { /// 50 //dis_B < 20 is correct
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    sensor();
    angle_Right(dis_R2, dis_R1);
  }
  Stop_Sumo();
  delay(50);
  Run_Sumo(speed);

  theta = 30;///// 35
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


  ////////////
  theta = 70;
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
    angle_Left(bl, fl);
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
  while (dis_F > 37) { /// 50 //dis_B < 20 is correct
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    sensor();
    angle_Right(dis_R2, dis_R1);
  }
  Stop_Sumo();
  delay(50);
  Run_Sumo(speed);

  theta = 30;///// 35
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


  ////////////
  theta = 70;
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  theta += ref_mpu;
  Back_Sumo(50);
  while (abs(theta_mpu - theta) >= 2)
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
  while (b > 10) {
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
    }
    sensor();
    angle_Left(bl, fl);
  }
    Stop_Sumo();
      delay(100);
      Run_Sumo(speed);

}
///////////////////Monday night/////////////////
void following_of_green()
{

  if (detected_ser == 'N') {

    follow_G_Counter();

  }
  if (detected_ser == 'G') {

    follow_G_Counter();


  }
  if (detected_ser == 'R' ) {
    avoid_GR();

  }
  reference_back += 90;
}
void search_ser()
{
  if (Serial.available() > 0)
  {
    data = Serial.readStringUntil('#');

    if (data == "R" || data == "r") {
      detected_ser = 'R';
    }
    if (data == "G" || data == "g") {
      detected_ser = 'G';
    }

  }
}
/////////////////////////////
double get_distance_F ()
{
  duration_F = sonar_F.ping();
  f = (duration_F / 2) * 0.0343;
  //mean_f = Filter(f, mean_f, Means_F);
  return f;
}

double get_distance_B() {
  duration_B = sonar_B.ping();
  b = ((duration_B / 2) * 0.0343);
//  mean_b = Filter(b, mean_b, Means_B);
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
  //mean_fl = Filter(fl, mean_fl, Means_FL);
  return fl ;
}

double get_distance_L2()
{
  duration_L2 = sonar_L2.ping();
  bl = ((duration_L2 / 2) * 0.0343) - 4 ;
  //mean_bl = Filter(bl, mean_bl, Means_BL);
  return bl;
}

double get_distance_R1 ()
{
  duration_R1 = sonar_R1.ping();
  fr = ((duration_R1 / 2) * 0.0343) - 4;
  //mean_fr = Filter(fr, mean_fr, Means_FR);
  return fr;
}

double get_distance_R2()
{
  duration_R2 = sonar_R2.ping();
  br = ((duration_R2 / 2) * 0.0343) - 4;
  //mean_br = Filter(br, mean_br, Means_BR);
  return br;
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
void reset_means()
{
//  while (!Means_B.isEmpty())
//  {
//    Means_B.dequeue();
//    Means_BL.dequeue();
//    Means_BR.dequeue();
//    Means_FL.dequeue();
//    Means_FR.dequeue();
//    Means_F.dequeue();
//
//  }
}
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
//void Print() {
//  Serial.print("dis_L1 ");
//  Serial.println(dis_L1);
//  Serial.print("dis_L2 ");
//  Serial.println(dis_L2);
//  Serial.print("dis_R1 ");
//  Serial.println(dis_R1);
//  Serial.print("dis_R2 ");
//  Serial.println(dis_R2);
//  Serial.print("dis_F ");
//  Serial.println(dis_F);
//  Serial.print("dis_B ");
//  Serial.println(dis_B);
//}
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

double get_angle(double dis_1 , double dis_2) {
  angle =  angle_cal (dis_1, dis_2);
  Angle = (angle * 180) / 3.14;
  return Angle;
}
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
double Distance(double L1 , double L2)
{
  distance = (L1 + L2) / 2;
  angle = angle_cal (L1, L2);
  real_distance = distance * cos(angle);
  return real_distance;
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

double P_distanceR_Red (double real_distance)
{
  conskp = 1.7;
  pos_distance = conskp * (real_distance - Setpoint_Red_R);
  jbes = pos_L + pos_distance;
  myservo.write(jbes);
}
double P_distanceL_Green (double real_distance)
{
  conskp = 1.7;
  pos_distance = conskp * (real_distance - Setpoint_Green_L);
  jbes = pos_L - pos_distance;
  myservo.write(jbes);
}
void Run_Sumo (int pwm)
{
  digitalWrite (R_EN, HIGH);
  digitalWrite (L_EN, HIGH);
  analogWrite (R_PWM, pwm);
  analogWrite (L_PWM, 0);
}

void Stop_Sumo ()
{
  digitalWrite (R_EN, HIGH);
  digitalWrite (L_EN, HIGH);
  analogWrite (R_PWM, 0);
  analogWrite (L_PWM, 0);

}

void Back_Sumo (int pwm)
{
  digitalWrite (R_EN, HIGH);
  digitalWrite (L_EN, HIGH);
  analogWrite (R_PWM, 0);
  analogWrite (L_PWM, pwm);

}

void followmiddle_straight()
{
    /////////$£
  /////////$£
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
  /////////$£

  /////////$£
 
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
      // angle_Left(dis_L1, dis_L2);
      angle_Left(fl, bl);/////////$£
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////LAST ADDDDDD
    ////read_serial();
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
      //Run_Sumo(speed);
    }
//Run_Sumo(speed);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////LAST ADDDDDD
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
  /////////$£
  while (true) {
   
    sensor();
    angle_Left(dis_L1, dis_L2);
    if (fr < 100 && dis_B > 80)break;

  }

  detected_cube = 'G';
  detected_ser = 'N';
}
void follow_G_cww() {
  /////////$£

  //reset_means();
  sensor();
  /////////$£
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
    ////////////////////////////////////////////////////////////////////////////////////////////////////////LAST ADDDDDD
    ////read_serial();
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('#');
      //Run_Sumo(speed);
    }
     //Run_Sumo(speed);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////LAST ADDDDDD
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

  /////////$£
  
  servo_write(pos_R);
  /////////$£
  while (true) {
    //    if (Serial.available() > 0)
    //    {
    //      data = Serial.readStringUntil('#');
    //    }
    sensor();
    if (br < 40 && dis_B > 80)break;
    angle_Left(fl, bl);


  }

  detected_cube = 'R';
  detected_ser = 'N';
}
void follow_R_Counter() {
  Ana_Khales();
  /////////$£

  //reset_means();
//  sensor();
  /////////$£
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
      //angle_FollowR(dis_R1, dis_R2);
      angle_FollowR(fr, br); /////////$£
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
  
//      Stop_Sumo();
//        delay(100000);
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
    theta = 35; // $$$ 46
  }
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  delay(10);
  ref_mpu = theta_mpu;
//  if (counter_angle_Right == 1)
//  {
//    theta -= angleR;
//  }
//  else
//  {
//    theta -= angleL;
//  }
  
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
    ///servo_write(pos_R + 10);
    mpu.update();
//    if (abs(endReTurn - ref_mpu) > 60 && f < 24 && f >= 1) {
//      Stop_Sumo();  /// @@@@$$$
//      delay(5000);
//      Run_Sumo(speed);
//      break;
//    }
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
void avoid_R_cw(String info = "N", double dis = 62)
{

  sensor();
  Run_Sumo(speed);
  // Real distance & theta deleted (edit)
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
  //delay 500 & stop sumo deleted (edit)
  dis_BL = get_distance_BL();

  delay(10);
  //run sumo deletd (edit)
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
      /////// follow_G_Counter(); (edit)
    }
   
  detected_first = 0;
  detected_ser = 'N';
  //////////////////////////////////////////
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
    // Real distance & theta deleted (edit)
    if (counter_angle_Right == 1 || (counter_angle_Right == 0 && counter_angle_Left == 0))
    {
      theta = abs(atan(2 * (Real_dis_R - 10) / 60)); ///////// %% 17
      angle = angle_cal(dis_R1,dis_R2);
    }
    if (counter_angle_Left == 1)
    {
      theta = abs(atan(1.5 * (100 - Real_dis_L - 10) / 60)); //edited $ 1.5  /////////////// %% 17
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
    while ((theta_mpu + theta) >= 0)////////(abs(theta_mpu + theta) >= 2)
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
    //delay 500 & stop sumo deleted (edit)
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
      //if (abs(desired_angle) < 10 || (dis_R1 < 20 || dis_R2 < 20 ))break; //
      //if(abs(endReTurn - ref_mpu) > 60 && f < 24 && f >= 1){Stop_Sumo();delay(5000);Run_Sumo(speed);break;}/// @@@@$$$
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      PID_angle(theta_mpu);
      dis_D2R = get_distance_D2R();
      delay(10);
      //get_distance_F();
      //delay(10);
    }
    }

    mpu.update();
    theta_mpu = mpu.getAngleZ();
    // ref_mpu -= 10; deleted (edit)
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
  //theta = 0.785;
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
    //Stop_Sumo();%%%%
    //delay(500);%%%%%

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
      //if (abs(desired_angle) < 10 || (dis_L1 < 35 || dis_L2 < 35 ))break;
      
      ///servo_write(pos_R + 10);
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
  /////////////////////////////////////////
  /* if (Serial.available() > 0)
    {
     data = Serial.readStringUntil('#');
    }
  */
  
  //////////////////////////////////////////////////////////
  
  Run_Sumo(speed);
    if (info == "R" || info == "RR") {

      //follow_G_Counter();
      follow_R_Counter();
  
    }
    if (info == "RG") {
      avoid_RG(13);
    }
   

  detected_first = 0;
  detected_ser = 'N';
  ///////////////////////////////REEEEEEEEEEEEEEEEEEEEEEEEEED
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
 // Real_dis_R = Distance(dis_R1, dis_R2);
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
  while (abs(theta_mpu - theta) >= 2)//edit$$$$$########
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
  //Stop_Sumo();%%%
  //delay(500);%%%


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
    ///servo_write(pos_R + 10);
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    PID_angle(theta_mpu);
    dis_D2L = get_distance_D2L();
    delay(10);
  }

  //Stop_Sumo();%%%
  //delay(1000);%%%
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

/*void G_Corner(double dis = 50)
  {

  sensor();
  Run_Sumo(speed);
  // Real distance & theta deleted (edit)
  theta = 45;
  mpu.update();
  theta_mpu = mpu.getAngleZ();
  ref_mpu = theta_mpu;
  angle = angle * 180 / pi;
  theta += angle;
  theta -= theta_mpu;
  dis_D2R = get_distance_D2R();

  delay(10);
  while (abs(theta_mpu + theta) >= 2)
  {

    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_R + 40);
  }
  Stop_Sumo();
  delay(110);
  Run_Sumo(speed);

  mpu.update();
  theta_mpu = mpu.getAngleZ();
  // ref_mpu -= 10; deleted (edit)
  while (abs(theta_mpu - ref_mpu - 30) >= 2 )
  {
    dis_D2R = get_distance_D2R();
    delay(10);
    dis_F = get_distance_F();
    delay(10);
    if (dis_D2R > 1 && dis_D2R < 5 ) break;
    if (dis_F > 1 && dis_F < 5 ) break;
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    delay(10);
    servo_write(pos_L - 40);
  }
  Stop_Sumo();
  delay(110);
  Run_Sumo(speed);


  theta = 0;
  angle = 0;


  }*/
void avoid_G(double dis = 15) {
  if (Serial.available() > 0)
  {
    data = Serial.readStringUntil('#');
  }
  sensor();
  Run_Sumo(speed);
  if (Real_dis_L > 15 ) /////////@
  {
    Real_dis_R = Distance(dis_R1, dis_R2);
    //theta = 0.785; deleted (edit)
    if (counter_angle_Right == 1 || (counter_angle_Right == 0 && counter_angle_Left == 0) )
    {
      theta = abs(atan(1.5 * (100 - Real_dis_R - 10) / 60 ));
      angle = angle_cal(dis_R1,dis_R2);
    }
    if (counter_angle_Left == 1)
    {
      theta = abs(atan(1.5 * (Real_dis_L - 10) / 60 ));
      angle = angle_cal(dis_L1,dis_L2);
    }
    if (theta < 0)
      theta = 0;
    theta = theta * 180 / pi;
    if(theta > 40) theta = 40;
    desired_angle = theta;
    angle = angle * 180 / pi;
    //angle = 0 ;
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    ref_mpu = theta_mpu;
    theta -= angle;
    theta += theta_mpu;
    dis_D2L = get_distance_D2L();
    while (abs(theta_mpu - theta) >= 2 && (dis_D2L > dis || dis_D2L<1))//
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
    //stop summo & delay 500 deleted (edit)

    dis_D2L = get_distance_D2L();
    delay(10);
    //run sumo deleted
    mpu.update();
    reference = mpu.getAngleZ();
    servo_write(pos_L);
   if (abs(desired_angle) > 5 )
    {
    while (dis_D2L >= dis || dis_D2L<1)
    {
      // if(counter_angle_Right == 1 && dis_BR > 65)break; $$
      ///// $ no condition here
      sensor();
      if (Serial.available() > 0)
      {
        data = Serial.readStringUntil('#');
      }
     // if (abs(desired_angle) < 10 || (dis_L1 < 20 || dis_L2 < 20 ) )break; //
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      PID_angle(theta_mpu);
      dis_D2L = get_distance_D2L();
      delay(10);
    }
    }
    

//    Stop_Sumo();//%%%
//    delay(200);//%%%
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

  }

  servo_write(pos_R);
  follow_G_Counter();
  theta = 0;
  angle = 0;

}
void avoid_G_cw(String info = "N" ,double dis = 13)
{
  sensor();
  Run_Sumo(speed);
  Real_dis_L = Distance(dis_L1, dis_L2);
  if (Real_dis_L > 15 ) /////////@
  {
    //theta = 0.785; deleted (edit)
    theta = abs(atan(2 * (Real_dis_L - 10) / 60 ));
    if (theta < 0)
      theta = 0;
    theta = theta * 180 / pi;
    if(theta > 40) theta = 40;
    desired_angle = theta;
    mpu.update();
    theta_mpu = mpu.getAngleZ();
    ref_mpu = theta_mpu;
    theta += theta_mpu;
    dis_D2L = get_distance_D2L();
    while (abs(theta_mpu - theta) >= 2 && dis_D2L > dis )
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
    //stop summo & delay 500 deleted (edit)


    dis_D2L = get_distance_D2L();
    delay(10);
    //run sumo deleted
    mpu.update();
    reference = mpu.getAngleZ();
    servo_write(pos_L);
    if (abs(desired_angle) > 5)
    {
    while (dis_D2L >= dis)
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
    

    //Stop_Sumo();%%%
    //delay(200);%%%
    Run_Sumo(speed);

    mpu.update();
    theta_mpu = mpu.getAngleZ();

    while (abs(theta_mpu - ref_mpu) >= 2 )
    {
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      delay(10);
      servo_write(pos_R + 40);
    }
  }
  follow_G_cw();
  theta = 0;
  angle = 0;

 

  Run_Sumo(speed);
 
    if (info == "G" || info == "GG") {

      follow_G_GN();

    }
   
    if (info == "GR") {

      avoid_GR();
    }
  
 
  detected_first = 0;
  detected_ser = 'N';
}
void avoid_G_ccw( String info="N", double dis = 59) {// TestV3 63
  if (Serial.available() > 0)
  {
    data = Serial.readStringUntil('#');
  }

  sensor();
  Run_Sumo(speed);
  //  theta = abs(atan(2 * (100 - Real_dis_R - 10) / 50));
  //   theta = abs(atan(2 * (Real_dis_R - 15) / 60)); //(eeddiitt)

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
  //Stop_Sumo();%%%
  //delay(500);%%%
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
  // angle = angle_cal(dis_R1, dis_R2) * 180 / pi; (edit)
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
  //Stop_Sumo();%%%
  //delay(500);%%%
  

  theta = 0;
  angle = 0;

  //////////////////////////////////////////////NEW CODE TODAY IS HERE, I commented the reverse move ////////////////////////////////////////////
 

  Run_Sumo(40);
  
    if (info == "G" || info == "GG") {
      follow_G_GN();
    }
   
    if (info == "GR") {
      avoid_GR();
    }
  
 
  detected_first = 0;
  detected_ser = 'N';
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
void Turn_L_Back() {
turnL:
  if (!last_turn_to_L)
  { 
    counter_angle_Right = 1;
    counter_1 = counter_1 + 1;
    if (counter_1 == 9 && detected_cube == 'R' && !Ana_Last && !start_G)
    {
      Ana_Last = true;
      turn_last_to_cw();
      //Stop_Sumo();%%%

      ///////////////////////////////////////////
      //delay(1000);%%%
      counter_angle_Left = 1;
      counter_angle_Right = 0;
      counter_1 = counter_1 - 1;
      last_turn_to_R = true;
      Turn_R_Back();
      return ;

    }

    Run_Sumo(speed);

    if (detected_cube == 'G') {
   
      /////////$£
      sensor();
      while (f > 30) {
        sensor();
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        angle_Right(fr, br);
      }

    }////////////////////////////////////////////////////////////////////////////////////////////////////////LAST ADDDDDD

    if (detected_cube == 'N' || detected_cube == 'G') {
      if (detected_cube == 'N')
      {
        servo_write(pos_R);
   
        
        /////////$£
        while (true) {
          if (Serial.available() > 0) {
            data = Serial.readStringUntil('#');
          }
          sensor();
          if (f < 42 && detected_cube == 'N')break;
          // if(f < 20 && detected_cube == 'G')break;
//          if (Real_dis_R < 10)
//          {
//            P_distanceR(Real_dis_R, 10);
//          }
//          else if (Real_dis_R > 70)
//          {
//            P_distanceR(Real_dis_R, 70);
//          }
//          else
//          {
            angle_Right(fr, br);
            Run_Sumo(speed);

       //   }
        }
         Run_Sumo(speed);
        
      }
      /////////////

      Real_dis_R = Distance(fr, br);//$£ replaced with means
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
      while (abs(theta_mpu - theta) >= 2 )
      {
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }

        dis_D2R = get_distance_D2R();
        f = get_distance_F();
        delay(10);
        mpu.update();
        if (dis_D2R > 1 && dis_D2R < 7)break;
        if (f > 1 && f < 7)break;
        theta_mpu = mpu.getAngleZ();
        delay(10);
        servo_write(pos_R - 40);
      }
      Stop_Sumo();
      delay(500);

      ////////////
      theta = 70;
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      theta += ref_mpu;
       sensor();
      Back_Sumo(60);///// ^&^
      while ((theta_mpu - theta) <= 0)
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
      /////////$£
      Back_Sumo(60);///// ^&^
      while (b > 16 || b < 1) {
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        sensor();
        angle_Right(dis_R2, dis_R1);
      }
    }

    if (detected_cube == 'R') {
      servo_write(pos_R);
      /////////$£
      
      sensor();
      /////////$£
      Run_Sumo(speed);
      while (dis_F > 60) {
        sensor();
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        angle_Right(fr, br);

      }
      //Stop_Sumo();%%%
      //delay(500);%%%
      Run_Sumo(60);
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
      while (dis_B > 16 || b < 1) {
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        sensor();
        angle_Right(dis_R2, dis_R1);
              Back_Sumo(60);///// ^&^
      }
      // Run_Sumo(speed);
    }
  
  }
  Stop_Sumo();
  delay(200);
      Serial.println("L");  
      delay(200);
           while (Serial.available() > 0)
            {
             
                 ser = Serial.readStringUntil('#');
              if(ser == "RR" || ser == "R" || ser == "RG" || ser == "GR" || ser == "GG" || ser == "G" )
                {
                  data = ser;
               
                }
            }
//  for (int j = 0; j < 500; j++)
//    {
//      Stop_Sumo();
//      if (Serial.available() > 0)
//      {
//        data = Serial.readStringUntil('#');
//
//      }
//      else
//      {
//        break;
//      }
//      delay(1);
//    }

  last_turn_to_L = false ;
  counter_stop = counter_stop + 1;

  sensor();
  Run_Sumo(speed);
  servo_write(pos_R);

    //read_serial();
  
    if (data == "G" || data == "GR" || data == "GG") {

     
        avoid_G_ccw(data);
        goto turnL;
     


    }
    else if (data == "R" || data == "RG" || data == "RR") {
     
        avoid_R_ccw(data);
        
        goto turnL;
      
    
    }
  
  //Stop_Sumo();%%%
  //delay(1000);%%%

  detected_first = 0;
  detected_cube = 'N';
  error = 0;// TestV3
  correction_it = 0;// TestV3
  mpu.update();// TestV3
  reference = mpu.getAngleZ();// TestV3
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
      //Stop_Sumo();%%%
      ///////////////////////////////////////////
      //delay(1000);%%%
      counter_angle_Left = 0;
      counter_angle_Right = 1;
      last_turn_to_L = true;
      Turn_L_Back();
      return ;


    }
    Run_Sumo(speed);

    if (detected_cube == 'R') {
      // G_Corner();
    
      sensor();
      /////////$£
      while (f > 25) {
        sensor();
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
        }
        angle_Left(fl, bl);
      }

    }////////////////////////////////////////////////////////////////////////////////////////////////////////LAST ADDDDDD

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
          // if(f < 20 && detected_cube == 'G')break;
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
      /////////////


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
      //Stop_Sumo();%%%
      //delay(500);%%%


      ////////////
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
     
      //Stop_Sumo();%%%
      //delay(500);%%%
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
    
//  /    Run_Sumo(speed);
    }
      
    counter_stop = counter_stop + 1;
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
//  for (int j = 0; j < 500; j++)
//    {
//      Stop_Sumo();
//      if (Serial.available() > 0)
//      {
//        data = Serial.readStringUntil('#');
//
//      }
//      else
//      {
//        break;
//      }
//      delay(1);
//    }
  last_turn_to_R = false;
  sensor();
  Run_Sumo(speed);
  servo_write(pos_R);
  
 
    if (data == "G" || data == "GG" || data == "GR") {
      //$$
    
        avoid_G_cw(data);
        goto turnR;
    }
    
    else if (data == "R" || data == "RR" || data == "RG") {
   
        avoid_R_cw(data);
        goto turnR;
    
    }


  detected_first = 0;
  detected_cube = 'N';
  error = 0;// TestV3
  correction_it = 0;// TestV3
  mpu.update();// TestV3
  reference = mpu.getAngleZ();// TestV3
  Run_Sumo(speed);

}

String reading;
void read_serial() {
  for (int ii = 0 ; ii < 2 ; ii++) {
    if (Serial.available() > 0)
    { Run_Sumo(speed);
      reading = Serial.readStringUntil('#');

      if (reading == "g" || reading == "r" || reading == "G" || reading == "R" || reading == "N" )
        data = reading;
      delay(2);
    }

  }
}
void debug()
{
  Stop_Sumo();
  delay(500);
  Run_Sumo(speed);
  delay(400);
  Stop_Sumo();
  delay(500);
  Run_Sumo(speed);

}
bool safety()
{

  dis_D2L = get_distance_D2L();
  delay(10);
  f = get_distance_F();
  delay(10);
  if (dis_D2L > 1 && dis_D2L < 5)return false;
  if (f > 1 && f < 5)return false;
  return true;
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
//  while (start == 0)
//  {
//    start = digitalRead(Switch);
//    if (Serial.available() > 0) {
//       debug();
//      data = Serial.readStringUntil('#');
//    }
//    delay(1);
//  }
//  if(data == "R")
//  {
//    debug();
//   Stop_Sumo();
//   delay(21312312);
//  }
  myservo.attach (7);
}
bool loop_start_pos = 1;
void loop() {
  if(counter_1 == 0)
  {
    while (start == 0)
  {
    start = digitalRead(Switch);
    if (Serial.available() > 0) {
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
  if (loop_start_pos) { //to set the servo motor at its setpoint
    servo_write(pos_R);
    loop_start_pos = 0;
    mpu.update();
    reference = mpu.getAngleZ();
  }
  //// read_serial(); //reading what the python code is sending
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('#');
    Run_Sumo(speed);
  }
 
  //deteceted serial deleted (edit)

  if (data == "R" || data == "RG" || data == "RR") {
   
    avoid_R();
  }
  else if (data == "G" || data == "GR" || data == "GG") {
     start_G = true;
    avoid_G();
  }

  else {
    if (Real_dis_R < 100 && Real_dis_L < 100) {
    
      followmiddle_straight();
    }
  }
  if (counter_angle_Left == 0 && counter_angle_Right == 0 ) {
    if (fl > 100 || bl > 100) {

      Turn_L_Back();

    }
    if (fr > 100 || br > 100) {
      Turn_R_Back();

    }
  }
  if (counter_angle_Right == 1) {
    if ((fl > 100 || bl > 100)&&b>190&&detected_cube=='N') {// TestV3
      Turn_L_Back();
    }
    else if ((fl > 100 || bl > 100)&&(detected_cube == 'R' || detected_cube == 'G')) {// TestV3
      Turn_L_Back();
    }
  }
  if (counter_angle_Left == 1) {
    if ((fr > 100 || br > 100)&&b>190&&detected_cube == 'N') {// TestV3
      Turn_R_Back();
    }
    else if ((fr > 100 || br > 100)&&(detected_cube == 'R' || detected_cube == 'G')) {// TestV3
      Turn_R_Back();
    }
  }
}
