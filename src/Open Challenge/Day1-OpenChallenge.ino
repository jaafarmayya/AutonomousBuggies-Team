////////////////////////////////////////////////////////////////////////////// Including Required Libraries////////////////////////////////////////////////////////////////////////////// 
#include <ArduinoQueue.h>
#include <NewPing.h>
#include <Servo.h>
#include<math.h>
#include <MPU6050_light.h>

////////////////////////////////////////////////////////////////////////////// Here is variables definition and Initialization //////////////////////////////////////////////////////////////////////////////

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

#define B_trigger 13
#define B_echo 12

#define MAX_DISTANCE 300

#define R_EN 10
#define L_EN 8
#define R_PWM 11
#define L_PWM 9

MPU6050 mpu(Wire);
double theta=0, theta_mpu=0 , ref_mpu=0;
String data;
bool Set_Servo = 1 , Switch = 50 ;
int start = 0;
int pos_R = 90 , pos_L =  85,  pos_L_Back = 95, turn_pos = 45, turn_pos2 = 45;
int Delay_Turn_Left = 1000, Delay_Turn_Right = 1150;
int delay_towide = 1200 , delay_tonarrow = 1150;
int counter_angle_Right = 0, counter_angle_Left = 0, counter = 0;
int dis_narrow_F = 100; 
double Position_L = pos_R - turn_pos , Position_R = pos_R + turn_pos2;
double position_angle = 0;
double distance = 0, real_distance = 0 , Real_dis_L = 0, Real_dis_R = 0, dis_L1 = 0, dis_L2 = 0, dis_R1 = 0, dis_R2 = 0, dis_F = 0, dis_B =0;
double angle = 0, Angle = 0, Aangle = 0;
double conskp = 0.65, kp = 1 , kpl = 1 , kpr = 1,conskp_Notstart = 1.7;
double Setpoint = 10;
double highest_angle = 0.27;
double pos_distance;
double jbes = 0;
double duration_F = 0, duration_L1 = 0, duration_L2 = 0, duration_R1 = 0, duration_R2 = 0, duration_B = 0;
int spd_now = 60;
int spd_turn_now = 60;
int spd_before_turn = 50;
int spd_before_turn_back = 60;
double max_ang = max(pos_R,pos_L) + 35;
double min_ang = min(pos_R,pos_L) - 35;

double currentTime_angle = 0, dtc = 0 , lastTime_angle = 0 , current_angle = 0 , error_c = 0, output_c = 0  , last_angle_error = 0;
double kpc = 1, kdc = 1;
double reference = 0;
int reference_counter = 5; 
double angle_mean = 0;


Servo myservo;
NewPing sonar_L1(TRIGGER_PIN_L1, ECHO_PIN_L1, MAX_DISTANCE);
NewPing sonar_L2(TRIGGER_PIN_L2, ECHO_PIN_L2, MAX_DISTANCE);
NewPing sonar_R1(TRIGGER_PIN_R1, ECHO_PIN_R1, MAX_DISTANCE);
NewPing sonar_R2(TRIGGER_PIN_R2, ECHO_PIN_R2, MAX_DISTANCE);
NewPing sonar_F(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);
NewPing sonar_B(B_trigger, B_echo, MAX_DISTANCE);

int Filter_Size = 10;
double temp = 0;
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
////////////////////////////////////////////////////////////////////////////// End of variables definition and Initialization //////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////// Functions definition and declaration //////////////////////////////////////////////////////////////////////////////


void Stop_Sumo();
void Back_Sumo(int pwm);
void Run_Sumo(int pwm);

void myservo_write(double ang) {// Move the servo motor "ang" degrees, set upper and lower limit for the angle.
  if (ang > max_ang) {
    ang = max_ang;
  }
  else if (angle < min_ang) {
    ang = min_ang;
  }
  myservo.write(ang);
}
////////////////////////////////////////////////////////////////////////////// Sensors reading functions are here //////////////////////////////////////////////////////////////////////////////

// - "Filter": This function the current reading, readings queue and last reading as input,
//    it take the average of the last 5 readings and reutrn it as the sensor reading.
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
    temp = Means.dequeue();
    last_mean -= (double(temp / Filter_Size));
    last_mean += (double(read / Filter_Size));
    Means.enqueue(read);
  }
  return last_mean;
}

double get_distance_F ()
{
  duration_F = sonar_F.ping();
  f = (duration_F / 2) * 0.0343;// Calculate the distance based on utlra sonic wave speed and time taken by the wave.
  mean_f = Filter(f, mean_f, Means_F);// Take the filtered value
  return mean_f;
}

double get_distance_B() {
  duration_B = sonar_B.ping();
  b = ((duration_B / 2) * 0.0343);
  mean_b = Filter(b, mean_b, Means_B);
  return mean_b;
}

double get_distance_L1 ()
{
  duration_L1 = sonar_L1.ping();
  fl = ((duration_L1 / 2) * 0.0343) - 4 ;
  
  mean_fl = Filter(fl, mean_fl, Means_FL);
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
  fr = ((duration_R1 / 2) * 0.0343)-4;
  mean_fr = Filter(fr, mean_fr, Means_FR);
  return mean_fr;
}

double get_distance_R2()
{
  duration_R2 = sonar_R2.ping();
  br = ((duration_R2 / 2) * 0.0343) - 4;
  mean_br = Filter(br, mean_br, Means_BR);
  
  return mean_br ;
}



////////////////////////////////////////////////////////////////////////////// End of sensors reading functions //////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////// Distance and angle calculation functions are here //////////////////////////////////////////////////////////////////////////////
/*
The provided functions below are responsible for all mathematical calculations (angle, prependicular distance,filtering):
- "Distance": Takes distance sensors readings as input, and returns the prependicular distance.
- "angle_cal": Takes sensor readings as input, and returns the angle between the robot and the wall.
- "reset_means": Resets the sensors readings queues,
   this is necessary when moving from corner section to straight forward section
   (Values may some time to be updated due to the nature of filtering, thus, resetting them is beneficial).

*/
double angle_cal (double dis_1 , double dis_2)
{
  Angle =  atan ((dis_1 - dis_2 ) / 7.4 );
  if (Angle > highest_angle)
  {
    Angle =  highest_angle;
  }
  if (Angle < ((-1)*highest_angle))
  {
    Angle = (-1) * highest_angle;
  }
  return Angle;
}
double Distance(double L1 , double L2)
{
  distance = (L1 + L2) / 2;
  angle = angle_cal (L1, L2);
  real_distance = distance * cos(angle);
  return real_distance;
}

void sensor() {
 
  dis_L1 = get_distance_L1();
  delay(10);
  dis_R2 = get_distance_R2();

  delay(10);

  dis_R1 = get_distance_R1();
  delay(10);
  dis_L2 = get_distance_L2();
   
  delay(10);
  dis_B = get_distance_B();
  dis_F = get_distance_F();
  delay(10);
  Real_dis_L = Distance(fl, bl);
  Real_dis_R = Distance(fr, br);
}
void reset_means()
{
  while(!Means_B.isEmpty())
  {
    Means_B.dequeue();
    Means_BL.dequeue();
    Means_BR.dequeue();
    Means_FL.dequeue();
    Means_FR.dequeue();
    Means_F.dequeue();
    
  }
}



////////////////////////////////////////////////////////////////////////////// End of distance and angle calculation functions //////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////// Motion in protected area and unprotected area functions are here //////////////////////////////////////////////////////////////////////////////

/*
 The functions that drive the motion in the protected and unprotected areas are presented here:
- ("angle_Left","angle_Left_Back","angle_Right"): These functions are repsonsible for PID following with ki and kd constants equal to zero,
  they use the angle calculated using sensors readings. These functions differ in the setpoint and used sensors.

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

/*- ("P_distanceL","P_distanceR"): These functions work by the same manner with a higher kp constant and drive the motion in the unprotected .
    The function has one parameter requireed is the prependicular distance, The function apply a P controller 
    to keep the robot away enough from the wall for a Setpoint.
    */

void P_distanceL (double real_distance)
{ 
  if (counter != 0){
    conskp = conskp_Notstart;
  }

  pos_distance = conskp * (real_distance - Setpoint);
  jbes = pos_L - pos_distance;
  myservo_write(jbes);
}
void P_distanceR (double real_distance)
{
   if (counter != 0){
    conskp = conskp_Notstart;
  }
  pos_distance = conskp * (real_distance - Setpoint);
  jbes = pos_R + pos_distance;
  myservo_write(jbes);
}

  /*
  -"PID_angle"/"PID_angle_Back": These functions are repsonsible for PID following with ki constant equal to zero,
   they use the parameter current_angle from the gyroscope sensor yaw reading with the PD controller to keep the robot 
   orientation exactly to the reference angle (prependicular to the wall)
   */

void PID_angle(double current_angle) {
  currentTime_angle = (micros()) * (1e-3);
  dtc =  currentTime_angle - lastTime_angle;
  lastTime_angle = currentTime_angle;
  error_c = current_angle - reference;
  output_c = kpc * error_c + ((error_c - last_angle_error) / dtc) * kdc ;
  myservo_write(output_c + pos_R);
  last_angle_error = error_c;
}
void Back_PID_angle(double current_angle) {
  currentTime_angle = (micros()) * (1e-3);
  dtc =  currentTime_angle - lastTime_angle;
  lastTime_angle = currentTime_angle;
  error_c = current_angle - reference;
  output_c = kpc * error_c + ((error_c - last_angle_error) / dtc) * kdc ;
  myservo_write( pos_R + output_c);
  last_angle_error = error_c;
}

////////////////////////////////////////////////////////////////////////////// End of motion in protected area and unprotected area functions are here //////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////// DC Motor controlling functions are here //////////////////////////////////////////////////////////////////////////////

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


////////////////////////////////////////////////////////////////////////////// end of DC motor controlling functions //////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////Turn right and turn left functions are here //////////////////////////////////////////////////////////////////////////////
/*
Each function works as the following:
1- If the distance from the front wall is less then 40cm, then the robot's turn is impossible and the robot needs to move backwards.
2- The robot turns 90 degrees (this value is adjusted based on calibration and testing).
3- The program resets the sensors means queues.

*/ 
void turn_right_gyro(){
      myservo_write(pos_R);
      sensor();
      if(f < 40  )
      {
        Stop_Sumo();
      while(f < 40)
      {
       mpu.update();
       Back_PID_angle(mpu.getAngleZ());
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
      Run_Sumo(spd_turn_now);
      }
      theta = 75 - angle_cal(fl,bl)*180/acos(-1);
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta = ref_mpu - theta;
       get_distance_F();
      delay(10);
      while (true)
      {
        if((theta_mpu - theta) <= 0 )break;
        get_distance_F();
        mpu.update();
        theta_mpu = mpu.getAngleZ();
        delay(30);
        myservo_write(pos_R + 40);
        
      }
      myservo_write(pos_R);
      counter+=1;
      counter_angle_Left = 1;

      reset_means();
      mpu.update();
      reference = mpu.getAngleZ(); 
      reference_counter = 5;
}

void turn_left_gyro(){
      myservo_write(pos_L);
      sensor();
      if(f < 40)
      {
        Stop_Sumo();
      while(f < 40)
      {
        angle_Right(br,fr);
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
      Run_Sumo(spd_turn_now);
      }
      theta = 83 - angle_cal(fr,br)*180/acos(-1);
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta = ref_mpu + theta;
      get_distance_F();
      delay(10);
     while(true)
      {
        if((theta_mpu - theta) >= 0)break;
        mpu.update();
        theta_mpu = mpu.getAngleZ();
        delay(30);
        myservo_write(pos_L- 40);
        get_distance_F();
        
      }
      myservo_write(pos_L);
      counter+=1;
      counter_angle_Right = 1;

      reset_means();
      mpu.update();
      reference = mpu.getAngleZ();
      reference_counter = 5;
}
//////////////////////////////////////////////////////////////////////////////End of turn right and turn left functions //////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////End Functions definition and declaration //////////////////////////////////////////////////////////////////////////////

void setup() {
  // Set Pin Mode for (Sensors, Switch, Servo Motor)
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
  myservo.attach (7);

  //Initializing I2C Connection protocol with MPU6050 Sensor
   Wire.begin();
  byte status = mpu.begin();
  while (status != 0) {
    Serial.println("Satus!=0");
  }

  while (start == 0)// Start switch
 {
   start = digitalRead(Switch);
   delay(1);
 }


  mpu.calcOffsets();
  mpu.update();
  delay(10);
}


void loop() {
  sensor();
  if(Set_Servo)// Reset servo motor's angle at the start of the program
  {
    myservo_write(Position_L);
    Set_Servo = 0 ;
    mpu.update();
    reference = mpu.getAngleZ();
  }
  if (f > 90) {// if the front distance sensor is more than 90 cm -> this means that the robot is in the straight forward section or moving to it from the corner.
    if (Real_dis_L < Setpoint)// If prependicular distance is less than the specified setpoint, then this is Unprotected Area (UPA).
    { 
      Run_Sumo(spd_before_turn);
      P_distanceL(Real_dis_L);
 
    }

    if (Real_dis_R < Setpoint)// If prependicular distance is less than the specified setpoint, then this is UPA.
    {
     Run_Sumo(spd_before_turn);
     P_distanceR(Real_dis_R);

    }

    if (Real_dis_L > Setpoint && Real_dis_R > Setpoint && counter_angle_Left == 1)// The robot is in Protected Area (PA).
    {
      Run_Sumo(spd_now);
    if(reference_counter > 0)
    {
        angle_mean+=angle_cal(fl,bl)*180/acos(-1); 
        reference_counter--;
    }
    if(reference_counter == 0)// Gyroscope refernce correction
    {
      reference += angle_mean/5;
      reference_counter = -1;
      angle_mean = 0;
    }
    mpu.update();
      PID_angle(mpu.getAngleZ());
    }

    if (Real_dis_L > Setpoint && Real_dis_R > Setpoint && counter_angle_Right == 1)// if the direction is CCW
    {
      Run_Sumo(spd_now);
      if(reference_counter > 0)
    {
        angle_mean+=angle_cal(fr,br)*180/acos(-1); 
        reference_counter--;
    }
    if(reference_counter == 0)
    {
     
      reference -= angle_mean/5;
      reference_counter = -1;
      angle_mean = 0;
    }     
      mpu.update();
      PID_angle(mpu.getAngleZ());
    }

    if (Real_dis_L > Setpoint && Real_dis_R > Setpoint && counter_angle_Left == 0 && counter_angle_Right == 0)// If the rotation movement isn't identified yet.
    { 
      Run_Sumo(spd_before_turn);
      
      angle_Left(dis_L1, dis_L2);
    }
    if (b < 100 && (fr > 100 || br > 100)) {
      Run_Sumo(60);
      while (b < 100 && (fr > 100 || br > 100)) {
        sensor();
        mpu.update();
        PID_angle(mpu.getAngleZ());
      }
     }
     if (b < 100 && (fl > 100 || bl > 100)) {
      
      Run_Sumo(60);
      while (b < 100 && (fl > 100 || bl > 100)) {
        sensor();
         mpu.update();
         PID_angle(mpu.getAngleZ());
        
      }
      
     }
 }
  if (f < 90 && f>0) {// In case the robot becomes closer to the front wall, then lower the speed
  
    Run_Sumo(spd_before_turn);
    if (Real_dis_L < Setpoint)
    {
      P_distanceL(Real_dis_L);
    }

    if (Real_dis_R < Setpoint)
    {
      P_distanceR(Real_dis_R);
    }

    if (Real_dis_L > Setpoint && Real_dis_R > Setpoint && counter_angle_Left == 1)
    {

      mpu.update();
      PID_angle(mpu.getAngleZ());
    }

    if (Real_dis_L > Setpoint && Real_dis_R > Setpoint && counter_angle_Right == 1)
    {
      mpu.update();
      PID_angle(mpu.getAngleZ());

    }

    if (Real_dis_L > Setpoint && Real_dis_R > Setpoint && counter_angle_Left == 0 && counter_angle_Right == 0)
    {
       mpu.update();
      PID_angle(mpu.getAngleZ());

    }


    if (counter_angle_Right == 1){// if the direction is CCW
        /* if one of the sensors on the left side reads a big distance (>100cm), 
        or the front distance sensor reads small distance and back distance sensor reading is big 
        (The robot is in the end of the current section), then trun left.
        */
        if ( (fl >  100 || bl > 100)|| (f<50 && f>0 && b>210) ) {
         Run_Sumo(spd_turn_now);
         turn_left_gyro();
             
        }
 
    }

    if (counter_angle_Left == 1){// if the direction is CW
        /* if one of the sensors on the right side reads a big distance (>100cm),
           or the front distance sensor reads small distance and back distance sensor reading is big 
           (The robot is in the end of the current section), then trun right.
        */

        if (fr >  100 || br > 100 || (f<50 && f>0 && b>210) ) { 
          
          Run_Sumo(spd_turn_now);
          turn_right_gyro();
         
        }
     
    }

    if (counter_angle_Left == 0 && counter_angle_Right == 0){// if the direction isn't identified

        if (fl >  100 || bl > 100 ){  // if one of the sensors on the left side reads a big distance (>100cm), then trun left.

           
           turn_left_gyro();

        }
        if (fr >  100 || br > 100 ){ // if one of the sensors on the right side reads a big distance (>100cm), then trun left.
         
           turn_right_gyro();
        }
    }
  }
  if(counter>=12)// If it is the last section
  {
    sensor();
    while(f > 200 || b < 110)// Move until the robot becomes enirely inside the section then stop
    {
    if(counter_angle_Left == 1)
    {
      if (Real_dis_L < Setpoint) {
          P_distanceL(Real_dis_L);
        } 
        else if (Real_dis_L > Setpoint) {
           mpu.update();
           PID_angle(mpu.getAngleZ());
        }
    }
    else
    {
      if (Real_dis_R < Setpoint) {
          P_distanceR(Real_dis_R);
        } 
        else if (Real_dis_R > Setpoint) {
           mpu.update();
           PID_angle(mpu.getAngleZ());
        }
    }
    sensor();
  }
    Stop_Sumo();
      delay(100000000000);
}
}
