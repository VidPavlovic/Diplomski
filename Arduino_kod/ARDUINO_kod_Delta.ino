#include "AccelStepper.h"
#include <math.h>
#define mode_driver 1
#define step1 32
#define dir1 33
#define step2 25
#define dir2 26
#define step3 27
#define dir3 14
#define home_sw_1 5
#define home_sw_2 4
#define home_sw_3 15
#define valve_pin 21

struct position_current{
  float X;
  float Y;
  float Z;
  long q1s;
  long q2s;
  long q3s;
  float q1d;
  float q2d;
  float q3d;
};
position_current position_curr = {0,0,0,0,0,0,0,0,0};

int homing_done = 0;
int initial_mov = 0;
long initial_homing_1 = -1; long initial_homing_2 = -1; long initial_homing_3 = -1;
String readString;
String inverse_result = "";

float wb = 141.0;
float wp = 26.25;
float L = 230;
float l = 544;

float up = wp * 2;
float ub = wb * 2;
float sb = (6 / sqrt(3)) * wb;
float sp = (6 / sqrt(3)) * wp;
float a = wb - up;
float b = sp/2 - (sqrt(3)/2) * wb;
float c = wp - 0.5 * wb;
int steps_per_revolution = 400;
float reduction = (13 + (212/289))*1.76;
//float reduction = (13.73)*1.76;

int min_angle = -60;
int max_angle = 89;
float beta_max = 20;
float z_lim = -695.0;

long offset_limit_1 = (58.0/360.0)*steps_per_revolution*reduction;
long offset_limit_2 = (58.0/360.0)*steps_per_revolution*reduction;
long offset_limit_3 = (58.0/360.0)*steps_per_revolution*reduction;
long initial_steps = (50/360.0)*steps_per_revolution*reduction;

AccelStepper stepper1 = AccelStepper(mode_driver, step1, dir1);
AccelStepper stepper2 = AccelStepper(mode_driver, step2, dir2);
AccelStepper stepper3 = AccelStepper(mode_driver, step3, dir3);

void setup() {
  Serial.begin(115200);
  pinMode(home_sw_1, INPUT_PULLUP);
  pinMode(home_sw_2, INPUT_PULLUP);
  pinMode(home_sw_3, INPUT_PULLUP);
  pinMode(valve_pin, OUTPUT);
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(5000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(5000);
  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(5000);
  digitalWrite(valve_pin, HIGH);
}

void loop() {
  check_serial();
}


void homing(){
  if (homing_done == 1){
    Serial.println("homing_already_done*");
  }
  if (homing_done == 0){
    int switch_state1 = 0;
    int switch_state2 = 0;
    int switch_state3 = 0;

    stepper1.setMaxSpeed(400);
    stepper2.setMaxSpeed(400);
    stepper3.setMaxSpeed(400);

    stepper1.moveTo(initial_steps);
    stepper2.moveTo(initial_steps);
    stepper3.moveTo(initial_steps);

    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0){
        if (stepper1.distanceToGo() != 0){
          stepper1.run();
        }
        if (stepper2.distanceToGo() != 0){
          stepper2.run();
        }
        if (stepper3.distanceToGo() != 0){
          stepper3.run();
        }
      }

    Serial.println("initial_done*");
    delay(2000);

    while (!digitalRead(home_sw_1)) {
      stepper1.moveTo(initial_homing_1);
      initial_homing_1++;
      stepper1.run();
    }
    stepper1.setCurrentPosition(0);
    initial_homing_1=1;
    Serial.println("limit_1*");
    switch_state1 = 1;
    delay(50);

    while (!digitalRead(home_sw_2)) {
      stepper2.moveTo(initial_homing_2);
      initial_homing_2++;
      stepper2.run();
    }

    stepper2.setCurrentPosition(0);
    initial_homing_2=1;
    Serial.println("limit_2*");
    switch_state2 = 1;
    delay(50);

    while (!digitalRead(home_sw_3)) {
      stepper3.moveTo(initial_homing_3);
      initial_homing_3++;
      stepper3.run();
    }

    stepper3.setCurrentPosition(0);
    initial_homing_3=1;
    Serial.println("limit_3*");
    switch_state3 = 1;
    delay(1000);

    if (switch_state1 == 1 && switch_state2 == 1 && switch_state3 == 1){
      Serial.println("limiti_aktivirani*");
      stepper1.setMaxSpeed(3000);
      stepper2.setMaxSpeed(3000);
      stepper3.setMaxSpeed(3000);

      stepper1.moveTo(-offset_limit_1);
      stepper2.moveTo(-offset_limit_2);
      stepper3.moveTo(-offset_limit_3);

      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0){
        if (stepper1.distanceToGo() != 0){
          stepper1.run();
        }
        if (stepper2.distanceToGo() != 0){
          stepper2.run();
        }
        if (stepper3.distanceToGo() != 0){
          stepper3.run();
        }
      }
    }

    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);

    float homing_result[4] = {0};
    direct(0,0,0,0,homing_result);
    position_curr.X = homing_result[0];
    position_curr.Y = homing_result[1];
    position_curr.Z = homing_result[2];
    position_curr.q1s = 0;
    position_curr.q2s = 0;
    position_curr.q3s = 0;
    position_curr.q1d = 0.0;
    position_curr.q2d = 0.0;
    position_curr.q3d = 0.0;

    Serial.println("homing_done," + String(position_curr.q1d) + "," + String(position_curr.q2d) + "," + String(position_curr.q3d) + "," + String(position_curr.X) + ","+ String(position_curr.Y) + ","+ String(position_curr.Z) + "*");
    homing_done = 1;
  }

}

void check_serial(){
  if (Serial.available())  {
    char c = Serial.read();
    if (c == '*') {     
      int ind1 = readString.indexOf(',');
      String action_message = readString.substring(0, ind1);
      action_message.trim();
      int ind2 = readString.indexOf(',', ind1+1 );
      String x_message = readString.substring(ind1+1, ind2);
      int ind3 = readString.indexOf(',', ind2+1 );
      String y_message = readString.substring(ind2+1, ind3);
      int ind4 = readString.indexOf(',', ind3+1 );
      String z_message = readString.substring(ind3+1, ind4);
      int ind5 = readString.indexOf(',', ind4+1 );
      String tool_off = readString.substring(ind4+1, ind5);
      int ind6 = readString.indexOf(',', ind5+1 );
      String speed = readString.substring(ind5+1, ind6);
      int ind7 = readString.indexOf(',', ind6+1 );
      String acc = readString.substring(ind6+1);

      
      float x_req = x_message.toFloat();
      float y_req = y_message.toFloat();
      float z_req = z_message.toFloat();
      float tool_offset = tool_off.toFloat();
      float speed_req = speed.toInt();
      long acc_req = acc.toInt();




      if (action_message == "homing"){
        homing();
      }
      if (action_message == "joint" || action_message == "joint_t" || action_message == "linear") {
        if (z_req != 0 && tool_offset != 0 && speed_req != 0 && acc_req != 0) {
          if (action_message == "joint"){
            if (homing_done == 1){
              prepare_joint(x_req, y_req, z_req, tool_offset, speed_req, acc_req);
            }
            if (homing_done == 0){
              Serial.println("Homing_not_done*");
            }
          }
          if (action_message == "joint_t"){
            if (homing_done == 1){
              prepare_joint_t(x_req, y_req, z_req, tool_offset, acc_req);
            }
            if (homing_done == 0){
              Serial.println("Homing_not_done*");
            }
          }
          if (action_message == "linear") {
            if (homing_done == 1) {
              prepare_linear(x_req,y_req,z_req,tool_offset,speed_req,acc_req);
            }  
            if (homing_done == 0){
              Serial.println("Homing_not_done*");
            }
          }
        }
        else{
          Serial.println("Message incomplete*");
        }
      }

      if (action_message == "valve_on"){
        digitalWrite(valve_pin, LOW);
        Serial.println("valve_on*");
      }
      if (action_message == "valve_off"){
        digitalWrite(valve_pin, HIGH);
        Serial.println("valve_off*");
      }


      readString="";
      action_message = "";
      x_message="";
      y_message="";
      z_message="";
      //empty char??
      c = '\0';
    }  
    else {     
      readString += c;
    }
  }
}


void joint_movement_max_a(long steps1, long steps2, long steps3, int speed, long acc){

  stepper1.setMaxSpeed(speed);
  stepper1.setAcceleration(acc);
  stepper2.setMaxSpeed(speed);
  stepper2.setAcceleration(acc);
  stepper3.setMaxSpeed(speed);
  stepper3.setAcceleration(acc);
  stepper1.moveTo(-steps1);
  stepper2.moveTo(-steps2);
  stepper3.moveTo(-steps3);
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0){
    if (stepper1.distanceToGo() != 0){
      stepper1.run();
    }
    if (stepper2.distanceToGo() != 0){
      stepper2.run();
    }
    if (stepper3.distanceToGo() != 0){
      stepper3.run();
    }
  }
}

void linear_movement(long steps1, long steps2, long steps3, int speed1, int speed2, int speed3){
  
  stepper1.moveTo(-steps1);
  stepper2.moveTo(-steps2);
  stepper3.moveTo(-steps3);
  stepper1.setMaxSpeed(speed1);
  stepper2.setMaxSpeed(speed2);
  stepper3.setMaxSpeed(speed3);

  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0){
    if (stepper1.distanceToGo() != 0){
      stepper1.runSpeedToPosition();
    }
    if (stepper2.distanceToGo() != 0){
      stepper2.runSpeedToPosition();
    }
    if (stepper3.distanceToGo() != 0){
      stepper3.runSpeedToPosition();
    }
  }
}

void generateWaypointsLinear(float pointA[3], float pointB[3], int speed, int numWaypoints, long q1_arr[], long q2_arr[], long q3_arr[], float *timestep, float tool_offset) {
    float direction[3] = {pointB[0] - pointA[0], pointB[1] - pointA[1], pointB[2] - pointA[2]};
    float distance = sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    for (int i = 0; i < 3; i++) {
        direction[i] /= distance;
    }
    float totalTime = distance / speed;
    float timeInterval = totalTime / (numWaypoints - 1);
    *timestep = timeInterval;

    float waypoints[numWaypoints][3];

    for (int i = 0; i < numWaypoints; i++) {
        float t = timeInterval * i;
        waypoints[i][0] = pointA[0] + direction[0] * (t * speed);
        waypoints[i][1] = pointA[1] + direction[1] * (t * speed);
        waypoints[i][2] = pointA[2] + direction[2] * (t * speed);

        long steps_result_lin[4] = {0};
        inverse(waypoints[i][0], waypoints[i][1], waypoints[i][2], tool_offset, steps_result_lin);
        q1_arr[i] = steps_result_lin[0];
        q2_arr[i] = steps_result_lin[1];
        q3_arr[i] = steps_result_lin[2];
    }
}

void inverse(float x, float y, float z, float tcp_offset, long steps_result[]) {
  z = z + tcp_offset;
  float E1 = 2 * L * (y + a);
  float F1 = 2 * z * L;
  float G1 = x * x + y * y + z * z + a * a + L * L + 2 * y * a - l * l;
  float E2 = -L * (sqrt(3) * (x + b) + y + c);
  float F2 = 2 * z * L;
  float G2 = x * x + y * y + z * z + b * b + c * c + L * L + 2 * (x * b + y * c) - l * l;
  float E3 = L * (sqrt(3) * (x - b) - y - c);
  float F3 = 2 * z * L;
  float G3 = x * x + y * y + z * z + b * b + c * c + L * L + 2 * (-x * b + y * c) - l * l;
  float dis1 = E1 * E1 + F1 * F1 - G1 * G1;
  float dis2 = E2 * E2 + F2 * F2 - G2 * G2;
  float dis3 = E3 * E3 + F3 * F3 - G3 * G3;
  if (dis1 < 0 || dis2 < 0 || dis3 < 0) {
      steps_result[3] = 0;
      return;
  }
  float theta_11 = 2 * atan((-F1 + sqrt(dis1)) / (G1 - E1));
  float theta_12 = 2 * atan((-F1 - sqrt(dis1)) / (G1 - E1));
  float theta_21 = 2 * atan((-F2 + sqrt(dis2)) / (G2 - E2));
  float theta_22 = 2 * atan((-F2 - sqrt(dis2)) / (G2 - E2));
  float theta_31 = 2 * atan((-F3 + sqrt(dis3)) / (G3 - E3));
  float theta_32 = 2 * atan((-F3 - sqrt(dis3)) / (G3 - E3));
  
  float theta1 = 0;
  float theta2 = 0;
  float theta3 = 0;
  if (degrees(theta_11) > min_angle && degrees(theta_11) < max_angle) {
    theta1 = degrees(theta_11);
    theta2 = degrees(theta_21);
    theta3 = degrees(theta_31);
  } 
  else {
    theta1 = degrees(theta_12);
    theta2 = degrees(theta_22);
    theta3 = degrees(theta_32);
  }

  float x01 = x;
  float x02 = x * cos(2 * PI / 3) + y * sin(2 * PI / 3);
  float x03 = x * cos(4 * PI / 3) + y * sin(4 * PI / 3);
  float beta1d = degrees(asin(x01 / l));
  float beta2d = degrees(asin(x02 / l));
  float beta3d = degrees(asin(x03 / l));

  if (beta1d > -beta_max && beta1d < beta_max &&
    beta2d > -beta_max && beta2d < beta_max &&
    beta3d > -beta_max && beta3d < beta_max &&
    theta1 > min_angle && theta1 < max_angle &&
    theta2 > min_angle && theta2 < max_angle &&
    theta3 > min_angle && theta3 < max_angle){
      
      steps_result[0] = round(theta1 * (steps_per_revolution * reduction) / 360);
      steps_result[1] = round(theta2 * (steps_per_revolution * reduction) / 360);
      steps_result[2] = round(theta3 * (steps_per_revolution * reduction) / 360);
      steps_result[3] = 1;
  }
  else {
      steps_result[3] = 0;
  }
}

void direct(float theta1, float theta2, float theta3, float tcp_offset, float position_result[]){
  theta1 = theta1 * (PI / 180.0);
  theta2 = theta2 * (PI / 180.0);
  theta3 = theta3 * (PI / 180.0);
  
  float t = a;
  float y1 = -(t + L * cos(theta1));
  float z1 = -L * sin(theta1);
  float y2 = (t + L * cos(theta2)) * sin(30 * DEG_TO_RAD);
  float x2 = y2 * tan(60 * DEG_TO_RAD);
  float z2 = -L * sin(theta2);
  float y3 = (t + L * cos(theta3)) * sin(30 * DEG_TO_RAD);
  float x3 = -y3 * tan((60 * DEG_TO_RAD));
  float z3 = -L * sin(theta3);

  float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;
  float w1 = sq(y1) + sq(z1);
  float w2 = sq(x2) + sq(y2) + sq(z2);
  float w3 = sq(x3) + sq(y3) + sq(z3);
  float a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;
  float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;
  float A = sq(a1) + sq(a2) + sq(dnm);
  float B = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * sq(dnm));
  float C = sq(b2 - y1 * dnm) + sq(b1) + sq(dnm) * (sq(z1) - sq(l));
  float D = sq(B) - 4 * A * C;
  
  if (D < 0) {
      float x0 = NAN;
      float y0 = NAN;
      float z0 = NAN;
      Serial.println("Greska");
      return;
  }

  float z0 = -0.5 * (B + sqrt(D)) / A;
  float x0 = (a1 * z0 + b1) / dnm;
  float y0 = (a2 * z0 + b2) / dnm;

  position_result[0] = x0;
  position_result[1] = y0;
  position_result[2] = z0 - tcp_offset ;

}

void prepare_joint(float x_req, float y_req, float z_req, float tool_offset, int speed_req, long acc_req){
  long steps_result[4] = {0};
  inverse(x_req, y_req, z_req, tool_offset, steps_result);
  if (steps_result[3] == 1 && z_req > z_lim){
    joint_movement_max_a(steps_result[0],steps_result[1],steps_result[2], speed_req, acc_req);
    position_curr.q1s = steps_result[0];
    position_curr.q2s = steps_result[1];
    position_curr.q3s = steps_result[2];
    position_curr.q1d = (steps_result[0] * 360) / (steps_per_revolution * reduction);
    position_curr.q2d = (steps_result[1] * 360) / (steps_per_revolution * reduction);
    position_curr.q3d = (steps_result[2] * 360) / (steps_per_revolution * reduction);
    float position_result[4] = {0};
    direct(position_curr.q1d,position_curr.q2d,position_curr.q3d, tool_offset, position_result);
    position_curr.X = position_result[0];
    position_curr.Y = position_result[1];
    position_curr.Z = position_result[2];
    Serial.println("joint_done," + String(position_curr.q1d) + "," + String(position_curr.q2d) + "," + String(position_curr.q3d) + "," + String(position_curr.X) + ","+ String(position_curr.Y) + ","+ String(position_curr.Z) + "*");

  }
  else {
    Serial.println("Unreachable_pose*");
  }
}

void prepare_linear(float x_req, float y_req, float z_req, float tool_offset, int speed_req, int waypoints){
  float point1[3] = {position_curr.X, position_curr.Y, position_curr.Z};
  float point2[3] = {x_req, y_req, z_req};
  if (z_req > z_lim){
    int numWaypoints = waypoints;
    long q1_arr[numWaypoints] = {};
    long q2_arr[numWaypoints] = {};
    long q3_arr[numWaypoints] = {};
    float time_step = 0;
    generateWaypointsLinear(point1, point2, speed_req, numWaypoints, q1_arr, q2_arr, q3_arr, &time_step, tool_offset);
    for (int i = 1; i < numWaypoints-1; i++) {
        int speed1 = abs((q1_arr[i] - q1_arr[i - 1]) / time_step);
        int speed2 = abs((q2_arr[i] - q2_arr[i - 1]) / time_step);
        int speed3 = abs((q3_arr[i] - q3_arr[i - 1]) / time_step);
        linear_movement(q1_arr[i], q2_arr[i], q3_arr[i], speed1, speed2, speed3);
    }

    long steps_result[4] = {0};
    inverse(x_req, y_req, z_req, tool_offset, steps_result);
    position_curr.q1s = steps_result[0];
    position_curr.q2s = steps_result[1];
    position_curr.q3s = steps_result[2];
    position_curr.q1d = (steps_result[0] * 360) / (steps_per_revolution * reduction);
    position_curr.q2d = (steps_result[1] * 360) / (steps_per_revolution * reduction);
    position_curr.q3d = (steps_result[2] * 360) / (steps_per_revolution * reduction);
    float position_result[4] = {0};
    direct(position_curr.q1d,position_curr.q2d,position_curr.q3d, tool_offset, position_result );
    position_curr.X = position_result[0];
    position_curr.Y = position_result[1];
    position_curr.Z = position_result[2];
    Serial.println("linear_done," + String(position_curr.q1d) + "," + String(position_curr.q2d) + "," + String(position_curr.q3d) + "," + String(position_curr.X) + ","+ String(position_curr.Y) + ","+ String(position_curr.Z) + "*");

  }
  else{
    Serial.println("Unreachable_pose*");
  }

}

void prepare_joint_t(float x_req, float y_req, float z_req, float tool_offset, long acc_req) {
  long steps_result[4] = {0};
  inverse(x_req, y_req, z_req, tool_offset, steps_result);
  if (steps_result[3] == 1 && z_req > z_lim){
    int distances[3];
    long steps_current[3] = {position_curr.q1s, position_curr.q2s, position_curr.q3s};
    int max_distance = 0;
    for (int i = 0; i < 3; i++) {
      distances[i] = abs(steps_result[i] - steps_current[i]);
      if (distances[i] > max_distance) {
          max_distance = distances[i];
      }
    }

    int speed[3];
    long acc[3];

    float tau = 2 * sqrt((float)max_distance / (float)acc_req);

    for (int i = 0; i < 3; i++) {
      speed[i] = (2 * distances[i]) / tau;
      acc[i] = (4 * distances[i]) / (tau * tau);
    }
    joint_movement_t(steps_result[0], steps_result[1], steps_result[2], speed[0], speed[1], speed[2], acc[0], acc[1], acc[2]);
    position_curr.q1s = steps_result[0];
    position_curr.q2s = steps_result[1];
    position_curr.q3s = steps_result[2];
    position_curr.q1d = (steps_result[0] * 360) / (steps_per_revolution * reduction);
    position_curr.q2d = (steps_result[1] * 360) / (steps_per_revolution * reduction);
    position_curr.q3d = (steps_result[2] * 360) / (steps_per_revolution * reduction);
    float position_result[4] = {0};
    direct(position_curr.q1d,position_curr.q2d,position_curr.q3d, tool_offset, position_result);
    position_curr.X = position_result[0];
    position_curr.Y = position_result[1];
    position_curr.Z = position_result[2];
    Serial.println("joint_done," + String(position_curr.q1d) + "," + String(position_curr.q2d) + "," + String(position_curr.q3d) + "," + String(position_curr.X) + ","+ String(position_curr.Y) + ","+ String(position_curr.Z) + "*");

  }
  else {
    Serial.println("Unreachable_pose*");
  }
}

void joint_movement_t(long steps1, long steps2, long steps3, int speed1,int speed2, int speed3, long acc1, long acc2, long acc3){
  stepper1.setMaxSpeed(speed1);
  stepper1.setAcceleration(acc1);
  stepper2.setMaxSpeed(speed2);
  stepper2.setAcceleration(acc2);
  stepper3.setMaxSpeed(speed3);
  stepper3.setAcceleration(acc3);
  stepper1.moveTo(-steps1);
  stepper2.moveTo(-steps2);
  stepper3.moveTo(-steps3);
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0){
    if (stepper1.distanceToGo() != 0){
      stepper1.run();
    }
    if (stepper2.distanceToGo() != 0){
      stepper2.run();
    }
    if (stepper3.distanceToGo() != 0){
      stepper3.run();
    }
  }
}
