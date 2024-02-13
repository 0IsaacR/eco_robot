#include <Servo.h>

Servo base_servo;
Servo shoulder_servo;
Servo elbow_servo;

int base_pin = 9;
int shoulder_pin = 10;
int elbow_pin = 11;

int low = -30;
int high = 30;

int pos = 0;
float base_angle = 5, shoulder_angle = 5, elbow_angle = 180;
float x_coor = 0, y_coor = 0, z_coor = 0, new_z_coor;
double theta1 = 0, theta2 = 0, theta3 = 0;
float r = 0, k = 0, phii = 0, gamm = 0;
//arm lengths in cm
float l1 = 5, l2 = 10, l3 = 13;

float x = 0, y = 0, z = 0;
float t1 = 0, t2 = 0, t3 = 0;

float pii = 3.14;


void setup() {
  base_servo.attach(base_pin);
  shoulder_servo.attach(shoulder_pin);
  elbow_servo.attach(elbow_pin);
  Serial.begin(9600);

  Serial.println("Performing Initial Tests. Moving to Home Position!");
  moveservos(base_angle, shoulder_angle, elbow_angle);
  Serial.println("Ready to Take Commands!");
  delay(3000);
}



void loop() {

  //Angle limits
  //base_angle_min = 30, base_angle_max = 120;
  //shoulder_angle_min = 30, base_angle_max = 90;
  //shoulder_angle_min = -60, base_angle_max = 0;


  base_angle = 0;
  shoulder_angle = 0;
  elbow_angle = 0;

  // Take angle input:
  Serial.print("Input Base Angle (5 to 180):");
  while (base_angle == 0) {
    base_angle = Serial.parseFloat();
  }
  Serial.println(base_angle);

  Serial.print("Input Shoulder Angle (5 to 65):");
  while (shoulder_angle == 0) {
    shoulder_angle = Serial.parseFloat();
  }
  Serial.println(shoulder_angle);

  Serial.print("Input Elbow Angle (80 to 180):");
  while (elbow_angle == 0) {
    elbow_angle = Serial.parseFloat();
  }
  Serial.println(elbow_angle);
  t1 = base_angle * (pii / 180);
  t2 = shoulder_angle * (pii / 180);
  t3 = elbow_angle * (pii / 180);

  //Coordinates - came from matlab (dx, dy, and dz)

  x = 10 * cos(t1) * cos(t2 - (pii / 2)) + 13 * cos(t1) * cos(t2 - (pii / 2)) * cos(t3 + pii) + 13 * cos(t1) * sin(t2 - (pii / 2)) * sin(t3 + pii);

  y = 10 * sin(t1) * cos(t2 - (pii / 2)) + 13 * sin(t1) * cos(t2 - (pii / 2)) * cos(t3 + pii) + 13 * sin(t1) * sin(t2 - (pii / 2)) * sin(t3 + pii);

  z = 13 * cos(t2 - (pii / 2)) * sin(t3 + pii) - 10 * sin(t2 - (pii / 2)) - 13 * cos(t3 + pii) * sin(t2 - (pii / 2)) + 5;


  Serial.println("Angles (degrees):");
  Serial.print("(");
  Serial.print(base_angle);
  Serial.print(",");
  Serial.print(shoulder_angle);
  Serial.print(",");
  Serial.print(elbow_angle);
  Serial.println(")");


  Serial.println("Coordinates (cm):");
  Serial.print("(");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(z);
  Serial.println(")");

  moveservos(base_angle, shoulder_angle, elbow_angle);

  delay(3000);

  // write code for inverse kinematics
  x_coor = 0;
  y_coor = 0;
  z_coor = 0;
  // enter your coordinates:
  Serial.print("Input X coordinate (0 to ?):");
  while (x_coor == 0) {
    x_coor = Serial.parseFloat();
  }
  Serial.println(x_coor);

  Serial.print("Input Y coordinate (0 to ?):");
  while (y_coor == 0) {
    y_coor = Serial.parseFloat();
  }
  Serial.println(y_coor);

  Serial.print("Input Z coordinate (0 to ?):");
  while (z_coor == 0) {
    z_coor = Serial.parseFloat();
  }
  Serial.println(z_coor);


// Inverse Kinematics from Matlab
  new_z_coor = z_coor - 5; 
  r = sqrt((x_coor * x_coor) + (y_coor * y_coor));
  k = sqrt((new_z_coor * new_z_coor) + (r*r));
  
  theta1 = atan2(y_coor,x_coor) * 180 / pii;

  phii = acos(((l2 * l2) + (k * k) - (l3 * l3)) / (2 * l2 * k)) * 180 / pii;
  gamm = asin(new_z_coor / k) * 180 / pii;

  theta2 = 90 - phii - gamm;

  theta3 = acos(((l2 * l2) + (l3 * l3) - (k * k)) / (2 * l2 * l3)) * 180 / pii;


  Serial.println("Coordinates (cm):");
  Serial.print("(");
  Serial.print(x_coor);
  Serial.print(",");
  Serial.print(y_coor);
  Serial.print(",");
  Serial.print(z_coor);
  Serial.println(")");


  Serial.println("Angles (degrees):");
  Serial.print("(");
  Serial.print(theta1);
  Serial.print(",");
  Serial.print(theta2);
  Serial.print(",");
  Serial.print(theta3);
  Serial.println(")");

  moveservos(theta1, theta2, theta3);

  delay(3000);
}


// function to move servos

void moveservos(float angle1, float angle2, float angle3) {

  if (angle1 < 5) {
    //base_servo.write(20);
    slowmove(base_servo, 5 * 0.72);
    Serial.println("Base angle too small moving to 5!");
  } else if (angle1 > 180) {
    //base_servo.write(110);
    slowmove(base_servo, 180 * 0.72);
    Serial.println("Base angel too large moving to 180!");
  } else {
    //base_servo.write(angle1);
    slowmove(base_servo, angle1 * 0.72);
    Serial.println("Base Servo moved to assigned angle");
  }
  delay(500);

  if (angle2 < 5) {
    //shoulder_servo.write(25);
    slowmove(shoulder_servo, 5 * 0.72);
    Serial.println("Shoulder angle too small moving to 5!");
  } else if (angle2 > 65) {
    //shoulder_servo.write(85);
    slowmove(shoulder_servo, 65 * 0.72);
    Serial.println("Shoulder angel too large moving to 65!");
  } else {
    //shoulder_servo.write(angle2);
    slowmove(shoulder_servo, angle2 * 0.72);
    Serial.println("Shoulder Servo moved to assigned angle");
  }
  delay(500);
  if (angle3 < 80) {
    //-elbow_servo.write(20);
    slowmove(elbow_servo, 80 * 0.72);
    Serial.println("Elbow angle too small moving to 80!");
  } else if (angle3 > 180) {
    //elbow_servo.write(80);
    slowmove(elbow_servo, 180 * 0.72);
    Serial.println("Elbow angle too large moving to 180!");
  } else {
    //elbow_servo.write(angle3);
    slowmove(elbow_servo, angle3 * 0.82);
    Serial.println("Elbow Servo moved to assigned angle");
  }
  delay(500);
}

void slowmove(Servo servo, float angle) {
  float curangle = servo.read();

  if (angle > curangle) {
    while (angle > curangle) {
      curangle++;
      servo.write(curangle);
      curangle = servo.read();
      delay(25);
    }
  } else if (angle < curangle) {

    while (angle < curangle) {
      curangle--;
      servo.write(curangle);
      curangle = servo.read();
      delay(25);
    }
  }
}