/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       teampoweron                                               */
/*    Created:      Sun Oct 27 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"
using namespace vex;
/*-Ramp is too fast check*/
/*-Ramp and intake are backwards check*/
/*-Ramp is falling*/
/*-Ramp bounderies -17 to -170 check*/
/*-Some motors are not spinning check*/
/*Ramp dosen't go down check*/
/* Right front dosen't go foward but does turn check*/
/* Defining the motors and variables and controllers check*/
/*Ramp goes slower*/
vex::motor FLeftMotor = vex::motor(vex::PORT9, false);
vex::motor FRightMotor = vex::motor(vex::PORT10, true);
vex::motor BLeftMotor = vex::motor(vex::PORT1, false);
vex::motor BRightMotor = vex::motor(vex::PORT2, true);
vex::controller Remote = vex::controller();
vex::motor LeftIntakeMotor = vex::motor(vex::PORT3, false);
vex::motor RightIntakeMotor = vex::motor(vex::PORT8, true);
vex::motor RamperMotor = vex::motor(vex::PORT7, true);
vex::controller::button Intake = Remote.ButtonL1;
vex::controller::button Outake = Remote.ButtonL2;
vex::controller::button RampUp = Remote.ButtonR1;
vex::controller::button RampDown = Remote.ButtonR2;

// Percentage speed of the intake motors of the robot also ramp motors
double IntakeSpeed = 100;
double OutakeSpeed = -50;
double RampUpSpeed = 10;
double RampDownSpeed = -15;

double DriveSpeed(vex::controller::axis axis) {
  if (axis.position() < 50 && axis.position() > -50) {
    return axis.position() * 0.5;
  } else if (axis.position() > 0) {
    return (axis.position() - 50) * 1.5 + 25;
  } else {
    return (axis.position() + 50) * 1.5 - 25;
  }
}

// Combining the wheels
void movement() {
  double adjustedAxis3Position = DriveSpeed(Remote.Axis3);
  double adjustedAxis1Position = DriveSpeed(Remote.Axis1);
  int Left_Speed = (adjustedAxis3Position + adjustedAxis1Position);
  int maxSpeed = 75;
  if (Left_Speed > maxSpeed) {
    Left_Speed = maxSpeed;
  } else if (Left_Speed < -maxSpeed) {
    Left_Speed = -maxSpeed;
  }
  FLeftMotor.setVelocity(Left_Speed, vex::velocityUnits::pct);
  BLeftMotor.setVelocity(Left_Speed, vex::velocityUnits::pct);

  int Right_Speed = (adjustedAxis3Position - adjustedAxis1Position);
  if (Right_Speed > maxSpeed) {
    Right_Speed = maxSpeed;
  } else if (Right_Speed < -maxSpeed) {
    Right_Speed = -maxSpeed;
  }
  FRightMotor.setVelocity(Right_Speed, vex::velocityUnits::pct);
  BRightMotor.setVelocity(Right_Speed, vex::velocityUnits::pct);
}

// The intake code :)
void buttons() {
  if (Intake.pressing()) {
    RightIntakeMotor.setVelocity(IntakeSpeed, vex::velocityUnits::pct);
    LeftIntakeMotor.setVelocity(IntakeSpeed, vex::velocityUnits::pct);
  } else if (Outake.pressing()) {
    RightIntakeMotor.setVelocity(OutakeSpeed, vex::velocityUnits::pct);
    LeftIntakeMotor.setVelocity(OutakeSpeed, vex::velocityUnits::pct);
  } else {
    RightIntakeMotor.setVelocity(0, vex::velocityUnits::pct);
    LeftIntakeMotor.setVelocity(0, vex::velocityUnits::pct);
  }
  double RamperPosition = RamperMotor.rotation(vex::rotationUnits::deg);
  Brain.Screen.print(RamperPosition);
  Brain.Screen.newLine();
  if (RampUp.pressing() && RamperPosition < 160) {
    RamperMotor.setVelocity(RampUpSpeed, vex::velocityUnits::pct);
  } else if (RampDown.pressing() && RamperPosition > 1) {
    RamperMotor.setVelocity(RampDownSpeed, vex::velocityUnits::pct);
  } else {
    RamperMotor.setVelocity(0, vex::velocityUnits::pct);
  }
}

// Return a number/integer --v
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Intake spinning mechanism
  RightIntakeMotor.setVelocity(0, vex::velocityUnits::pct);
  LeftIntakeMotor.setVelocity(0, vex::velocityUnits::pct);
  RightIntakeMotor.spin(vex::directionType::fwd);
  LeftIntakeMotor.spin(vex::directionType::fwd);

  // Ramp spinning mechanism
  RamperMotor.setVelocity(0, vex::velocityUnits::pct);
  RamperMotor.spin(vex::directionType::fwd);
  RamperMotor.setStopping(vex::brakeType::hold);

  // Speed + Direction of motors
  FLeftMotor.setVelocity(0, vex::velocityUnits::pct);
  BLeftMotor.setVelocity(0, vex::velocityUnits::pct);
  FRightMotor.setVelocity(0, vex::velocityUnits::pct);
  BRightMotor.setVelocity(0, vex::velocityUnits::pct);

  // Spinning the motors!
  FLeftMotor.spin(vex::directionType::fwd);
  BLeftMotor.spin(vex::directionType::fwd);
  FRightMotor.spin(vex::directionType::fwd);
  BRightMotor.spin(vex::directionType::fwd);

  // Printing Words
  Brain.Screen.print("carina!!");
  Brain.Screen.newLine();
  Brain.Screen.print("chloe!!");
  Brain.Screen.newLine();

  while (true) {
    movement();
    buttons();
  }
}