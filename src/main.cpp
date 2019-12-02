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

// Percentage speed of the intake motors of the robot also ramp motors
double IntakeSpeed = 100;
double OutakeSpeed = -25;
double RampUpSpeed = 30;
double RampDownSpeed = -15;
double AutonWheelsSpeed = 30;

vex::motor FLeftMotor =
    vex::motor(vex::PORT9, vex::gearSetting::ratio18_1, false);
vex::motor FRightMotor =
    vex::motor(vex::PORT10, vex::gearSetting::ratio18_1, true);
vex::motor BLeftMotor =
    vex::motor(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor BRightMotor =
    vex::motor(vex::PORT2, vex::gearSetting::ratio18_1, true);
vex::controller Remote = vex::controller();
vex::motor LeftIntakeMotor =
    vex::motor(vex::PORT3, vex::gearSetting::ratio18_1, false);
vex::motor RightIntakeMotor =
    vex::motor(vex::PORT8, vex::gearSetting::ratio18_1, true);
vex::motor RamperMotor =
    vex::motor(vex::PORT7, vex::gearSetting::ratio18_1, false);
vex::controller::button Intake = Remote.ButtonL1;
vex::controller::button Outake = Remote.ButtonL2;
vex::controller::button RampUp = Remote.ButtonR1;
vex::controller::button RampDown = Remote.ButtonR2;

// Autonomous code
void autonomous();

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

  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Wheels: %d %d %d %d",
                     (int)FLeftMotor.rotation(vex::rotationUnits::deg),
                     (int)FRightMotor.rotation(vex::rotationUnits::deg),
                     (int)BLeftMotor.rotation(vex::rotationUnits::deg),
                     (int)BRightMotor.rotation(vex::rotationUnits::deg));
  Brain.Screen.newLine();
}

void intake() {
  RightIntakeMotor.setVelocity(IntakeSpeed, vex::velocityUnits::pct);
  LeftIntakeMotor.setVelocity(IntakeSpeed, vex::velocityUnits::pct);
  Brain.Screen.print("INTAKE IS WORKING");
}

void outake() {
  RightIntakeMotor.setVelocity(OutakeSpeed, vex::velocityUnits::pct);
  LeftIntakeMotor.setVelocity(OutakeSpeed, vex::velocityUnits::pct);
  Brain.Screen.print("OUTAKE IS WORKING");
}

void stopIntake() {
  RightIntakeMotor.setVelocity(0, vex::velocityUnits::pct);
  LeftIntakeMotor.setVelocity(0, vex::velocityUnits::pct);
}

void rampUpAuton() {
  RamperMotor.setVelocity(RampUpSpeed, vex::velocityUnits::pct);
  RamperMotor.rotateFor(160, vex::rotationUnits::rev,
                        false /* wait for completion*/);
  outake();
  Brain.Screen.print("RAMP IS WORKING");
}

void rampUp() {
  RamperMotor.setVelocity(RampUpSpeed, vex::velocityUnits::pct);
  outake();
}

void rampDownAuton() {
  RamperMotor.setVelocity(RampDownSpeed, vex::velocityUnits::pct);
  RamperMotor.rotateFor(-160, vex::rotationUnits::rev,
                        true /* wait for completion*/);
  intake();
}

void rampDown() {
  RamperMotor.setVelocity(RampDownSpeed, vex::velocityUnits::pct);
  intake();
}

bool autonomousStarted = false;

// The intake code :)
void buttons() {

  double RamperPosition = RamperMotor.rotation(vex::rotationUnits::deg);
  Brain.Screen.print(RamperPosition);
  Brain.Screen.newLine();
  if (RampUp.pressing() /*&& RamperPosition < 160*/) {
    rampUp();
  } else if (RampDown.pressing() /*&& RamperPosition > 1*/) {
    rampDown();
  } else {
    RamperMotor.setVelocity(0, vex::velocityUnits::pct);
    if (!Intake.pressing() && !Outake.pressing()) {
      stopIntake();
    }
  }
  if (Intake.pressing()) {
    intake();
  } else if (Outake.pressing()) {
    outake();
  } else {
    stopIntake();
  }
  if (Remote.ButtonA.pressing() && !autonomousStarted) {
    autonomousStarted = true;
    autonomous();
    autonomousStarted = false;
  }
}

// Move the robot calculations by given inches
void moveInches(double distanceInches) {
  double revolutions =
      distanceInches / 12.5; // 12.5 = circumference of the wheel
  Brain.Screen.print("revolutions:%f", revolutions);
  Brain.Screen.newLine();
  FRightMotor.setVelocity(AutonWheelsSpeed, vex::velocityUnits::pct);
  FRightMotor.rotateFor(revolutions, vex::rotationUnits::rev,
                        false /* wait for completion */);
  BRightMotor.setVelocity(AutonWheelsSpeed, vex::velocityUnits::pct);
  BRightMotor.rotateFor(revolutions, vex::rotationUnits::rev,
                        false /* wait for completion */);
  FLeftMotor.setVelocity(AutonWheelsSpeed, vex::velocityUnits::pct);
  FLeftMotor.rotateFor(revolutions, vex::rotationUnits::rev,
                       false /* wait for completion */);
  BLeftMotor.setVelocity(AutonWheelsSpeed, vex::velocityUnits::pct);
  BLeftMotor.rotateFor(revolutions, vex::rotationUnits::rev,
                       true /* wait for completion */);
}

// Radius
int radius = 10;

// Turning the robot calculations by degree
// Clockwise degrees are POSITIVE !!
void turnDegree(double degrees) {
  double roboCircum = 40;                          // radius * 2 * 3.14
  double wheelTravel = degrees / 360 * roboCircum; /* inches */
  double revolutions = wheelTravel / 12.5; // 12.5 = circumference of the wheel
  FRightMotor.setVelocity(AutonWheelsSpeed, vex::velocityUnits::pct);
  FRightMotor.rotateFor(-revolutions, vex::rotationUnits::rev,
                        false /* wait for completion */);
  BRightMotor.setVelocity(AutonWheelsSpeed, vex::velocityUnits::pct);
  BRightMotor.rotateFor(-revolutions, vex::rotationUnits::rev,
                        false /* wait for completion */);
  FLeftMotor.setVelocity(AutonWheelsSpeed, vex::velocityUnits::pct);
  FLeftMotor.rotateFor(revolutions, vex::rotationUnits::rev,
                       false /* wait for completion */);
  BLeftMotor.setVelocity(AutonWheelsSpeed, vex::velocityUnits::pct);
  BLeftMotor.rotateFor(revolutions, vex::rotationUnits::rev,
                       true /* wait for completion */);
}

// Autonomous code
void autonomous() {
  Brain.Screen.print("AUTON IS WORKING");
  Brain.Screen.newLine();
  intake();
  // task::sleep(300);
  Brain.Screen.print("INTAKE IS WORKING");
  Brain.Screen.newLine();
  moveInches(42);
  // task::sleep(300);
  Brain.Screen.print("MOVING IS WORKING");
  Brain.Screen.newLine();
  stopIntake();
  // task::sleep(300);
  Brain.Screen.print("STOP INTAKE IS WORKING");
  Brain.Screen.newLine();
  turnDegree(180);
  // task::sleep(300);
  Brain.Screen.print("TURNING IS WORKING");
  Brain.Screen.newLine();
  moveInches(20);
  // task::sleep(300);
  Brain.Screen.print("MOVING2 IS WORKING\n");
  turnDegree(45);
  // task::sleep(300);
  Brain.Screen.print("TURNING2 IS WORKING\n");
  moveInches(26);
  // task::sleep(300);
  Brain.Screen.print("MOVING3 IS WORKING\n");
  rampUpAuton();
  // task::sleep(300);
  Brain.Screen.print("RAMP2 IS WORKING");
  Brain.Screen.newLine();
  moveInches(5);
  // task::sleep(300);
  Brain.Screen.print("MOVINIG4 IS WORKING");
  Brain.Screen.newLine();
  moveInches(-20);
  // task::sleep(300);
  Brain.Screen.print("MOVING5 IS WORKING");
  Brain.Screen.newLine();
  rampDownAuton();
  // task::sleep(300);
  Brain.Screen.print("RAMP3 IS WORKING");
  Brain.Screen.newLine();
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