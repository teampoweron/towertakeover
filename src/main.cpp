/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
// Percentage speed of the intake motors of the robot also ramp motors
double IntakeSpeed = 200;
double OutakeSpeed = -18;
double RampUpSpeed = 22;
double FastRampUpSpeed = 22;
double SlowRampUpSpeed = 11;
double RampDownSpeed = -30;
double AutonRampUpSpeed = 28;
double AutonWheelsSpeed = 45;
double AutonTurningSpeed = 30;
double RampRotationRev = 2.6;
double RampBigSideRotationRev = 2.8;
double RampBigSideDownRotationRev = 0.8;

// blue=left=true-apply to both sides(so make the Big Side either true(on the big side) or false(not on the big side))
// red=right=false-" "
//BigSide=true=we are on the big side
bool LeftAuton = false;
bool CompetitionMode = true;
bool BigSide = false;

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
vex::controller::button FastUpRamp = Remote.ButtonX;
vex::controller::button SlowUpRamp = Remote.ButtonY;

// Autonomous code
void autonomous();
void autonomousBigSide();

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
  Brain.Screen.setCursor(1, 1);
  double adjustedAxis3Position = DriveSpeed(Remote.Axis3);
  double adjustedAxis1Position = DriveSpeed(Remote.Axis1);
  int Left_Speed = (adjustedAxis3Position + adjustedAxis1Position);
  int maxSpeed = 60; /* used to be 75 */
  if (Left_Speed > maxSpeed) {
    Left_Speed = maxSpeed;
  } else if (Left_Speed < -maxSpeed) {
    Left_Speed = -maxSpeed;
  }
  FLeftMotor.setVelocity(Left_Speed, vex::velocityUnits::pct);
  BLeftMotor.setVelocity(Left_Speed, vex::velocityUnits::pct);

  int Right_Speed = (adjustedAxis3Position - adjustedAxis1Position);

  Brain.Screen.print("Axis 3: %f Axis 1: %f \n", adjustedAxis3Position,
                     adjustedAxis1Position);
  Brain.Screen.newLine();
  if (Right_Speed > maxSpeed) {
    Right_Speed = maxSpeed;
  } else if (Right_Speed < -maxSpeed) {
    Right_Speed = -maxSpeed;
  }
  FRightMotor.setVelocity(Right_Speed, vex::velocityUnits::pct);
  BRightMotor.setVelocity(Right_Speed, vex::velocityUnits::pct);

  Brain.Screen.print("Left speed: %f Right_Speed: %f", Left_Speed, Right_Speed);

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
  RamperMotor.setVelocity(AutonRampUpSpeed, vex::velocityUnits::pct);
  RamperMotor.spin(vex::directionType::fwd);
  RamperMotor.rotateFor(RampRotationRev, vex::rotationUnits::rev,
                        false /* wait for completion*/);
  outake();
  Brain.Screen.print("RAMP IS WORKING");
}
void rampUpBigSideAuton() {
  RamperMotor.setVelocity(AutonRampUpSpeed, vex::velocityUnits::pct);
  RamperMotor.spin(vex::directionType::fwd);
  RamperMotor.rotateFor(RampBigSideRotationRev, vex::rotationUnits::rev,
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
  outake();
  RamperMotor.rotateFor(-RampBigSideDownRotationRev, vex::rotationUnits::rev,
                        true /* wait for completion*/);

  RamperMotor.setVelocity(0, vex::velocityUnits::pct);
  stopIntake();
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
  } else if (RampDown.pressing() /*&& RamperPosition > 0*/) {
    rampDown();
  } else {
    RamperMotor.setVelocity(0, vex::velocityUnits::pct);
    if (!Intake.pressing() && !Outake.pressing()) {
      stopIntake();
    }
  }
  if (FastUpRamp.pressing()) {
    RampUpSpeed = FastRampUpSpeed;
  }
  if (SlowUpRamp.pressing()) {
    RampUpSpeed = SlowRampUpSpeed;
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
    LeftAuton = true;
    autonomous();
    autonomousStarted = false;
  }
  if (Remote.ButtonB.pressing() && !autonomousStarted) {
    autonomousStarted = true;
    LeftAuton = false;
    autonomous();
    autonomousStarted = false;
  }
  // left arrow = blue
  // down arrow = red
  if (Remote.ButtonLeft.pressing() && !autonomousStarted) {
    autonomousStarted = true;
    LeftAuton = false;
    autonomousBigSide();
    autonomousStarted = false;
  }
  if (Remote.ButtonDown.pressing() && !autonomousStarted) {
    autonomousStarted = true;
    LeftAuton = true;
    autonomousBigSide();
    autonomousStarted = false;
  }
}

void setWheelVelocity(double velocity) {

  FRightMotor.setVelocity(velocity, vex::velocityUnits::pct);
  BRightMotor.setVelocity(velocity, vex::velocityUnits::pct);
  FLeftMotor.setVelocity(velocity, vex::velocityUnits::pct);
  BLeftMotor.setVelocity(velocity, vex::velocityUnits::pct);
  FLeftMotor.spin(vex::directionType::fwd);
  BLeftMotor.spin(vex::directionType::fwd);
  FRightMotor.spin(vex::directionType::fwd);
  BRightMotor.spin(vex::directionType::fwd);
}

// Move the robot calculations by given inches
void moveInches(double distanceInches) {
  double revolutions =
      distanceInches / 12.5; // 12.5 = circumference of the wheel
  Brain.Screen.print("revolutions:%f", revolutions);
  Brain.Screen.newLine();
  setWheelVelocity(AutonWheelsSpeed / 2);
  double initialSlowDistance = .2;
  if (distanceInches < 0) {
    initialSlowDistance = initialSlowDistance * -1;
  }
  FRightMotor.rotateFor(initialSlowDistance, vex::rotationUnits::rev,
                        false /* wait for completion */);
  BRightMotor.rotateFor(initialSlowDistance, vex::rotationUnits::rev,
                        false /* wait for completion */);
  FLeftMotor.rotateFor(initialSlowDistance, vex::rotationUnits::rev,
                       false /* wait for completion */);
  BLeftMotor.rotateFor(initialSlowDistance, vex::rotationUnits::rev,
                       true /* wait for completion */);
  setWheelVelocity(AutonWheelsSpeed);
  revolutions = revolutions - initialSlowDistance;
  FRightMotor.rotateFor(revolutions, vex::rotationUnits::rev,
                        false /* wait for completion */);
  BRightMotor.rotateFor(revolutions, vex::rotationUnits::rev,
                        false /* wait for completion */);
  FLeftMotor.rotateFor(revolutions, vex::rotationUnits::rev,
                       false /* wait for completion */);
  BLeftMotor.rotateFor(revolutions, vex::rotationUnits::rev,
                       true /* wait for completion */);
}

// Radius
int radius = 10;

// Turning the robot calculations by degree
// Clockwise degrees are POSITIVE !!
void turnDegree(double degrees) {
  double roboCircum = 45;                          // radius * 2 * 3.14
  double wheelTravel = degrees / 360 * roboCircum; /* inches */
  double revolutions = wheelTravel / 12.5; // 12.5 = circumference of the wheel
  FRightMotor.setVelocity(AutonTurningSpeed, vex::velocityUnits::pct);
  FRightMotor.rotateFor(-revolutions, vex::rotationUnits::rev,
                        false /* wait for completion */);
  BRightMotor.setVelocity(AutonTurningSpeed, vex::velocityUnits::pct);
  BRightMotor.rotateFor(-revolutions, vex::rotationUnits::rev,
                        false /* wait for completion */);
  FLeftMotor.setVelocity(AutonTurningSpeed, vex::velocityUnits::pct);
  FLeftMotor.rotateFor(revolutions, vex::rotationUnits::rev,
                       false /* wait for completion */);
  BLeftMotor.setVelocity(AutonTurningSpeed, vex::velocityUnits::pct);
  BLeftMotor.rotateFor(revolutions, vex::rotationUnits::rev,
                       true /* wait for completion */);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

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
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  double DirectionMultiplyer;
  if (LeftAuton) {
    DirectionMultiplyer = 1;
  } else {
    DirectionMultiplyer = -1;
  }
  intake();
  AutonWheelsSpeed = 30;
  moveInches(42);
  AutonWheelsSpeed = 45;
  moveInches(-22);
  turnDegree(DirectionMultiplyer * -130);
  setWheelVelocity(AutonWheelsSpeed);
  stopIntake();
  task::sleep(2800);
  rampUpAuton();
  setWheelVelocity(15);
  task::sleep(3000);
  moveInches(-20);
}

// IF RUNNING THIS AUTON MAKE THE SPEED WAYYYYY FASTERRR!!!!!!
// NEVER MIND :D :D :D
void autonomous6cubes(void) {
  double DirectionMultiplyer;
  if (LeftAuton) {
    DirectionMultiplyer = 1;
  } else {
    DirectionMultiplyer = -1;
  }
  intake();
  moveInches(42);
  moveInches(-17);
  turnDegree(DirectionMultiplyer * 47);
  moveInches(-29);
  turnDegree(DirectionMultiplyer * -54);
  moveInches(30);
  moveInches(-13);
  stopIntake();
  turnDegree(DirectionMultiplyer * -145);
  setWheelVelocity(AutonWheelsSpeed);
  task::sleep(700);
  rampUpAuton();
  setWheelVelocity(AutonWheelsSpeed);
  task::sleep(1600);
  setWheelVelocity(20);
  task::sleep(2000);
  moveInches(-20);
}

// FIX Comp mode big side vs small side
void autonomousBigSide(void) {
  double DirectionMultiplyer;
  if (LeftAuton) {
    DirectionMultiplyer = 1;
  } else {
    DirectionMultiplyer = -1;
  }
  intake();
  moveInches(15);
  turnDegree(DirectionMultiplyer * -95);
  // turnDegree(DirectionMultiplyer * -45);
  moveInches(25);
  //moveInches(-5);
  turnDegree(DirectionMultiplyer * -55);
  // moveInches(-25);
  // turnDegree(DirectionMultiplyer * -135);
  setWheelVelocity(AutonWheelsSpeed);
  task::sleep(1000);
  stopIntake();
  rampUpBigSideAuton();
  setWheelVelocity(15);
  task::sleep(3500);
  rampDownAuton();
  moveInches(-20);
  // turnDegree(180);
}
// void autonomousBigSide(void) {
//   double DirectionMultiplyer;
//   if (LeftAuton) {
//     DirectionMultiplyer = 1;
//   } else {
//     DirectionMultiplyer = -1;
//   }
//   intake();
//   moveInches(42);
//   turnDegree(DirectionMultiplyer * -135);
//   //moveInches(-25);
//   //turnDegree(DirectionMultiplyer * -45);
//   moveInches(40);
//   turnDegree(DirectionMultiplyer * -15);
//   //moveInches(-25);
//   //turnDegree(DirectionMultiplyer * -135);
//   setWheelVelocity(AutonWheelsSpeed);
//   task::sleep(1500);
//   stopIntake();
//   rampUpBigSideAuton();
//   setWheelVelocity(15);
//   task::sleep(3500);
//   rampDownAuton();
//   moveInches(-20);
//   //turnDegree(180);
// }

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    movement();
    buttons();

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  if (BigSide) {
    Competition.autonomous(autonomousBigSide);
  } else {
    Competition.autonomous(autonomous);
  }
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}