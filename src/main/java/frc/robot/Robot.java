/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.utils.DigitalInputManager;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Robot extends TimedRobot {

  private final Spark leftMotorRear = new Spark(0);
  private final Spark leftMotorFront = new Spark(1);
  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMotorRear, leftMotorFront);

  private final Spark rightMotorRear = new Spark(2);
  private final Spark rightMotorFront = new Spark(3);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMotorRear, rightMotorFront);

  private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotors, rightMotors);
  private final Joystick stick = new Joystick(0);

  private final Spark intakeMotor = new Spark(8);
  private final Spark elevatorMotor = new Spark(9);

  // this variable tracks whether or not the intake motor should be running
  private Boolean intakeState = true;

  // this variable tracks which end of the robot is currently defined as the "front" of the robot for
  // purposes of steering and camera
  // 0 = intake end is front
  // 1 = output is front
  private int direction = 0;

  // switches on intake/elevator
  // private final DigitalInputManager captureSwitch = new DigitalInputManager(4);
  // private final DigitalInputManager spacingSwitch = new DigitalInputManager(5);

  //private Boolean elevatorMotorRunning = false;

  DigitalInput spacingSwitch;
  DigitalInput captureSwitch;

  private VideoSink cameraServer;
  private UsbCamera frontCamera;
  private UsbCamera backCamera;
  
  // these variables are timers that track...
  private double elevatorStart; // when the elevator starts running
  private double elevatorEnd = -1; // when the elevator should stop running
  private double autonomousStart; // when autonomous mode starts
  private double autophase1; // the duration of autonomous phase1
  private double autophase2; // the duration of autonomous phase2

  @Override
  // when the robot boots up, configure the cameras and create the switches
  public void robotInit() {
    frontCamera = CameraServer.getInstance().startAutomaticCapture();
    backCamera = CameraServer.getInstance().startAutomaticCapture();
    cameraServer = CameraServer.getInstance().getServer();
    captureSwitch = new DigitalInput(5);
    spacingSwitch = new DigitalInput(4);
  }

  @Override
  // when entering teleop mode, we need to set the intake motor as running (or not)
  // leaving this false is probably the safer bet, though in competition we may want it to be true so that the
  // driver need not remember to activate the intake motor
  public void teleopInit() {
    intakeState = false;
  }

  @Override
  public void teleopPeriodic() {
    // TODO - what purpose does this serve?
    Scheduler.getInstance().run();

    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.

    // read throttle to compute speed modification
    Double speed = (-stick.getThrottle() + 1) / 2;
    speed = (speed * 0.5) + 0.5;
    // System.out.println(speed); // debug

    // apply speed modification based on throttle
    double driveSpeed = stick.getY() * speed;
    double driveRotation = stick.getX() * speed;

    // invert direction based on current configuration
    if (direction == 1) {
      driveSpeed = -driveSpeed;
    }

    // instantaneous propulsion is based on the computed speed and rotation
    robotDrive.arcadeDrive(driveSpeed, driveRotation);

    // toggle intake motor state based on button press
    if (stick.getRawButtonPressed(7)) {
      intakeState = !intakeState;
    }

    // set the speed of intake motor to X if it should be on, else 0
    // TODO: parametrize this as a class constant
    if (intakeState) {
      intakeMotor.set(0.45);
    } else {
      intakeMotor.set(0);
    }

    // based on a button press, invert the definition of the "front" of the robot
    // main involved inverting controls and which camera is used
    // Button 2 is the right thumb trigger
    if (stick.getRawButtonPressed(2)) {
      switch (direction) {
        case 0:
          direction = 1;
          cameraServer.setSource(backCamera);
          break;
        case 1:
          direction = 0;
          cameraServer.setSource(frontCamera);
          break;
      }
    }

    // since a lower battery means a slower motor, we need to scale the time
    double elevatorMotorTime = 8 * (12 / RobotController.getBatteryVoltage());
    boolean elevatorMotorRunning = Timer.getFPGATimestamp() < elevatorStart + elevatorMotorTime;

    // turn the elevator motor on when the capture switch is pressed
    //captureSwitch.periodic();
    if (captureSwitch.get()) {
        elevatorMotorRunning = true;
    }
    
    // stop it once the spacing switch is pressed
    //spacingSwitch.periodic();
    if (spacingSwitch.get()) {
      elevatorMotorRunning = false;
    }

    elevatorMotor.set(elevatorMotorRunning || stick.getRawButton(1) ? 1 : 0);

    // start elevator/dump moto
    if (stick.getRawButtonPressed(5) && !elevatorMotorRunning) {
      elevatorStart = Timer.getFPGATimestamp();
    }

    // trigger activates elevator/dump motor for X seconds at Y speed
    // TODO: parametrize these two values
    // Button 1 is the main trigger
    if (stick.getRawButtonPressed(1)) {
      elevatorStart = Timer.getFPGATimestamp();
      elevatorEnd = elevatorStart + 1; // run the elevator for this many seconds
    }

    if (Timer.getFPGATimestamp() < elevatorEnd) {
      elevatorMotor.set(0.5);
    } else {
      elevatorMotor.set(0);
    }

  }
  
  @Override
  public void autonomousInit() {
    autonomousStart = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {

    double delta = Timer.getFPGATimestamp() - autonomousStart;
    int state;
    if (delta < 2) {
      state = 1;
    } else if (delta < 4) {
      state = 2;
    } else {
      state = 0;
    }

    if (state == 1) {
      robotDrive.arcadeDrive(-0.7, 0);
    } else {
      robotDrive.arcadeDrive(0, 0);
    }

    if (state == 2) {
      elevatorMotor.set(0.5);
    } else {
      elevatorMotor.set(0);
    }
    
  }
}
