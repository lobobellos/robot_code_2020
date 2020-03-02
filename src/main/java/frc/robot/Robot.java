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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with arcade steering.
 */
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

  private VideoSink cameraServer;
  private UsbCamera frontCamera;
  private UsbCamera backCamera;
  
  // these variables are timers that track...
  private double elevatorStart = 0; // when the elevator starts running
  private double autonomousStart; // when autonomous mode starts

  @Override
  // when the robot boots up, configure the cameras
  public void robotInit() {
    frontCamera = CameraServer.getInstance().startAutomaticCapture();
    backCamera = CameraServer.getInstance().startAutomaticCapture();
    cameraServer = CameraServer.getInstance().getServer();
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
    // FIXME - what purpose does this serve?
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
    intakeMotor.set(intakeState ? 0.45 : 0);
    
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

    // trigger activates elevator/dump motor for X seconds at Y speed
    // TODO: parametrize these two values
    // Button 1 is the main trigger
    if (stick.getRawButtonPressed(1)) {
      elevatorStart = Timer.getFPGATimestamp();
    }
    elevatorMotor.set(Timer.getFPGATimestamp() < elevatorStart + 1 /* seconds */ ? 0.5 : 0);
  
  }
  
  @Override
  public void autonomousInit() {
    autonomousStart = Timer.getFPGATimestamp();
  }
  @Override
  public void autonomousPeriodic() {
    // TODO
  }
}
