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

  private Boolean intakeState = true;

  private int direction = 0; // 0: intake front, 1:

  private VideoSink cameraServer;
  private UsbCamera frontCamera;
  private UsbCamera backCamera;
  private double elevatorStart = 0;
  @Override
  public void robotInit() {
    frontCamera = CameraServer.getInstance().startAutomaticCapture();
    backCamera = CameraServer.getInstance().startAutomaticCapture();
    cameraServer = CameraServer.getInstance().getServer();
  }
  @Override
  public void teleopInit() {
    intakeState = true;
  }
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.

    // read throttle to computer speed modification
    Double speed = (-stick.getThrottle() + 1) / 2;
    speed = (speed * 0.5) + 0.5;
    // System.out.println(speed);
    double driveSpeed = stick.getY() * speed;
    double driveRotation = stick.getX() * speed;
    if (direction == 1) {
      driveSpeed = -driveSpeed;
    }
    robotDrive.arcadeDrive(driveSpeed, driveRotation);

    // toggle intake on button press
    if (stick.getRawButtonPressed(7)) {
      intakeState = !intakeState;
    }
    // set the speed to 0.5 if it should be on, else 0
    intakeMotor.set(intakeState ? 0.45 : 0);
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

    // trigger activates elevator/dump motor for 1 second
    if (stick.getRawButtonPressed(1)) {
      elevatorStart = Timer.getFPGATimestamp();
    }
    elevatorMotor.set(Timer.getFPGATimestamp() < elevatorStart + 1 /* seconds */ ? 0.5 : 0);
  }
  private double autonomousStart;
  @Override
  public void autonomousInit() {
    autonomousStart = Timer.getFPGATimestamp();
  }
  @Override
  public void autonomousPeriodic() {
    // TODO
  }
}
