/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
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
  // Store the last state of the intake button, so weird things don't happen when the button is held.
  private Boolean intakeButtonDown = false;

  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();
  }
  @Override
  public void teleopInit() {
    intakeState = true;
  }
  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.

    // read throttle to computer speed modification
    Double speed = (-stick.getThrottle() + 1) / 2;
    speed = (speed * 0.5) + 0.5;
    // System.out.println(speed);

    robotDrive.arcadeDrive(stick.getY() * speed, stick.getX() * speed);

    // check if the buttin is down + make sure that the button wasn't down on the last iteration, so the toggle doesn't spazz.
    if (stick.getRawButton(7) && !intakeButtonDown) {
      intakeState = !intakeState;
    }
    // set the speed to 0.5 if it should be on, else 0
    intakeMotor.set(intakeState ? 0.45 : 0);
    intakeButtonDown = stick.getRawButton(7);
    // trigger activates elevator/dump motor
    if(stick.getRawButton(1)) {
      elevatorMotor.set(0.5);
    } else {
      elevatorMotor.set(0);
    }
  }
}
