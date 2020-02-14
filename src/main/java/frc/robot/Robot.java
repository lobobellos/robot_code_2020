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
  private final Spark m_leftMotor_rear = new Spark(0);
  private final Spark m_leftMotor_front = new Spark(1);
  private final SpeedControllerGroup m_left = new SpeedControllerGroup(m_leftMotor_rear, m_leftMotor_front);
  
  private final Spark m_rightMotor_rear = new Spark(2); 
  private final Spark m_rightMotor_front = new Spark(3);
  private final SpeedControllerGroup m_right = new SpeedControllerGroup(m_rightMotor_rear, m_rightMotor_front);
  
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);
  private final Joystick m_stick = new Joystick(0);

  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();
  }
  //if you see this it worked v2
  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.

    // read throttle to computer speed modification
    Double speed = (-m_stick.getThrottle() + 1) / 2;
    speed = (speed * 0.5) + 0.5;
    // System.out.println(speed);

    m_robotDrive.arcadeDrive(m_stick.getY() * speed, m_stick.getX() * speed);
  }
}
