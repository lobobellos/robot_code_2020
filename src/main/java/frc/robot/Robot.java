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

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
public class Robot extends TimedRobot {
  private final PWMVictorSPX m_leftMotor_rear = new Spark(0);
  private final PWMVictorSPX m_leftMotor_front = new Spark(1);
  private final PMWVictorSPX m_rightMotor_rear = new Spark(2); 
  private final PWMVictorSPX m_rightMotor_front = new Spark(3);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor_rear, m_rightMotor_rear);
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
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }
}
