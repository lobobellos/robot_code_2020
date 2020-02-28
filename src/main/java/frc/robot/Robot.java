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

 // private final DigitalInputManager captureSwitch = new DigitalInputManager(4);
 // private final DigitalInputManager spacingSwitch = new DigitalInputManager(5);

  private Boolean intakeState = false;
  //private Boolean elevatorMotorRunning = false;
  private int direction = 0; // 0: intake front, 1:

  DigitalInput spacingSwitch;
  DigitalInput captureSwitch;
  private VideoSink cameraServer;
  private UsbCamera frontCamera;
  private UsbCamera backCamera;
  private double elevatorStart = 0;
  @Override
  public void robotInit() {
    frontCamera = CameraServer.getInstance().startAutomaticCapture();
    backCamera = CameraServer.getInstance().startAutomaticCapture();
    cameraServer = CameraServer.getInstance().getServer();
    captureSwitch = new DigitalInput(5);
    spacingSwitch = new DigitalInput(4);
  }
  @Override
  public void teleopInit() {
    intakeState = false;
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


    // since a lower battery means a slower motor, we need to scale the time
    double elevatorMotorTime = 8 * (12 / RobotController.getBatteryVoltage());
    boolean elevatorMotorRunning =  Timer.getFPGATimestamp() < elevatorStart + elevatorMotorTime;


    // turn the elevator motor on when the capture switch is pressed
    //captureSwitch.periodic();
    if (captureSwitch.get()) {
      
        elevatorMotorRunning = true;
        
        
      }
      //elevatorStart = 1;
    
    //// stop it once the spacing switch is pressed
    //spacingSwitch.periodic();
    if (spacingSwitch.get()) {
      elevatorMotorRunning = false;
    }

    elevatorMotor.set(elevatorMotorRunning || stick.getRawButton(1) ? 1 : 0);

    

    // start elevator/dump motor
    
    if (stick.getRawButtonPressed(5) && !elevatorMotorRunning) {
      elevatorStart = Timer.getFPGATimestamp();
    }
    

    // toggle intake on button press
    if (stick.getRawButtonPressed(7)) {
      intakeState = !intakeState;
    }
    // set the speed to 0.5 if it should be on, else 0
    intakeMotor.set(intakeState && !elevatorMotorRunning ? 0.85 : 0);
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
  }





  private double autonomousStart;
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
