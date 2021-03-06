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
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.utils.DigitalInputManager;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {

  // DRIVE MOTORS AND CONTROLS
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

  private final Gyro gyro = new ADXRS450_Gyro();


  // SENSORS AND RELATED STATE
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  int proximity;
  private boolean lastProximityState = false;
  
  DigitalInputManager spacingSwitch;
  private Boolean spacingSwitchActivated = false;

  private VideoSink cameraServer;
  private UsbCamera frontCamera;
  private UsbCamera backCamera;
  

  // ROBOT STATE

  // these variables track whether or not the intake motor should be running, and how fast
  private Boolean intakeEnabled = true; // allows softare control
  private Boolean manualIntakeEnabled = true; // allows manual disable
  private Boolean manualIntakeOverride = false; // allows manual force operation
  private double intakeSpeed;

  // these variables track whether or not the elevator motor should be running
  private Boolean elevatorEnabled = false; // does software grant permission?
  private Boolean manualElevatorEnabled = true; // does human grant permission?
  private Boolean manualElevatorOverride = false; // allows manual force operation

  private int nPowerCells = 0;

  // this variable tracks which end of the robot is currently defined as the "front" of the robot for
  // purposes of steering and camera
  // 1 = intake end is front
  // -1 = output is front
  private int direction = 1;


  // TIMING
  // these variables are timers that track...
  private double spacingSwitchTime; // when the spacing switch was pressed during a pulse
  private double autonomousStart; // when autonomous mode starts

  // these varbables keep track of things related to quick turning...
  private double startingAngle = 0; // angle the bot was at when starting
  private double turningAngle = 0; // the angle the bot should be turned to

  // TUNABLE PARAMETERS
  private int INTAKETHRESHOLD = 300; // proximity sensor reading that triggers intak
  private double ELEVATOR_INTAKE_DELAY = 0.068; // delay after spacing switch activation; 0.07 seems like a good value for 4 power cells
  private final int MAX_POWER_CELLS = 4; // number of power cells before intake stops
  private final double INTAKE_COLLECTION_SPEED = 0.8; // speed of intake motor
  private final double INTAKE_HOLD_SPEED = 0.4; // keep the motor running a bit when capturing final power cell to make sure it's held in place
  private final double ELEVATOR_SPEED = 1.0; // speed of elevator
  private final double ROTATION_SCALE = 1.0; // scale factor 0 to 1 to make turns easier to control, but not too slow

  // select which version of autonomous code to use
  // TODO: make switchable in hardware?
  private int AUTOMODE = 5;


  @Override
  // when the robot boots up, configure the cameras and create the switches
  // TODO: can we just do all this in the class variable declarations? What's the tradeoff?
  public void robotInit() {
    gyro.calibrate();
    frontCamera = CameraServer.getInstance().startAutomaticCapture();
    backCamera = CameraServer.getInstance().startAutomaticCapture();
    cameraServer = CameraServer.getInstance().getServer();
    spacingSwitch = new DigitalInputManager(5);
  }

  @Override
  // when entering teleop mode, we need to set the intake motor as running (or not)
  // leaving this false is probably the safer bet, though in competition we may want it to be true so that the
  // driver need not remember to activate the intake motor
  public void teleopInit() {
    intakeEnabled = true;
    manualIntakeEnabled = true;
    manualIntakeOverride = false;
    intakeSpeed = INTAKE_COLLECTION_SPEED;

    elevatorEnabled = false;
    manualElevatorEnabled = true;
    manualElevatorOverride = false;

    nPowerCells = 0;

  }

  @Override
  public void teleopPeriodic() {

    drivecontrol();
    handleManualInputs();
    handleSensorInputs();
    actuateMechanisms();
    updateDisplay();
    // TODO: data logging?
  }

  private void drivecontrol() {

    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.

    // read throttle to compute speed modification
    // linearly maps value from 1 to -1 into a value that is 0.5 to 1
    Double throttle = (-stick.getThrottle() + 1) / 2;
    throttle = throttle * 0.4 + 0.6;
    // System.out.println(speed); // debug

    // apply speed modification based on throttle and direction
    double driveSpeed = stick.getY() * throttle * direction;
    double driveRotation = stick.getX() * throttle * ROTATION_SCALE;
    double angle = gyro.getAngle();
    if (turningAngle > 0) {
      double offset = angle - startingAngle;
      if (offset < turningAngle) {
        driveRotation = 0.7;
      } else {
        driveRotation = -1; // attempt to correct for overrotation
        turningAngle = 0;
      }
    } else if (turningAngle < 0) {
      double offset = startingAngle - angle;
      if (offset < -turningAngle) {
        driveRotation = -0.7;
      } else {
        driveRotation = 1; // attempt to correct for overrotation
        turningAngle = 0;
      }
    }
    // instantaneous propulsion is based on the computed speed and rotation
    robotDrive.arcadeDrive(driveSpeed, driveRotation);

  }
  
  private void handleManualInputs() {
    
    // toggle enable/disable of intake motor
    // button 7 is top left outer on the base
    if (stick.getRawButtonPressed(7)) {
      manualIntakeEnabled = !manualIntakeEnabled;
    }

    // press and hold override button to force intake
    // button 8 is top left inner on base
    manualIntakeOverride = stick.getRawButton(8);

    // toggle enable/disable of elevator motor
    if (stick.getRawButtonPressed(9)) {
      manualElevatorEnabled = !manualElevatorEnabled;
    }

    // press and hold override button (trigger) to force elevator
    if (stick.getRawButton(1)) {
      manualElevatorOverride = true;
      elevatorEnabled = false; // terminate any pulse in progress
    }

    // after purging, restart the intake and assume the pipeline has been purged
    if (stick.getRawButtonReleased(1)) {
      manualElevatorOverride = false;
      intakeSpeed = INTAKE_COLLECTION_SPEED;
      nPowerCells = 0;
    }

    // manually terminate pulse operation e.g. if switch fails to detect
    if (stick.getRawButtonPressed(11)) {
      elevatorEnabled = false;
    }
    
    // based on a button press, invert the definition of the "front" of the robot
    // mainly involves inverting controls and which camera is used
    // Button 2 is the right thumb trigger
    // TODO: figure out how to get these cameras displayed on the smart dashboard
    if (stick.getRawButtonPressed(3)) {
      direction = -direction; // reverse between 1 and -1
      if (direction == -1) {
        cameraServer.setSource(backCamera);
      } else {
        cameraServer.setSource(frontCamera);
      }
    }

    // failsafe: reinitialize robot
    // button 11 is bottom left outter button on base
    if(stick.getRawButtonPressed(12)) {
      reset();
    }
    // left top button
    if (stick.getRawButtonPressed(5)) {
      if (turningAngle == 0) {
        startingAngle = gyro.getAngle();
      }
      turningAngle -= 90;
    }
    // right top button
    if (stick.getRawButtonPressed(6)) {
      if (turningAngle == 0) {
        startingAngle = gyro.getAngle();
      }
      turningAngle += 90;
    }

  }

  private void handleSensorInputs() {

    // read proximity sensor and check if it meets criteria for power cell detection
    int proximity = colorSensor.getProximity();
    boolean proximityState = proximity > INTAKETHRESHOLD;

    // if we changed proximity states, it means a power cell appeared
    // except if the elevator is already running, in which case it's a false double reading due to bounce
    // we can tell if it's infeasible for a new ball to show up if the elevator is already running as part of a pulse
    // or if we are already theoreticall at capacity
    if (!lastProximityState && proximityState && !elevatorEnabled && nPowerCells != MAX_POWER_CELLS) {
      nPowerCells++;
      if (nPowerCells == MAX_POWER_CELLS) {
        intakeSpeed = INTAKE_HOLD_SPEED;
      }
    }

    // keep a record to detect transitions
    lastProximityState = proximityState;

    // if there's a power cell at the intake, and we're not full, pulse the pipeline
    if (proximityState && nPowerCells != MAX_POWER_CELLS) {
      elevatorEnabled = true;
      intakeSpeed = INTAKE_HOLD_SPEED;
      spacingSwitchActivated = false;
    }

    // check it it's time to end an active elevator pulse
    if (elevatorEnabled) {

      // if the spacing switch is pressed, initiate shutdown
      spacingSwitch.periodic(); // update switch state
      if (spacingSwitch.pressed()) { // if the state has changed to a press...
        spacingSwitchTime = Timer.getFPGATimestamp();
        spacingSwitchActivated = true;
      }

      // determine any additional time that should be added after switch is pressed
      // scaled based on instantaneous battery voltage (could be a bit noisy)
      double pulseElongation = ELEVATOR_INTAKE_DELAY * (12 / RobotController.getBatteryVoltage());

      if (spacingSwitchActivated && Timer.getFPGATimestamp() >= spacingSwitchTime + pulseElongation) {
        elevatorEnabled = false;
        intakeSpeed = INTAKE_COLLECTION_SPEED;
      }

    }

  }

  private void actuateMechanisms() {

    // run the elevator motor if human and software grant permission, or if human forces operation
    if (manualElevatorEnabled && elevatorEnabled || manualElevatorOverride) {
      elevatorMotor.set(ELEVATOR_SPEED);
    } else {
      elevatorMotor.set(0);
    }

    // run the intake motor if human and software grant permission, or if human forces operation
    if (manualIntakeEnabled && intakeEnabled || manualIntakeOverride) {
      intakeMotor.set(intakeSpeed);
    } else {
      intakeMotor.set(0);
    }

  }

  // update info displayed on dashboard
  private void updateDisplay() {
    // TODO: format display in smart dashboard
    // TODO: display throttle and speed
    // TODO: display enable/disable and override for mechanisms
    // TODO: why are these displaying unreliably?
    // TODO: display motor speeds
    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putNumber("Power Cells", nPowerCells);
    SmartDashboard.putNumber("Angle", gyro.getAngle());
  }

  // TODO: can't we just call teleop init?
  private void reset() {
    nPowerCells = 0;

    intakeEnabled = true;
    manualIntakeEnabled = true;
    manualIntakeOverride = false;
    intakeSpeed = INTAKE_COLLECTION_SPEED;

    elevatorEnabled = false;
    manualElevatorEnabled = true;
    manualElevatorOverride = false;

    nPowerCells = 0;
  }

  @Override
  public void autonomousInit() {
    reset();
    updateDisplay();
    autonomousStart = Timer.getFPGATimestamp();
    if (AUTOMODE == 5) { // automode 5 keeps track of multiple powercells
      nPowerCells = MAX_POWER_CELLS - 2; // try to pickup 2 powercells
    }
  }

  @Override
  public void autonomousPeriodic() {
    
    if (AUTOMODE == 1) {
      autonomous1();
    } else if (AUTOMODE == 2) {
      autonomous2();
    } else if (AUTOMODE == 3) {
      autonomous3();
    } else if (AUTOMODE == 4) {
      autonomous4();
    } else if (AUTOMODE == 5) {
      autonomous5();
    }
  }

  // Option 1: Start in line with port, or off with some angular deviation. Drive forward, then purge.
  private void autonomous1() {
  
    double DELAY = 0; // could optionally add a delay if we need to wait for another robot to evacuate bottom port
    double elapsedTime = Timer.getFPGATimestamp() - autonomousStart - DELAY;
    intakeMotor.set(0); // unused in this mode

    if (elapsedTime < 3.5) { // drive forward
      robotDrive.arcadeDrive(-0.7, 0);
      elevatorMotor.set(0);
    } else if (elapsedTime < 6.5) { // then purge pipeline
      robotDrive.arcadeDrive(0, 0);
      elevatorMotor.set(ELEVATOR_SPEED);
    } else { // then wait
      robotDrive.arcadeDrive(0, 0);
      elevatorMotor.set(0);
    }
  
  }

  // Option 2: Start offset to left side
  // forward, turn, forward, purge, wait
  // TODO: calibrate parameters
  private void autonomous2() {
  
    double elapsedTime = Timer.getFPGATimestamp() - autonomousStart;
    intakeMotor.set(0); // unused in this mode
    
    if (elapsedTime < 2) { // move forward
      robotDrive.arcadeDrive(-0.7, 0);
      elevatorMotor.set(0);
    } else if (elapsedTime < 3) { // turn
      robotDrive.arcadeDrive(0, 1);
      elevatorMotor.set(0); 
    } else if (elapsedTime < 4) { // move forward
      robotDrive.arcadeDrive(-0.7, 0);
      elevatorMotor.set(0);
    } else if (elapsedTime < 7) { // purge
      robotDrive.arcadeDrive(0, 0);
      elevatorMotor.set(ELEVATOR_SPEED);
    } else { //wait
      robotDrive.arcadeDrive(0, 0);
      elevatorMotor.set(0);
    }

  }

  // Option 3: Just drive off the line if we're completely boxed out of lower port
  // ideally at least capture a fourth power cell
  // TODO: calibrate parameters
  private void autonomous3() {

    double elapsedTime = Timer.getFPGATimestamp() - autonomousStart;
    elevatorMotor.set(0); // unused
    
    if (elapsedTime < 3) { // move backwards
      robotDrive.arcadeDrive(0.7, 0);
      intakeMotor.set(INTAKE_COLLECTION_SPEED);
    } else { // wait
      robotDrive.arcadeDrive(0, 0);
      intakeMotor.set(0);
    }
    
  }

  // Option 4: Reverse to gather powercells, return to line, drive to power port, purge
  private void autonomous4() {

    double elapsedTime = Timer.getFPGATimestamp() - autonomousStart;
    
    if (elapsedTime < 3) { // try to collect 4th powercell
      robotDrive.arcadeDrive(0.7, 0);
      intakeMotor.set(INTAKE_COLLECTION_SPEED);
      elevatorMotor.set(0);
    } else if (elapsedTime < 6) { // drive back to line
      robotDrive.arcadeDrive(-0.7, 0);
      intakeMotor.set(0);
      elevatorMotor.set(0);
    } else if (elapsedTime < 6.12) { // rotate
      robotDrive.arcadeDrive(-0.7, -1);
      intakeMotor.set(0);
      elevatorMotor.set(0);
    } else if (elapsedTime < 9) { // drive to goal
      robotDrive.arcadeDrive(-0.7, 0);
      intakeMotor.set(0);
      elevatorMotor.set(0);
    } else if (elapsedTime < 15){ // dump
      robotDrive.arcadeDrive(0, 0);
      intakeMotor.set(0);
      elevatorMotor.set(ELEVATOR_SPEED);
    } else { // wait
      robotDrive.arcadeDrive(0, 0);
      intakeMotor.set(0);
      elevatorMotor.set(0);
    }

  }

  private void autonomous5() {

    double elapsedTime = Timer.getFPGATimestamp() - autonomousStart;
    if (elapsedTime < 4) { // try to collect more powercells
      robotDrive.arcadeDrive(0.7, 0);
      handleSensorInputs();
      intakeMotor.set(intakeSpeed);
      elevatorMotor.set(elevatorEnabled ? ELEVATOR_SPEED : 0); // controlled by sensors
    } else if (elapsedTime < 8) { // drive back to line
      robotDrive.arcadeDrive(-0.7, 0);
      intakeMotor.set(0);
      elevatorMotor.set(0);
    } else if (elapsedTime < 8.1) { // rotate
      robotDrive.arcadeDrive(-0.7, -1);
      intakeMotor.set(0);
      elevatorMotor.set(0);
    } else if (elapsedTime < 11) { // drive to goal
      robotDrive.arcadeDrive(-0.7, 0);
      intakeMotor.set(0);
      elevatorMotor.set(0);
    } else if (elapsedTime < 15){ // dump
      robotDrive.arcadeDrive(0, 0);
      intakeMotor.set(0);
      elevatorMotor.set(ELEVATOR_SPEED);
      nPowerCells = 0;
    } else { // wait
      robotDrive.arcadeDrive(0, 0);
      intakeMotor.set(0);
      elevatorMotor.set(0);
    }

  }
}
