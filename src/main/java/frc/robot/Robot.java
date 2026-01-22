// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(1);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(2);

  private final PWMSparkMax m_output = new PWMSparkMax(2); // change later
  private final PWMSparkMax m_input = new PWMSparkMax(2); // change later

  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
  double forwardSpeed = 0;
  double turnSpeed = 0;


  

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_rightDrive.setInverted(true);
    m_leftDrive.addFollower(new PWMSparkMax(3));
    m_rightDrive.addFollower(new PWMSparkMax(4));
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);
    m_robotDrive.setMaxOutput(0.5);
    
    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
      m_timer.start();

      // Move the Robot Back
      if (m_timer.get() < 1.5) {
        m_robotDrive.arcadeDrive(0.5, 0, false);
      } else if (m_timer.get() == 1.5) {
        m_robotDrive.arcadeDrive(0, 0, false);
      }

      if (m_timer.get() < 5.0) {
        // make output motor go forward
        m_output.set(0.5);
      } else if (m_timer.get() == 5.0) {
        m_output.set(0);
      }

      if (m_timer.get() < 10) {
        m_robotDrive.arcadeDrive(-1, 0, false);
      } else if (m_timer.get() == 10) {
        m_robotDrive.arcadeDrive(0, 0, false);
      }
      
      if (m_timer.get() < 13) {

      }

    





    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0, false);
    } else if (m_timer.get() < 4.0) {
      m_robotDrive.arcadeDrive(-0.5, 0, false);
    } else if (m_timer.get() < 5.0) {
      m_robotDrive.arcadeDrive(0, -1, false);
    }
  } 
  

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    double rawForwardSpeed = -m_controller.getLeftY();
    double rawTurnSpeed = -m_controller.getRightX();

    double maxAcceleration = 0.05;
    if (rawForwardSpeed >= forwardSpeed + maxAcceleration){
      forwardSpeed = forwardSpeed + maxAcceleration;
    } else {
      forwardSpeed = rawForwardSpeed;
    }
    if (rawTurnSpeed >= turnSpeed + maxAcceleration){
      turnSpeed = turnSpeed + maxAcceleration;
    } else {
      turnSpeed = rawTurnSpeed;
    }

    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  

}
