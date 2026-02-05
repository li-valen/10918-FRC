package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import com.studica.frc.AHRS;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; // Added for Brake Mode
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Robot extends TimedRobot {
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private final PIDController gyroPID = new PIDController(0.015, 0.0, 0.005);

  private double targetAngle = 0.0;
  private double gyroCorrection;
  private boolean gyroAssistEnabled = false;

  private final SparkMax leftLeader = new SparkMax(5, MotorType.kBrushed);
  private final SparkMax leftFollower = new SparkMax(6, MotorType.kBrushed);
  private final SparkMax rightLeader = new SparkMax(8, MotorType.kBrushed);
  private final SparkMax rightFollower = new SparkMax(7, MotorType.kBrushed);

  private final SparkMax input = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax output = new SparkMax(2, MotorType.kBrushed);
  
  private boolean inputRunning = false;
  private boolean inputReverseRunning = false;
  private boolean outputRunning = false;

  private double maxFwd = 0.7;
  private double maxRot = 0.7;

  private final XboxController joystick = new XboxController(0);
  
  private int printCount = 0;

  public Robot() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig.inverted(true);
    rightConfig.inverted(false);

    // FIX: Set Brake Mode so motors stop instantly when power is 0
    leftConfig.idleMode(IdleMode.kBrake);
    rightConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.follow(leftLeader.getDeviceId()); 
    leftFollowerConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(rightLeader.getDeviceId());
    rightFollowerConfig.idleMode(IdleMode.kBrake);

    leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_gyro.reset();
    
    gyroPID.setSetpoint(0);
    gyroPID.setTolerance(2); 
    gyroPID.enableContinuousInput(-180, 180); 
  }

  @Override
  public void robotPeriodic() {
    if (++printCount >= 10) {
      printCount = 0;
      System.out.println("Gyro Yaw: " + m_gyro.getYaw());
      System.out.println("Target Angle: " + targetAngle);
    }
  }

@Override
  public void teleopPeriodic() {
    double leftStick = -joystick.getLeftY();
    double rightStick = -joystick.getRightY();

    if (Math.abs(leftStick) < 0.05) leftStick = 0;
    if (Math.abs(rightStick) < 0.05) rightStick = 0;

 
    boolean drivingStraight = (leftStick != 0 && rightStick != 0) && (Math.abs(leftStick - rightStick) < 0.2);

    if (drivingStraight) {
      if (!gyroAssistEnabled) {
        gyroAssistEnabled = true;
        targetAngle = m_gyro.getYaw();
        gyroPID.setSetpoint(targetAngle);
      }
      
      gyroCorrection = gyroPID.calculate(m_gyro.getYaw(), targetAngle);
      gyroCorrection = Math.max(-0.3, Math.min(0.3, gyroCorrection));
    } else {
      gyroAssistEnabled = false;
      gyroCorrection = 0;
      targetAngle = m_gyro.getYaw();
    }

 
    double leftSpeed = leftStick - gyroCorrection;
    double rightSpeed = rightStick + gyroCorrection;

    leftSpeed = Math.max(-maxFwd, Math.min(maxFwd, leftSpeed));
    rightSpeed = Math.max(-maxFwd, Math.min(maxFwd, rightSpeed));

  
    leftLeader.set(leftSpeed);
    rightLeader.set(rightSpeed);

    if (joystick.getAButtonPressed()) inputRunning = !inputRunning;
    if (joystick.getBButtonPressed()) outputRunning = !outputRunning;
    if (joystick.getXButtonPressed()) inputReverseRunning = !inputReverseRunning;

    if (inputRunning) {
      input.set(0.7);
    } else if (inputReverseRunning) {
      input.set(-0.7);
    } else {
      input.set(0);
    }

    if (outputRunning) {
      output.set(0.7);
    } else {
      output.set(0);
    }
  }

  @Override
  public void disabledPeriodic() {
    leftLeader.set(0);
    rightLeader.set(0);
  }
}