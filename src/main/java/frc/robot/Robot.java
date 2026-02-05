package frc.robot;

import edu.wpi.first.math.controller.PIDController;
<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
>>>>>>> a21502e14d3a9a312a98653341c2e98a3bc45d84
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

<<<<<<< HEAD
=======
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Queue;
// Gryo imports for NavX
>>>>>>> a21502e14d3a9a312a98653341c2e98a3bc45d84
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
<<<<<<< HEAD
=======
  Timer timer = new Timer();
  double forwardSpeed;
  double turnSpeed;

  Optional<Alliance> alliance = DriverStation.getAlliance();
  OptionalInt station = DriverStation.getLocation();
>>>>>>> a21502e14d3a9a312a98653341c2e98a3bc45d84
  
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

<<<<<<< HEAD
=======
 @Override
  public void autonomousInit() {
    forwardSpeed = 0;
    turnSpeed = 0;

    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
      // Left station
      double fwd = 0; // positive - forward, negative - backward
      double rot = 0; // positive - turn left, negative - turn right

      if (station.equals(1)) {
        if (timer.get() < 1) {
          fwd = 0.5;
        } else if (timer.get() < 1.5) {
          fwd = 0;
        } else if (timer.get() < 2.5) {
          rot = 0.5;
        } else if (timer.get() < 3) {
          rot = 0;
        } else if (timer.get() < 4) {
          // output
        } else if (timer.get() < 10) {
          // output stop
        }
      } else if (station.equals(2)) {

      } else if (station.equals(3)) {

      }

      double leftSpeed = fwd + rot;
      double rightSpeed = fwd - rot;

      leftLeader.set(leftSpeed);
      rightLeader.set(rightSpeed);
    }

>>>>>>> a21502e14d3a9a312a98653341c2e98a3bc45d84
  @Override
  public void disabledPeriodic() {
    leftLeader.set(0);
    rightLeader.set(0);
  }
} 