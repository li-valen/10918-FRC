package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import com.studica.frc.AHRS;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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

  private final SparkMax input_shooter = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax output = new SparkMax(2, MotorType.kBrushed);

  private final SparkMax climb = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax indexer = new SparkMax(1, MotorType.kBrushed);


  private boolean inputRunning = false;
  private boolean inputReverseRunning = false;
  private boolean outputRunning = false;

  private double maxFwd = 0.7;

  private boolean autoStarted = false;

  private double forwardSpeed;
  private double turnSpeed;

  private double leftTriggerSpeed;
  private double rightTriggerSpeed;

  private int id;
  private double distanceToTag;

  Timer timer = new Timer();

  private final XboxController joystick = new XboxController(0);

  private int printCount = 0;

  public Robot() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig.inverted(true);
    rightConfig.inverted(false);

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

  public int getTargetID() {
    return (int) NetworkTableInstance.getDefault()
        .getTable("limelight")
        .getEntry("tid")
        .getInteger(-1);
  }

  @Override
  public void robotPeriodic() {
    id = getTargetID();
    boolean hasTarget = LimelightHelpers.getTV("limelight");

    if (hasTarget) {
      Pose3d targetPose = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
      double distanceMeters = Math.hypot(targetPose.getX(), targetPose.getY());

      if (distanceMeters > 0.1) {
        distanceToTag = distanceMeters;
      } else {
        double targetHeight = 1.397;
        double limelightHeight = 0.508;
        double limelightMountAngle = 30.0;
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        distanceToTag = (targetHeight - limelightHeight) / Math.tan(Math.toRadians(limelightMountAngle + ty));
      }
    }

    if (++printCount >= 10) {
      printCount = 0;
      System.out.println("Gyro Yaw: " + m_gyro.getYaw());
      System.out.println("id detected: " + id);
      System.out.println("Distance (cm): " + distanceToTag * 100);
      System.out.println("Left Trigger Axis: " + leftTriggerSpeed);
      System.out.println("Right Trigger Axis: " + rightTriggerSpeed
      );
    }

  }

  @Override
  public void teleopPeriodic() {
    double leftStick = -joystick.getLeftY();
    double rightStick = -joystick.getRightY();

    if (Math.abs(leftStick) < 0.05)
      leftStick = 0;
    if (Math.abs(rightStick) < 0.05)
      rightStick = 0;

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

    leftLeader.set(Math.max(-maxFwd, Math.min(maxFwd, leftSpeed)));
    rightLeader.set(Math.max(-maxFwd, Math.min(maxFwd, rightSpeed)));
    
    leftTriggerSpeed = joystick.getLeftTriggerAxis();
    rightTriggerSpeed = joystick.getRightTriggerAxis();

    if (leftTriggerSpeed > 0.1) {
      climb.set(leftTriggerSpeed);
    } else if (rightTriggerSpeed > 0.1) {
      climb.set(-rightTriggerSpeed);
    }

    if (joystick.getAButtonPressed())
      inputRunning = !inputRunning;
    if (joystick.getBButtonPressed())
      outputRunning = !outputRunning;
    if (joystick.getXButtonPressed())
      inputReverseRunning = !inputReverseRunning;

    if (inputRunning) {
      input_shooter.set(0.7);
    } else if (inputReverseRunning) {
      input_shooter.set(-0.7);
    } else {
      input_shooter.set(0);
    }

    if (outputRunning) {
      output.set(0.7);
    } else {
      output.set(0);
    }

  }

  @Override
  public void autonomousInit() {
    autoStarted = false;
    id = -1;
    m_gyro.reset();
    timer.stop();
    timer.reset();
  }

  @Override
  public void autonomousPeriodic() {
    if (!autoStarted) {
      id = getTargetID();
      if (id != -1) {
        autoStarted = true;
        timer.restart();
      } else {
        forwardSpeed = 0;
        turnSpeed = 0;
      }
    }

    if (autoStarted) {
      double time = timer.get();

      if (time < 0.5) {
        forwardSpeed = -0.5;
        turnSpeed = 0;
      } else if (time < 1.0) {
        forwardSpeed = 0;
        turnSpeed = 0;
      } else if (time < 1.5) {
        forwardSpeed = 0;
        turnSpeed = 0.2;
      } else {
        forwardSpeed = 0;
        turnSpeed = 0;
        timer.stop();
      }
    }

    leftLeader.set(forwardSpeed + turnSpeed);
    rightLeader.set(forwardSpeed - turnSpeed);

  }

  @Override
  public void disabledPeriodic() {
    leftLeader.set(0);
    rightLeader.set(0);
  }
}