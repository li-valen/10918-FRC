package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.studica.frc.AHRS;

import java.util.Optional;
import java.util.OptionalInt;

import com.andymark.jni.AM_CAN_HexBoreEncoder;
import com.andymark.jni.AM_CAN_HexBoreEncoder.AM_EncoderStatus;
import com.andymark.jni.AM_CAN_HexBoreEncoder.AM_Encoder_Telemetry;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Robot extends TimedRobot {
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private final PIDController gyroPID = new PIDController(0.02, 0.0, 0.05);

  private double targetAngle = 0.0;
  private double gyroCorrection;
  private boolean gyroAssistEnabled = false;

  private final SparkMax leftLeader = new SparkMax(5, MotorType.kBrushed);
  private final SparkMax leftFollower = new SparkMax(6, MotorType.kBrushed);
  private final SparkMax rightLeader = new SparkMax(8, MotorType.kBrushed);
  private final SparkMax rightFollower = new SparkMax(7, MotorType.kBrushed);

  private final SparkMax inputLeader = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax inputFollower = new SparkMax(3, MotorType.kBrushless);

  private final SparkMax climb = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax indexer = new SparkMax(1, MotorType.kBrushed);

  private final AM_CAN_HexBoreEncoder intakeEncoder = new AM_CAN_HexBoreEncoder(0);
  private final AM_CAN_HexBoreEncoder climbEncoder = new AM_CAN_HexBoreEncoder(1);
  private final Encoder leftEncoder = new Encoder(0, 1);
  private final Encoder rightEncoder = new Encoder(2, 3);

  // initializes variables for the movement encoders
  // gear ratio might need to be editted
  private final double cpr = 360;
  private final double wheelDiameter = 6.0;
  private final double wheelCircumference = Math.PI * wheelDiameter;
  private final double gearRatio = (10.71);
  private final double conversionFactor = wheelCircumference / gearRatio;
  // private final double climbInputTeeth = 10;
  // private final double climbOutputTeeth = 28;
  // private final double gearRatio = climbOutputTeeth / climbInputTeeth;

  private boolean Intake = false;
  private boolean Shooter = false;
  // private boolean indexerRunning = false;
  // private boolean indexerReverseRunning = false;

  private double maxFwd = 0.7;

  private double forwardSpeed = 0;
  private double turnSpeed = 0;

  private double leftTrigger;
  private double rightTrigger;

  private int id;
  private double distanceToTowerTag;
  private int autoStep;
  private boolean shootTimerStarted = false;
  private boolean gyroResetDone = false;

  // Encoder Distances
  double distance;
  Timer timer = new Timer();
  Timer shootTimer = new Timer();
  Timer turnTimer = new Timer();

  private final XboxController joystick = new XboxController(0);
  private final XboxController joystick1 = new XboxController(1);


  public Robot() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    SparkMaxConfig inputConfig = new SparkMaxConfig();
    SparkMaxConfig climbConfig = new SparkMaxConfig();

    climbConfig.idleMode(IdleMode.kBrake);

    leftConfig.inverted(true);
    rightConfig.inverted(false);
    leftConfig.idleMode(IdleMode.kBrake);
    rightConfig.idleMode(IdleMode.kBrake);
    leftConfig.openLoopRampRate(0.25);
    rightConfig.openLoopRampRate(0.25);

    inputConfig.inverted(true);
    inputConfig.idleMode(IdleMode.kCoast);
    inputConfig.smartCurrentLimit(60);
    inputConfig.voltageCompensation(12);

    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.follow(leftLeader.getDeviceId());
    leftFollowerConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(rightLeader.getDeviceId());
    rightFollowerConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig inputFollowerConfig = new SparkMaxConfig();
    inputFollowerConfig.follow(inputLeader, true);
    inputFollowerConfig.idleMode(IdleMode.kCoast);

    leftConfig.encoder.positionConversionFactor(conversionFactor);
    rightConfig.encoder.positionConversionFactor(conversionFactor);
    leftEncoder.setReverseDirection(true);

    leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    inputLeader.configure(inputConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    inputFollower.configure(inputFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climb.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEncoder.setDistancePerPulse(wheelCircumference / cpr);
    rightEncoder.setDistancePerPulse(wheelCircumference / cpr);

    m_gyro.reset();
    gyroPID.setSetpoint(0);
    gyroPID.setTolerance(2);
    gyroPID.enableContinuousInput(-180, 180);

    climbEncoder.getTelemetry(300);
  }

  public int getTargetID() {
    return (int) NetworkTableInstance.getDefault()
        .getTable("limelight")
        .getEntry("tid")
        .getInteger(-1);
  }

  @Override
  public void robotPeriodic() {
    if (!gyroResetDone && m_gyro.isConnected() && !m_gyro.isCalibrating()) {
      m_gyro.reset();
      gyroResetDone = true;
    }

    id = getTargetID();
    boolean hasTarget = LimelightHelpers.getTV("limelight");

    if (hasTarget) {
      Pose3d targetPose = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
      distanceToTowerTag = -targetPose.getZ(); // negative because Z is toward camera
    } else {
      distanceToTowerTag = -1;
    }

    // if (hasTarget) {
    // Pose3d targetPose = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
    // double distanceMeters = targetPose.getZ();

    // if (distanceMeters < 3.0) {
    // distanceToTowerTag = distanceMeters;
    // } else {
    // double targetHeight = 0.5524;
    // double limelightHeight = 0.508;
    // double limelightMountAngle = 9.5;
    // double ty =
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // distanceToTowerTag = (targetHeight - limelightHeight) /
    // Math.tan(Math.toRadians(limelightMountAngle + ty));
    // }
    // }

    SmartDashboard.putNumber("Gyro Yaw: ", m_gyro.getYaw());
    SmartDashboard.putBoolean("NavX Connected: ", m_gyro.isConnected());
    SmartDashboard.putBoolean("NavX Reset", gyroResetDone);
    SmartDashboard.putNumber("Climb Degrees: ", climbEncoder.getAngleDegrees());
    SmartDashboard.putNumber("TX: ", LimelightHelpers.getTX("limelight"));
    SmartDashboard.putNumber("TY: ", LimelightHelpers.getTY("limelight"));
    SmartDashboard.putNumber("Distance to Tag: ", distanceToTowerTag);
    SmartDashboard.putNumber("Limelight ID", id);
  }

  @Override
  public void teleopInit() {
    leftEncoder.reset();
    rightEncoder.reset();
    climbEncoder.setOffsetDegrees(360 - 60);
    Intake = false;
    Shooter = false;
  }

  @Override
  public void teleopPeriodic() {
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();

    AM_Encoder_Telemetry telemetry = climbEncoder.getTelemetry();
    double degree = climbEncoder.getAngleDegrees();
    double forward = -joystick.getLeftY();
    double rotation = joystick.getRightX();

    leftTrigger = joystick1.getLeftTriggerAxis();
    rightTrigger = joystick1.getRightTriggerAxis();

    if (Math.abs(forward) < 0.05)
      forward = 0;
    if (Math.abs(rotation) < 0.05)
      rotation = 0;

    boolean drivingStraight = Math.abs(forward) > 0.05 && Math.abs(rotation) < 0.05;

    if (drivingStraight) {
      if (!gyroAssistEnabled) {
        gyroAssistEnabled = true;
        targetAngle = m_gyro.getYaw();
        gyroPID.setSetpoint(targetAngle);
      }
      gyroCorrection = gyroPID.calculate(m_gyro.getYaw());
      gyroCorrection = Math.max(-0.3, Math.min(0.3, gyroCorrection));
    } else {

      gyroAssistEnabled = false;
      gyroCorrection = 0;
    }

    double leftSpeed = forward + rotation + gyroCorrection;
    double rightSpeed = forward - rotation - gyroCorrection;

    leftSpeed = Math.max(-maxFwd, Math.min(maxFwd, leftSpeed));
    rightSpeed = Math.max(-maxFwd, Math.min(maxFwd, rightSpeed));

    leftLeader.set(leftSpeed);
    rightLeader.set(rightSpeed);

    // Input
    if (joystick1.getLeftBumperButtonPressed()) {
      Intake = true;
    }

    if (joystick1.getLeftBumperButtonReleased()) {
      Intake = false;
    }

    if (joystick1.getRightBumperButtonPressed()) {
      Shooter = true;
    }

    if (joystick1.getRightBumperButtonReleased()) {
      Shooter = false;
    }

    // if (joystick.getXButtonPressed()){
    // shooterRunning = true;
    // }

    // if (joystick.getXButtonReleased()) {
    // shooterRunning = false;
    // }

    // if (joystick.getYButtonPressed()) {
    // indexerReverseRunning = true;
    // }

    // if (joystick.getYButtonReleased()) {
    // indexerReverseRunning = false;
    // }

    if (Intake) {
      inputLeader.set(0.5);
      indexer.set(-0.6);
    } else if (Shooter) {
      inputLeader.set(1);
      indexer.set(0.8);
    } else {
      indexer.set(0);
      inputLeader.set(0);
    }

    if (leftTrigger >= 0.5) {
      if (degree <= 10) {
        climb.set(0);
      } else if (degree <= 40) {
        double scale = (degree - 10.0) / (40 - 10);
        climb.set(leftTrigger * Math.max(0.1, scale));
      } else {
        climb.set(leftTrigger);
      }
    } else if (rightTrigger >= 0.5) {
      if (degree >= 350) {
        climb.set(0);
      } else if (degree >= 320) {
        double scale = (350 - degree) / (350 - 320);
        climb.set(-rightTrigger * Math.max(0.1, scale));
      } else {
        climb.set(-rightTrigger);
      }
    } else {
      climb.set(0);
    }
  }

   private double angleDifference(double current, double target) {
    double diff = target - current;
    while (diff > 180)  diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
  }

  public void leftAuto() {
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    distance = Math.abs((leftDistance + rightDistance) / 2);

    // init variables
    double inputSpeed = 0.0;
    double indexerSpeed = 0.0;
    forwardSpeed = 0;
    turnSpeed = 0;

    

    // STEP 0: drive backwards
    if (autoStep == 0) {
      if (distance < 30) {
        forwardSpeed = -0.25;
      } else {
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();

        turnTimer.reset();
        turnTimer.start();
      }
    }

    // STEP 1: turn 45 degrees
    else if (autoStep == 1) {
      if (m_gyro.getYaw() < 43 && turnTimer.get() < 3.0) {
        turnSpeed = 0.25;
      } else {
        turnSpeed = 0.0;
        autoStep++;
      }
    }

    // STEP 2: shoot preload
    else if (autoStep == 2) {
      if (!shootTimerStarted) {
        shootTimer.reset();
        shootTimer.start();
        shootTimerStarted = true;
      }
      inputSpeed = 1.0;
      indexerSpeed = 0.8;

      if (shootTimer.get() > 3) {
        inputSpeed = 0;
        indexerSpeed = 0;
        shootTimerStarted = false;
        autoStep++;

        turnTimer.reset();
        turnTimer.start();
      }
    }

    // STEP 3: turn around to balls
    else if (autoStep == 3) {
      if (m_gyro.getYaw() > -137 && turnTimer.get() < 4.0) {
        turnSpeed = -0.25;
      } else {
        autoStep++;
        turnSpeed = 0.0;
        leftEncoder.reset();
        rightEncoder.reset();
      }
    }

    // STEP 4: drive forward and intake balls
    else if (autoStep == 4) {
      if (distance < 120) {
        forwardSpeed = 0.4;
        inputSpeed = 0.6;
        indexerSpeed = -0.6;
      } else {
        inputSpeed = 0.0;
        indexerSpeed = 0.0;
        autoStep++;

        turnTimer.reset();
        turnTimer.start();
      }
    }

    // STEP 5: turn back toward goal
    else if (autoStep == 5) {
      double remaining = angleDifference(m_gyro.getYaw(), 43.0);
      if (remaining > 2.0 && turnTimer.get() < 4.0) {
        turnSpeed = 0.25;
      } else {
        turnSpeed = 0.0;
        autoStep++;
      }
    }

    // STEP 6: shoot collected balls
    else if (autoStep == 6) {
      if (!shootTimerStarted) {
        shootTimer.reset();
        shootTimer.start();
        shootTimerStarted = true;
      }

      inputSpeed = 1;
      indexerSpeed = 0.8;

      if (shootTimer.get() > 3) {
        inputSpeed = 0;
        indexerSpeed = 0;
        autoStep++;
      }
    }

    else if (autoStep > 6) {
      forwardSpeed = 0;
      turnSpeed = 0;
      inputSpeed = 0;
      indexerSpeed = 0;
    }

    inputLeader.set(inputSpeed);
    indexer.set(indexerSpeed);
  }

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    autoStep = 0;
    shootTimerStarted = false;
    turnTimer.reset();
    turnTimer.start();

    m_gyro.reset();
    leftEncoder.reset();
    rightEncoder.reset();

    climbEncoder.setOffsetDegrees(360-60);
  }

  @Override
  public void autonomousPeriodic() {
    double time = timer.get();

    Optional<Alliance> alliance = DriverStation.getAlliance();
    OptionalInt station = DriverStation.getLocation();

    if (time < 20.0) {
      if (alliance.isPresent() && station.isPresent()) {
        switch (alliance.get()) {
          case Red:
            if (station.getAsInt() == 1) {
              leftAuto();
            } else if (station.getAsInt() == 2) {
              // centerAuto();
            } else if (station.getAsInt() == 3) {
              // rightAuto();
            }
            break;

          case Blue:
            if (station.getAsInt() == 1) {
              leftAuto();
            } else if (station.getAsInt() == 2) {
              // centerAuto();
            } else if (station.getAsInt() == 3) {
              // rightAuto();
            }
            break;
        }
      }
    } else {
      forwardSpeed = 0;
      turnSpeed = 0;
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

