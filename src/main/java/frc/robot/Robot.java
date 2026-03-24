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
import edu.wpi.first.wpilibj.util.Color;

import com.studica.frc.AHRS;

import static edu.wpi.first.units.Units.Degree;

import java.util.Optional;
import java.util.OptionalInt;

import com.andymark.jni.AM_CAN_HexBoreEncoder;
import com.andymark.jni.AM_CAN_HexBoreEncoder.AM_EncoderStatus;
import com.andymark.jni.AM_CAN_HexBoreEncoder.AM_Encoder_Telemetry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.LEDPattern;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Robot extends TimedRobot {
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kUSB1);
  private final PIDController gyroPID = new PIDController(0.04, 0.0, 0.025);

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

  private double maxFwd = 0.8;

  private double forwardSpeed = 0;
  private double turnSpeed = 0;

  private double leftTrigger;
  private double rightTrigger;

  private int id;
  private double distanceToTowerTag;
  private int autoStep;
  private boolean shootTimerStarted = false;
  private boolean intakeTimerStarted = false;
  private boolean gyroResetDone = false;

  private double climbAngle;
  private double distanceAtShoot = 0;

  double leftDistance;
  double rightDistance;

  // Encoder Distances
  double distance;
  Timer timer = new Timer();
  Timer shootTimer = new Timer();
  Timer intakeTimer = new Timer();
  Timer turnTimer = new Timer();
  Timer climbTimer = new Timer();
  Timer intakeCurrentTimer = new Timer();
  Timer aligntimer = new Timer();
  private double lastRawDegrees = 0;
  private double accumulatedDegrees = 0;
  private double rotationOffset = 0;

  private boolean reverseIntake = false;

  private final XboxController joystick = new XboxController(0);
  private final XboxController joystick1 = new XboxController(1);

  private double leftSpeed;
  private double rightSpeed;
  private boolean climbResetDone = false;

  private boolean alignActive = false;

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
  public void robotInit() {
    lastRawDegrees = climbEncoder.getAngleDegrees();
    accumulatedDegrees = lastRawDegrees;
  }

  @Override
  public void robotPeriodic() {
    AM_Encoder_Telemetry telemetry = climbEncoder.getTelemetry();
    double degree = climbEncoder.getAngleDegrees();
    double inputCurrent = inputLeader.getOutputCurrent();

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

    double currentRawDegrees = climbEncoder.getAngleDegrees();
    double delta = currentRawDegrees - lastRawDegrees;

    if (delta < -180.0) {
      rotationOffset += 360.0;
    }

    else if (delta > 180.0) {
      rotationOffset -= 360.0;
    }

    accumulatedDegrees = currentRawDegrees + rotationOffset;
    lastRawDegrees = currentRawDegrees;

    SmartDashboard.putNumber("Gyro Yaw: ", m_gyro.getYaw());
    SmartDashboard.putBoolean("NavX Connected: ", m_gyro.isConnected());
    SmartDashboard.putNumber("Climb Degrees: ", climbEncoder.getAngleDegrees());
    SmartDashboard.putNumber("TX: ", LimelightHelpers.getTX("limelight"));
    SmartDashboard.putNumber("TY: ", LimelightHelpers.getTY("limelight"));
    SmartDashboard.putNumber("Distance to Tag: ", distanceToTowerTag);
    SmartDashboard.putNumber("Limelight ID", id);
    SmartDashboard.putNumber("Climb Angle", degree);
    SmartDashboard.putNumber("Intake Current", inputCurrent);
    SmartDashboard.putNumber("Left Distance", leftDistance);
    SmartDashboard.putNumber("Right Distance", rightDistance);
    SmartDashboard.putNumber("Continuous Climb Angle", accumulatedDegrees);
  }

  // public void resetClimbTracker() {
  // lastRawDegrees = climbEncoder.getAngleDegrees();
  // rotationOffset = 0;
  // accumulatedDegrees = lastRawDegrees;

  // if (lastRawDegrees < 0 || lastRawDegrees > 0) {
  // climb.set(0.5);
  // }
  // }

  @Override
  public void teleopInit() {
    leftEncoder.reset();
    rightEncoder.reset();
    Intake = false;
    Shooter = false;
    accumulatedDegrees = 0;
  }

  @Override
  public void teleopPeriodic() {
    double degree = climbEncoder.getAngleDegrees();
    double intakeCurrent = inputLeader.getOutputCurrent();

    leftDistance = leftEncoder.getDistance();
    rightDistance = rightEncoder.getDistance();
    int id = getTargetID();

    double forward = -joystick.getLeftY();
    double rotation = (joystick.getRightX() * 0.75);

    climbAngle = degree;

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

    leftSpeed = forward + rotation + gyroCorrection;
    rightSpeed = forward - rotation - gyroCorrection;

    leftSpeed = Math.max(-maxFwd, Math.min(maxFwd, leftSpeed));
    rightSpeed = Math.max(-maxFwd, Math.min(maxFwd, rightSpeed));

    // leftLeader.set(leftSpeed);
    // rightLeader.set(rightSpeed);

    // Input
    if(joystick.getRightBumperButtonPressed()) {
       maxFwd = 1.0;
    } else{
      maxFwd = 0.8;
    }
    if(joystick.getLeftBumperButtonPressed()) {
      rotation = joystick.getRightX() * 0.5;
    }
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

    // In teleopPeriodic:
    if (joystick1.getXButtonPressed())
      alignActive = true;

    if (joystick1.getXButtonReleased())
      alignActive = false;

    if (alignActive)
      alignDistance(id, 0.7);

    // if (joystick1.getAButtonPressed()) {
    // reverseIntake = true;
    // }

    // if (joystick1.getAButtonReleased()) {
    // reverseIntake = false;
    // }

    // Then set motors AFTER:
    leftLeader.set(leftSpeed);
    rightLeader.set(rightSpeed);

    if (Intake) {
      if (intakeCurrent >= 40) {
        intakeCurrentTimer.start();
        if (intakeCurrent >= 40 && intakeCurrentTimer.hasElapsed(0.3))
          inputLeader.set(1);
        indexer.set(-1);
        intakeCurrentTimer.reset();
      } else {
        inputLeader.set(0.5);
        indexer.set(-1);
        intakeCurrentTimer.reset();
      }
    } else if (Shooter) {
      if (intakeCurrent >= 40) {
        intakeCurrentTimer.start();
        if (intakeCurrent >= 10 && intakeCurrentTimer.hasElapsed(0.3)) {
          inputLeader.set(1);
          indexer.set(1);
        }
      } else {
        inputLeader.set(1);
        indexer.set(1);
      }
    } else {
      indexer.set(0);
      inputLeader.set(0);
    }

    // if (reverseIntake == true) {
    // indexer.set(-1);
    // inputLeader.set(-1);
    // } else {
    // inputLeader.set(0);
    // indexer.set(0);
    // }

    // if (leftTrigger >= 0.5) {
    // if (accumulatedDegrees < -150) {
    // climb.set(0);
    // } else {
    // climb.set(leftTrigger);
    // }
    // } else if (rightTrigger >= 0.5) {
    // if (accumulatedDegrees > -150) {
    // climb.set(-rightTrigger);
    // } else if (accumulatedDegrees >= 400) {
    // climb.set(0);
    // }
    // } else {
    // climb.set(0);
    // }

    if (rightTrigger >= 0.5) {
      climb.set(rightTrigger);
    } else if (leftTrigger >= 0.5) {
      climb.set(-leftTrigger);
    } else {
      climb.set(0);
    }
  }
  // if (leftTrigger >= 0.5) {
  // if (degree <= 10 && degree != 0) {
  // climb.set(0);
  // } else if (degree >= 10) {
  // double scale = (degree - 10) / (40 - 10);
  // climb.set(leftTrigger * Math.max(0.1, scale));
  // } else {
  // climb.set(leftTrigger);
  // }
  // } else if (rightTrigger >= 0.5) {
  // if (degree >= 355) {
  // climb.set(0);
  // } else if (degree >= 320) {
  // double scale = (355 - degree) / (355 - 320);
  // climb.set(-rightTrigger * Math.max(0.1, scale));
  // } else {
  // climb.set(-rightTrigger);
  // }
  // } else {
  // climb.set(0);
  // }
  // }

  private boolean alignDistance(int targetTag, double desiredDistance) {
    if (distanceToTowerTag < 0) {
      forwardSpeed = 0;
      // leftSpeed = 0;
      // rightSpeed = 0;
      turnSpeed = 0;
      return false;
    }
    double error = distanceToTowerTag - desiredDistance; // ← flipped
    if (Math.abs(error) > 0.05) {
      double speed = Math.max(-0.6, Math.min(0.6, error * 0.15));
      forwardSpeed = speed;
      double correction = gyroPID.calculate(m_gyro.getYaw());
      turnSpeed = Math.max(-0.2, Math.min(0.2, correction));
      return false;
      // leftSpeed = speed;
      // rightSpeed = speed;
    } else {
      forwardSpeed = 0;
      leftSpeed = 0;
      rightSpeed = 0;
      return true;
    }
  }

  public void leftRedAuto() {
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    distance = Math.abs((leftDistance + rightDistance) / 2);

    double inputSpeed = 0.0;
    double indexerSpeed = 0.0;
    forwardSpeed = 0;
    turnSpeed = 0;

    // STEP 0: drive backwards
    if (autoStep == 0) {
      if (distance < 20) {
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

    // STEP 2: drive towards april tag
    else if (autoStep == 2) {
      if (alignDistance(9, 1)) {
        distanceAtShoot = distance;
        leftEncoder.reset();
        rightEncoder.reset();
        autoStep++;
        turnTimer.reset();
        turnTimer.start();
        m_gyro.reset();
      }
    }

    // STEP 3: rotate 180 degrees for shooting
    else if (autoStep == 3) {
      if (Math.round(m_gyro.getYaw()) < 170) {
        turnSpeed = 0.5;
      } else {
        turnSpeed = 0.0;
        autoStep++;
        m_gyro.reset();
      }
    }

    // STEP 4: shoot preload
    else if (autoStep == 4) {
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
        m_gyro.reset();
      }
    }

    // STEP 5: drive back to starting position
    else if (autoStep == 5) {
      if (distance < distanceAtShoot + 5) {
        forwardSpeed = 0.5;
      } else {
        forwardSpeed = 0;
        autoStep++;
        turnTimer.reset();
        turnTimer.start();
        m_gyro.reset();
      }
    }

    // STEP 6: turn from -135 to -180 to straighten toward balls
    else if (autoStep == 6) {
      if (Math.round(m_gyro.getYaw()) > -45) {
        turnSpeed = -0.5;
      } else {
        turnSpeed = 0.0;
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();
        m_gyro.reset();
      }
    }

    // STEP 7: drive forward and collect balls after 1.5 seconds
    else if (autoStep == 7) {
      if (!intakeTimerStarted) {
        intakeTimer.reset();
        intakeTimer.start();
        intakeTimerStarted = true;
      }
      if (distance < 120) {
        forwardSpeed = 0.4;
        if (intakeTimer.get() > 1.5) {
          inputSpeed = 0.6;
          indexerSpeed = 0.6;
        }
      } else {
        inputSpeed = 0.0;
        indexerSpeed = 0.0;
        intakeTimerStarted = false;
        autoStep++;
        turnTimer.reset();
        turnTimer.start();
        leftEncoder.reset();
        rightEncoder.reset();
        m_gyro.reset();
      }
    }

    // STEP 8: turn 135 left so intake faces tower (-180 to -45)
    else if (autoStep == 8) {
      if (Math.round(m_gyro.getYaw()) < 170) {
        turnSpeed = 0.5;
      } else {
        turnSpeed = 0.0;
        autoStep++;
        turnTimer.reset();
        turnTimer.start();
        m_gyro.reset();
      }
    }

    // STEP 9: align intake side to tag
    else if (autoStep == 9) {
      if (!LimelightHelpers.getTV("limelight") || id != 9) {
        if (distance < 120) {
          forwardSpeed = 0.4;
        } else {
          forwardSpeed = 0;
          autoStep++;
          turnTimer.reset();
          turnTimer.start();
        }
      } else if (alignDistance(9, 1)) {
        autoStep++;
        turnTimer.reset();
        turnTimer.start();
        m_gyro.reset();
      }
    }

    // STEP 10: rotate 180 degrees to face shooter at goal
    else if (autoStep == 10) {
      if (Math.round(m_gyro.getYaw()) < 170) {
        turnSpeed = 0.5;
      } else {
        turnSpeed = 0.0;
        autoStep++;
      }
    }

    // STEP 11: shoot collected balls
    else if (autoStep == 11) {
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

    else if (autoStep > 11) {
      forwardSpeed = 0;
      turnSpeed = 0;
      inputSpeed = 0;
      indexerSpeed = 0;
    }

    inputLeader.set(inputSpeed);
    indexer.set(indexerSpeed);
  }

  public void leftBlueAuto() {
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    distance = Math.abs((leftDistance + rightDistance) / 2);

    double inputSpeed = 0.0;
    double indexerSpeed = 0.0;
    forwardSpeed = 0;
    turnSpeed = 0;

    // STEP 0: drive backwards
    if (autoStep == 0) {
      if (distance < 20) {
        forwardSpeed = -0.25;
      } else {
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();
        turnTimer.reset();
        turnTimer.start();
      }
    }

    // STEP 1: turn -45 degrees
    else if (autoStep == 1) {
      if (m_gyro.getYaw() < 43 && turnTimer.get() < 3.0) {
        turnSpeed = 0.25;
      } else {
        turnSpeed = 0.0;
        autoStep++;
        m_gyro.reset();
      }
    }

    // STEP 2: drive towards april tag
    else if (autoStep == 2) {
      if (alignDistance(25, 1)) {
        distanceAtShoot = distance;
        leftEncoder.reset();
        rightEncoder.reset();
        autoStep++;
        turnTimer.reset();
        turnTimer.start();
        m_gyro.reset();
      }
    }

    // STEP 3: rotate 180 degrees for shooting
    else if (autoStep == 3) {
      if (Math.round(m_gyro.getYaw()) < 170) {
        turnSpeed = 0.5;
      } else {
        turnSpeed = 0.0;
        autoStep++;
        m_gyro.reset();
      }
    }

    // STEP 4: shoot preload
    else if (autoStep == 4) {
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
        m_gyro.reset();
      }
    }

    // STEP 5: drive back to starting position
    else if (autoStep == 5) {
      if (distance < distanceAtShoot + 5) {
        forwardSpeed = 0.4;
      } else {
        forwardSpeed = 0;
        autoStep++;
        turnTimer.reset();
        turnTimer.start();
        m_gyro.reset();
      }
    }

    // STEP 6: turn from 135 to 180 to straighten toward balls
    else if (autoStep == 6) {
      if (Math.round(m_gyro.getYaw()) > -45) {
        turnSpeed = -0.5;
      } else {
        turnSpeed = 0.0;
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();
        m_gyro.reset();
      }
    }

    // STEP 7: drive forward and collect balls after 1.5 seconds
    else if (autoStep == 7) {
      if (!intakeTimerStarted) {
        intakeTimer.reset();
        intakeTimer.start();
        intakeTimerStarted = true;
      }
      if (distance < 120) {
        forwardSpeed = 0.4;
        if (intakeTimer.get() > 1.5) {
          inputSpeed = 0.6;
          indexerSpeed = 0.6;
        }
      } else {
        inputSpeed = 0.0;
        indexerSpeed = 0.0;
        intakeTimerStarted = false;
        autoStep++;
        turnTimer.reset();
        turnTimer.start();
        leftEncoder.reset();
        rightEncoder.reset();
        m_gyro.reset();
      }
    }

    // STEP 8: turn 135 right so intake faces tower
    else if (autoStep == 8) {
      if (Math.round(m_gyro.getYaw()) < 170) {
        turnSpeed = 0.5;
      } else {
        turnSpeed = 0.0;
        autoStep++;
        turnTimer.reset();
        turnTimer.start();
        m_gyro.reset();
      }
    }

    // STEP 9: align intake side to tag
    else if (autoStep == 9) {
      if (!LimelightHelpers.getTV("limelight") || id != 25) {
        if (distance < 120) {
          forwardSpeed = 0.4;
        } else {
          forwardSpeed = 0;
          autoStep++;
          turnTimer.reset();
          turnTimer.start();
          m_gyro.reset();
        }
      } else if (alignDistance(25, 1)) {
        autoStep++;
        turnTimer.reset();
        turnTimer.start();
        m_gyro.reset();
      }
    }

    // STEP 10: rotate 180 degrees to face shooter at goal
    else if (autoStep == 10) {
      if (Math.round(m_gyro.getYaw()) < 170) {
        turnSpeed = 0.5;
      } else {
        turnSpeed = 0.0;
        autoStep++;
        m_gyro.reset();
      }
    }

    // STEP 11: shoot collected balls
    else if (autoStep == 11) {
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

    else if (autoStep > 11) {
      forwardSpeed = 0;
      turnSpeed = 0;
      inputSpeed = 0; 
      indexerSpeed = 0;
    }

    inputLeader.set(inputSpeed);
    indexer.set(indexerSpeed);
  }

  public void centerRedAuto() {
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    distance = Math.abs((leftDistance + rightDistance) / 2);

    double inputSpeed = 0.0;
    double indexerSpeed = 0.0;
    forwardSpeed = 0;
    turnSpeed = 0;

    if (autoStep == 0) {
      if (distance < 30) {
        double correction = gyroPID.calculate(m_gyro.getYaw());
        correction = Math.max(-0.1, Math.min(0.1, correction));
        forwardSpeed = 0.25;
        turnSpeed = correction;
      } else {
        forwardSpeed = 0;
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();
        turnTimer.reset();
        turnTimer.start();
      }
    }

    // STEP 1: Shoot Preload
    else if (autoStep == 1) {
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
        shootTimerStarted = false;
        autoStep++;
        climbTimer.reset();
        climbTimer.start();
      }
    }

    // Step 2: align hooks for climbing
    else if (autoStep == 2) {
      if (climbTimer.get() < 4) {
        climb.set(-0.7);
      } else {
        climb.set(0);
        autoStep++;
        climbTimer.reset();
        climbTimer.start(); 
        aligntimer.reset();
        aligntimer.start();
      }
    }

    // STEP 3: Go to Climb position
    else if (autoStep == 3) {
      if (alignDistance(16, 0.4) || aligntimer.get() > 4) {
        autoStep++;
        climbTimer.reset();
        climbTimer.start();
      }
    }

    // Step 4: climb
    else if (autoStep == 4) {
      if (climbTimer.get() < 5) {
        climb.set(0.65);
      } else {
        climb.set(0);
        autoStep++;
        climbTimer.reset();
        climbTimer.start();
      }
    }
    
    
    else if (autoStep > 4) {
      forwardSpeed = 0;
      turnSpeed = 0;
      inputSpeed = 0;
      indexerSpeed = 0;
      climb.set(0);
    }

    inputLeader.set(inputSpeed);
    indexer.set(indexerSpeed);
  }

    public void centerBlueAuto() {
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    distance = Math.abs((leftDistance + rightDistance) / 2);

    double inputSpeed = 0.0;
    double indexerSpeed = 0.0;
    forwardSpeed = 0;
    turnSpeed = 0;

    if (autoStep == 0) {
      if (distance < 30) {
        double correction = gyroPID.calculate(m_gyro.getYaw());
        correction = Math.max(-0.1, Math.min(0.1, correction));
        forwardSpeed = 0.25;
        turnSpeed = correction;
      } else {
        forwardSpeed = 0;
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();
        turnTimer.reset();
        turnTimer.start();
      }
    }

    // STEP 1: Shoot Preload
    else if (autoStep == 1) {
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
        shootTimerStarted = false;
        autoStep++;
        climbTimer.reset();
        climbTimer.start();
      }
    }

    // Step 2: align hooks for climbing
    else if (autoStep == 2) {
      if (climbTimer.get() < 4) {
        climb.set(-0.7);
      } else {
        climb.set(0);
        autoStep++;
        climbTimer.reset();
        climbTimer.start(); 
        aligntimer.reset();
        aligntimer.start();
      }
    }

    // STEP 3: Go to Climb position
    else if (autoStep == 3) {
      if (alignDistance(32, 0.4) || aligntimer.get() > 4) {
        autoStep++;
        climbTimer.reset();
        climbTimer.start();
      }
    }

    // Step 4: climb
    else if (autoStep == 4) {
      if (climbTimer.get() < 5) {
        climb.set(0.65);
      } else {
        climb.set(0);
        autoStep++;
        climbTimer.reset();
        climbTimer.start();
      }
    }
    
    else if (autoStep > 4) {
      forwardSpeed = 0;
      turnSpeed = 0;
      inputSpeed = 0;
      indexerSpeed = 0;
      climb.set(0);
    }

    inputLeader.set(inputSpeed);
    indexer.set(indexerSpeed);
  }

  public void rightBlueAuto() {
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    distance = Math.abs((leftDistance + rightDistance) / 2);

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
        m_gyro.reset();
      }
    }

    // STEP 1: turn 90 degrees
    else if (autoStep == 1) {
      if (m_gyro.getYaw() < 90 && turnTimer.get() < 4.0) {
        turnSpeed = 0.25;
      } else {
        turnSpeed = 0.0;
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();
      }
    }

    // STEP 2: move forward to get to shooting position
    else if (autoStep == 2) {
      if (distance < 30) {
        forwardSpeed = 0.25;
      } else {
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();
        turnTimer.reset();
        turnTimer.start();
        m_gyro.reset();
      }
    }
    // Step 3: turns 180 degrees to align with tower
    else if (autoStep == 3) {
      if (m_gyro.getYaw() < 90 && turnTimer.get() < 3.0) {
        turnSpeed = -0.25;
      } else {
        turnSpeed = 0.0;
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();
        aligntimer.reset();
        aligntimer.start();
      }
    }

    // STEP 4: align with tower tag for shooting
    else if (autoStep == 4) {
      if (alignDistance(32, 1.75)) {
        autoStep++;
        climbTimer.reset();
        climbTimer.start();
      }
    }

    // STEP 5: shoot preload
    else if (autoStep == 5) {
      if (!shootTimerStarted) {
        shootTimer.reset();
        shootTimer.start();
        shootTimerStarted = true;
      }
      inputSpeed = 0.8;
      indexerSpeed = 0.9;

      if (shootTimer.get() > 3) {
        inputSpeed = 0;
        indexerSpeed = 0;
        shootTimerStarted = false;
        autoStep++;
        climbTimer.reset();
        climbTimer.start();
      }
    }
    // STEP 6: move back climber for 3 seconds
    else if (autoStep == 6) {
      if (climbTimer.get() < 3 && accumulatedDegrees < 600) {
        climb.set(-0.3);
      } else {
        climb.set(0);
        autoStep++;
        climbTimer.reset();
        climbTimer.start();
      }
    }

    // STEP 7: align with tower tag to prepare for climb
    else if (autoStep == 7) {
      if (alignDistance(32, 0.7)) {
        autoStep++;
        climbTimer.reset();
        climbTimer.start();
      }
    }

    // STEP 8: climb
    else if (autoStep == 8) {
      if (climbTimer.get() < 5) {
        climb.set(0.3);
      } else {
        climb.set(0);
        autoStep++;
      }
    }

    else if (autoStep > 8) {
      forwardSpeed = 0;
      turnSpeed = 0;
      inputSpeed = 0;
      indexerSpeed = 0;
      climb.set(0);
    }

    inputLeader.set(inputSpeed);
    indexer.set(indexerSpeed);
  }

  public void rightRedAuto() {
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    distance = Math.abs((leftDistance + rightDistance) / 2);

    double inputSpeed = 0.0;
    double indexerSpeed = 0.0;
    forwardSpeed = 0;
    turnSpeed = 0;

    // STEP 0: drive backwards
    if (autoStep == 0) {
      if (distance < 20) {
        forwardSpeed = -0.25;
      } else {
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();
        turnTimer.reset();
        turnTimer.start();
        m_gyro.reset();
      }
    }

    // STEP 1: turn -90 degrees
    else if (autoStep == 1) {
      if (m_gyro.getYaw() > -80 && turnTimer.get() < 4.0) {
        turnSpeed = -0.15;
      } else {
        turnSpeed = 0.0;
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();
        m_gyro.reset();
      }
    }

    // STEP 2: move forward to get to shooting position
    else if (autoStep == 2) {
      if (distance < 35) {
        forwardSpeed = 0.25;
      } else {
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();
        turnTimer.reset();
        turnTimer.start();
        m_gyro.reset();
      }
    }

    // STEP 3: turn to -177 degrees to align for shooting
    else if (autoStep == 3) {
      if (m_gyro.getYaw() > -80 && turnTimer.get() < 3.0) {
        turnSpeed = -0.15;
      } else {
        turnSpeed = 0.0;
        autoStep++;
        leftEncoder.reset();
        rightEncoder.reset();
        aligntimer.reset();
        aligntimer.start();
      }
    }

    // STEP 4: align with tower tag for shooting
    else if (autoStep == 4) {
      if (aligntimer.get() < 3.0) {
        alignDistance(16, 1.5);
      } else {
        autoStep++;
        climbTimer.reset();
        climbTimer.start();
      }
    }

    // STEP 5: shoot preload
    else if (autoStep == 5) {
      if (!shootTimerStarted) {
        shootTimer.reset();
        shootTimer.start();
        shootTimerStarted = true;
      }
      inputSpeed = 0.8;
      indexerSpeed = 0.7;

      if (shootTimer.get() > 3) {
        inputSpeed = 0;
        indexerSpeed = 0;
        shootTimerStarted = false;
        autoStep++;

        climbTimer.reset();
        climbTimer.start();
      }
    }
    // STEP 6: move back climber for 3 seconds
    else if (autoStep == 6) {
      if (climbTimer.get() < 3 && accumulatedDegrees < 400) {
        climb.set(0.3);
      } else {
        climb.set(0);
        autoStep++;
      }
    }

    // STEP :7 align with tower tag to prepare for climb
    else if (autoStep == 7) {
      if (alignDistance(16, 0.7)) {
        autoStep++;
        climbTimer.reset();
        climbTimer.start();
      }
    }
    // Step 8: Climb
    else if (autoStep == 8) {
      if (climbTimer.get() < 5) {
        climb.set(0.5);
      } else {
        climb.set(0);
        autoStep++;

      }
    } else if (autoStep > 8) {
      forwardSpeed = 0;
      turnSpeed = 0;
      inputSpeed = 0;
      indexerSpeed = 0;
      climb.set(0);
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
    intakeTimerStarted = false;
    turnTimer.reset();
    turnTimer.start();

    // resetClimbTracker();
    gyroPID.reset();
    m_gyro.reset();
    m_gyro.reset();
    leftEncoder.reset();
    rightEncoder.reset();
    targetAngle = 0.0;
    gyroPID.setSetpoint(0);

    // climbEncoder.setOffsetDegrees(360 - 340);

 
  }

  @Override
  public void autonomousPeriodic() {

    double time = timer.get();

    Optional<Alliance> alliance = DriverStation.getAlliance();
    OptionalInt station = DriverStation.getLocation();

    // if (time < 20.0) {
    if (alliance.isPresent() && station.isPresent()) {
      switch (alliance.get()) {
        case Red:
          if (station.getAsInt() == 1) {
            centerRedAuto(); 

          } else if (station.getAsInt() == 2) {
            centerRedAuto();
          } else if (station.getAsInt() == 3) {
            centerRedAuto();
          }
          break;

        case Blue:
          if (station.getAsInt() == 1) {
            centerBlueAuto();
          } else if (station.getAsInt() == 2) {
            centerBlueAuto();
          } else if (station.getAsInt() == 3) {
            centerBlueAuto();
          }
          break;
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