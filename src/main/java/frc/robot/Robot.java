package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

  private final SparkMax inputLeader = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax inputFollower = new SparkMax(3, MotorType.kBrushless);

  private final SparkMax climb = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax indexer = new SparkMax(1, MotorType.kBrushed);

  private final DutyCycleEncoder climbEncoder = new DutyCycleEncoder(0);

  private final double wheelDiameter = 6.0;
  private final double wheelCircumference = Math.PI * wheelDiameter;
  private final double climbInputTeeth = 10;
  private final double climbOutputTeeth = 28;
  private final double gearRatio = climbOutputTeeth / climbInputTeeth;

  private boolean inputLeaderRunning = false;
  private boolean inputLeaderReverseRunning = false;
  private boolean indexerRunning = false;

  private double maxFwd = 0.7;

  private boolean autoStarted = false;
  private double forwardSpeed = 0;
  private double turnSpeed = 0;

  private double leftTrigger;
  private double rightTrigger;

  private int id;
  private double distanceToTag;

  Timer timer = new Timer();

  private final XboxController joystick = new XboxController(0);

  private int printCount = 0;

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

    inputConfig.inverted(false);
    inputConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.follow(leftLeader.getDeviceId());
    leftFollowerConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(rightLeader.getDeviceId());
    rightFollowerConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig inputFollowerConfig = new SparkMaxConfig();
    inputFollowerConfig.follow(inputLeader.getDeviceId(), true);
    inputFollowerConfig.idleMode(IdleMode.kBrake);


    leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    inputLeader.configure(inputConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    inputFollower.configure(inputFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climb.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  // Auton set 1 - for IDs 12 and 28 (right side)
  // public void autonset1(double time) {
  //   if (time < 0.5) {
  //     forwardSpeed = -0.5;
  //     turnSpeed = 0;
  //   } else if (time < 1.0) {
  //     forwardSpeed = 0;
  //     turnSpeed = -0.5;
  //   } else if (time < 1.5) {
  //     forwardSpeed = 0.5;
  //     turnSpeed = 0;
  //   } else if (time < 2.0) {
  //     forwardSpeed = 0;
  //     turnSpeed = 0.5;
  //   } else if (time < 2.5) {
  //     forwardSpeed = 0;
  //     turnSpeed = 0;
  //     outputRunning = true;
  //   } else if (time < 3.5) {
  //     outputRunning = false;
  //   } else {
  //     forwardSpeed = 0;
  //     turnSpeed = 0;
  //   }
  // }

  // // Auton set 2 - for IDs 7 and 23 (left side)
  // public void autonset2(double time) {
  //   if (time < 0.5) {
  //     forwardSpeed = -0.5;
  //     turnSpeed = 0;
  //   } else if (time < 1.0) {
  //     forwardSpeed = 0;
  //     turnSpeed = 0.5;
  //   } else if (time < 1.5) {
  //     forwardSpeed = 0.5;
  //     turnSpeed = 0;
  //   } else if (time < 2.0) {
  //     forwardSpeed = 0;
  //     turnSpeed = -0.5;
  //   } else if (time < 2.5) {
  //     forwardSpeed = 0;
  //     turnSpeed = 0;
  //     outputRunning = true;
  //   } else if (time < 3.5) {
  //     outputRunning = false;
  //   } else {
  //     forwardSpeed = 0;
  //     turnSpeed = 0;
  //   }
  // }

  // // Auton set 3 - for IDs 9, 10, 25, 26 (middle)
  // public void autonset3(double time) {
  //   if (time < 0.5) {
  //     forwardSpeed = -0.5;
  //     turnSpeed = 0;
  //   } else if (time < 1.0) {
  //     forwardSpeed = 0;
  //     turnSpeed = 0;
  //     outputRunning = true;
  //   } else if (time < 2.0) {
  //     outputRunning = false;
  //   } else {
  //     forwardSpeed = 0;
  //     turnSpeed = 0;
  //   }
  // }

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
      System.out.println("Gyro Assist Enabled: " + gyroAssistEnabled);
      System.out.println("ID detected: " + id);
      System.out.println("Distance (cm): " + distanceToTag * 100);
    }

  }

  @Override
  public void teleopPeriodic() {
    double forward = -joystick.getLeftY();
    double rotation = joystick.getRightX();

    leftTrigger = joystick.getLeftTriggerAxis();
    rightTrigger = joystick.getRightTriggerAxis();
    
    if (Math.abs(forward) < 0.05) forward = 0;
    if (Math.abs(rotation) < 0.05) rotation = 0;

    boolean drivingStraight = forward != 0 && rotation == 0;

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

    double leftSpeed = forward + rotation - gyroCorrection;
    double rightSpeed = forward - rotation + gyroCorrection;

    leftSpeed = Math.max(-maxFwd, Math.min(maxFwd, leftSpeed));
    rightSpeed = Math.max(-maxFwd, Math.min(maxFwd, rightSpeed));

    leftLeader.set(leftSpeed);
    rightLeader.set(rightSpeed);


    // Input
    if (joystick.getAButtonPressed()){
      inputLeaderRunning = true;
      inputLeaderReverseRunning = false;
      
    }

    if (joystick.getBButtonPressed()) {
      indexerRunning = true;
    } 

    if (joystick.getBButtonReleased()) {
      indexerRunning = false;
    }

    if (joystick.getXButtonPressed()){
      inputLeaderReverseRunning = true;
      inputLeaderRunning = false;
      indexerRunning = true;
    }
  
    if (indexerRunning) {
      indexer.set(-0.2);
    } else {
      indexer.set(0);
    }

    if (inputLeaderRunning) {
      inputLeader.set(0.5);
      inputFollower.set(0.5);
    } else if (inputLeaderReverseRunning) {
      inputLeader.set(-0.5);
      inputFollower.set(0);
    } else {
      inputLeader.set(0);
      inputFollower.set(0);
    }

    // Climb 
     if (leftTrigger > 0.5) {
       climb.set(leftTrigger);
     } else if (rightTrigger > 0.5) {
       climb.set(-rightTrigger);  
     }
     else{
      climb.set(0);
     }

  }

  @Override
  public void autonomousInit() {
    autoStarted = false;
    autoStarted = false;
    id = -1;
    forwardSpeed = 0;
    turnSpeed = 0;
    // outputRunning = false;
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
      }
    }

    if (autoStarted) {
      double time = timer.get();

      if (id == 12 || id == 28) {
        //autonset1(time);
      } else if (id == 7 || id == 23) {
        //autonset2(time);
      } else if (id == 9 || id == 10 || id == 25 || id == 26) {
        //autonset3(time);
      } else {
        forwardSpeed = 0;
        turnSpeed = 0;
      }

      // if (outputRunning) {
      //   output.set(0.7);
      // } else {
      //   output.set(0);
      // }

      leftLeader.set(forwardSpeed + turnSpeed);
      rightLeader.set(forwardSpeed - turnSpeed);
    }
  }

  @Override
  public void disabledPeriodic() {
    leftLeader.set(0);
    rightLeader.set(0);
  }
}