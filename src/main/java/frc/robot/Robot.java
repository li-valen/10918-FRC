package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.Optional;
import java.util.OptionalInt;
import java.util.Queue;
// Gryo imports for NavX
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// REV Imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Robot extends TimedRobot {
  //Initiating a new gyro object
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  // PID for gyro correction, tweak kp in case of wobbling or bad correction
  private final PIDController gyroPID = new PIDController(0.02, 0.0, 0.001);
  private double targetAngle = 0.0;
  private boolean gyroAssistEnabled = true;
  // SparkMax objects (MotorType.kBrushless is for NEOs)

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
  Timer timer = new Timer();
  double forwardSpeed;
  double turnSpeed;

  Optional<Alliance> alliance = DriverStation.getAlliance();
  OptionalInt station = DriverStation.getLocation();
  
  private int printCount = 0;

  public Robot() {
    /* Create Config Objects */
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    /* Set Inversions */
    leftConfig.inverted(true);
    rightConfig.inverted(false);

    /* Set Following */
    // On SparkMax, the follower config points to the Leader's ID
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.follow(leftLeader.getDeviceId()); // Follows Left Leader (ID 1)
    
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(rightLeader.getDeviceId()); // Follows Right Leader (ID 3)

    /* Apply Configs */
    // kPersistParameters saves settings even if the robot loses power
    // kResetSafeParameters ensures a clean state before applying
    leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //resets the gyro 
    m_gyro.reset();
    //configures the PID
    gyroPID.setSetpoint(0);
    gyroPID.setTolerance(2);
    gyroPID.enableContinuousInput(-180, 180);
  }

  @Override
  public void robotPeriodic() {
    if (++printCount >= 10) {
      printCount = 0;
      // .get() returns -1.0 to 1.0
      System.out.println("Left Speed: " + leftLeader.get());
      System.out.println("Right Speed: " + rightLeader.get());
      System.out.println("Input Running: " + inputRunning);
      System.out.println("Output Running: " + outputRunning);
      System.out.println("InputReverse Running: " + inputReverseRunning);
      System.out.println("Gyro Yaw:"+ m_gyro.getYaw());
      System.out.println("Gyro Yaw:"+ m_gyro.getYaw());
    }
  }

  @Override
  public void teleopPeriodic() {
    double fwd = -joystick.getLeftY();
    double rot = joystick.getRightX();
    //makes it so that when rotating, it sets the target angle and disable gyro assist
    if(Math.abs(rot) > 0.1){
      targetAngle = m_gyro.getYaw();
      gyroAssistEnabled = false;
    } else if(Math.abs(fwd) > 0.1){
      //if driver wants to drive forward, enable the assist
      gyroAssistEnabled = true;
    } else{
      gyroAssistEnabled = false;
    }
    double gyroCorrection = 0;
    
    if (gyroAssistEnabled) {
      gyroPID.setSetpoint(targetAngle);
      gyroCorrection = gyroPID.calculate(m_gyro.getYaw());
      // Limit correction strength
      gyroCorrection = Math.max(-0.3, Math.min(0.3, gyroCorrection));
    }


    // fwd = Math.signum(fwd) * Math.min(maxFwd, Math.abs(fwd));
    // rot = Math.signum(rot) * Math.min(maxRot, Math.abs(rot));
    double finalRot = rot + gyroCorrection;
    double leftSpeed = fwd + finalRot;
    double rightSpeed = fwd - finalRot;

    leftLeader.set(leftSpeed);
    rightLeader.set(rightSpeed);

    if (joystick.getAButtonPressed()) {
      inputRunning = !inputRunning;
    }

    if (joystick.getBButtonPressed()) {
      outputRunning = !outputRunning;
    }

    if (joystick.getXButtonPressed()) {
      inputReverseRunning = !inputReverseRunning;
    }

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

  @Override
  public void disabledPeriodic() {
    leftLeader.set(0);
    rightLeader.set(0);
  }
} 