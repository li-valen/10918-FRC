package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// REV Imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Robot extends TimedRobot {
  // SparkMax objects (MotorType.kBrushless is for NEOs)
  private final SparkMax leftLeader = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax leftFollower = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax rightLeader = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax rightFollower = new SparkMax(4, MotorType.kBrushed);

  private final SparkMax input = new SparkMax(5, MotorType.kBrushed);
  private final SparkMax output = new SparkMax(6, MotorType.kBrushed);
  private boolean inputRunning = false;
  private boolean outputRunning = false;

  private final PS4Controller joystick = new PS4Controller(0);
  

  private int printCount = 0;

  public Robot() {
    /* Create Config Objects */
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    /* Set Inversions */
    leftConfig.inverted(false);
    rightConfig.inverted(true);

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
    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

  }

  @Override
  public void robotPeriodic() {
    if (++printCount >= 10) {
      printCount = 0;
      // .get() returns -1.0 to 1.0
      System.out.println("Left Speed: " + leftLeader.get());
      System.out.println("Right Speed: " + rightLeader.get());
    }
  }

  @Override
  public void teleopPeriodic() {
    double fwd = -joystick.getLeftY();
    double rot = joystick.getRightX();
 
    double leftSpeed = fwd + rot;
    double rightSpeed = fwd - rot;

    leftLeader.set(leftSpeed);
    rightLeader.set(rightSpeed);

    if (joystick.getSquareButtonPressed()) {
      inputRunning = !inputRunning;
    }

    if (joystick.getCrossButtonPressed()) {
      outputRunning = !outputRunning;
    }

    if (inputRunning) {
      input.set(0.7);
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
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
  // Instead of using multiple lines, you can simply use  DisabledInit and "stopMotor()" so the motors stop instantly.
  @Override
  public void disabledInit()
    {
      m_robotDrive.stopMotor();

  }

}
