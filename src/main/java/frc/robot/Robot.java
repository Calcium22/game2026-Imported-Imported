package frc.robot;

//wpilib imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

//photonvision imports
import edu.wpi.first.math.geometry.Transform3d;
import java.lang.annotation.Target;
import java.util.List;
import org.photonvision.PhotonCamera; // Added PhotonVision import
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

//revrobotics imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;


public class Robot extends TimedRobot {
  //  Drivetrain Motors
  private final SparkMax m_leftLeader = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax m_leftFollower = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax m_rightLeader = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax m_rightFollower = new SparkMax(4, MotorType.kBrushed);

  
    PhotonCamera Tag = new PhotonCamera("");
    

  // Use the leaders directly for DifferentialDrive
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);
 
  private final XboxController m_p1Controller = new XboxController(0);
  private final XboxController m_p2Controller = new XboxController(1);
  private final Timer m_timer = new Timer();


  // Turret & Shooter & hopper motors
  private final SparkMax m_hopperMotor = new SparkMax(6, MotorType.kBrushless);
  private final SparkClosedLoopController m_hopperPID = m_hopperMotor.getClosedLoopController();
  private final RelativeEncoder m_hopperEncoder = m_hopperMotor.getEncoder();
  double hopper_retract = 0;
  double hopper_extend = -0;


  private final SparkFlex m_intakeMotor = new SparkFlex(8, MotorType.kBrushless);
  private final RelativeEncoder m_intakeEncoder = m_intakeMotor.getEncoder();


  private final SparkFlex m_shooterMotor = new SparkFlex(7, MotorType.kBrushless);
  private final SparkClosedLoopController m_shooterPID = m_shooterMotor.getClosedLoopController();
  private final RelativeEncoder m_shooterEncoder = m_shooterMotor.getEncoder();

    double currentIntakeVelocity = m_intakeEncoder.getVelocity();
    double currentIntakeCurrent = m_intakeMotor.getOutputCurrent();

    double kStallVelocity = 10.0; // RPM
    double kStallCurrent = 30.0;   // Amps
    double kMinPower = 0.1;       // Only check if we are actually trying to move
    private final Timer m_stallTimer = new Timer();
    private final double kStallTimeLimit = 0.5; // seconds to wait before reversing

    public static int positionIndex = 1;
    public static double position[] = {2000, 6000, 5000, 6000}; // Preset shooter RPMs
    public static double kShooterRPM = position[positionIndex]; // Target shooter RPM

    public static double shooter_maxspeed = 0.1; // used for testing,
    public static double intake_maxspeed = 0.1; // used for testing,
  @SuppressWarnings("removal")
  public Robot() {
    // 1. Configure Drivetrain
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
   
    rightConfig.inverted(true);


    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.follow(1);


    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(3);


    m_leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // 2. Configure Shooter & Intake & Hopper
    SparkMaxConfig hopperConfig = new SparkMaxConfig();
    hopperConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0).d(.00001).outputRange(-1, 1);
    m_hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0).d(0).velocityFF(0.00018).outputRange(-1.0, 1.0);
    m_shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0).d(0).velocityFF(0.0003).outputRange(-1.0, 1.0);
    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void autonomousInit() { m_timer.restart(); }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

@Override
public void teleopPeriodic() {


  // photonvision code
  /*PhotonPipelineResult result = Tag.getLatestResult();
  if (result.hasTargets()) {
    List<PhotonTrackedTarget> targets = result.getTargets();
    double yaw = targets.get(0).getYaw();
    double pitch = targets.get(0).getPitch();
    double area = targets.get(0).getArea();
    double skew = targets.get(0).getSkew();

    int targetID = targets.get(0).getFiducialId();
    double poseAmbiguity = targets.get(0).getPoseAmbiguity();
    Transform3d bestCameraToTarget = targets.get(0).getBestCameraToTarget();
    Transform3d alternateCameraToTarget = targets.get(0).getAlternateCameraToTarget();

    if (poseAmbiguity < 0.2) {
      System.out.println("Target ID: " + targetID);
      System.out.println("Yaw: " + yaw);
      System.out.println("Pitch: " + pitch);
      System.out.println("Area: " + area);
      System.out.println("Skew: " + skew);
      System.out.println("Best Camera To Target: " + bestCameraToTarget);
    } else {
      System.out.println("Target ID: " + targetID);
      System.out.println("Pose Ambiguity: " + poseAmbiguity);
      System.out.println("Best Camera To Target: " + bestCameraToTarget);
      System.out.println("Alternate Camera To Target: " + alternateCameraToTarget);
    }
  } else {
    System.out.println("No targets detected.");
  */
  // --- Drive ---
  m_robotDrive.arcadeDrive(-m_p1Controller.getLeftY() * 0.5, -m_p1Controller.getRightX() * 0.5);

  // Player 1 controls shooting, no climbing controls yet
  // --- Cycle shooter preset ---
  if (m_p1Controller.getBackButtonPressed()) {
    positionIndex = (positionIndex + 1) % 4;
    kShooterRPM = position[positionIndex];
  }
  //intake speed in percent output
  double intakePercent = 0;
  //shootor and intake button logic
  if (m_p1Controller.getYButton()) {
    intakePercent = 0.6;  // forward
    m_shooterPID.setReference(kShooterRPM, ControlType.kVelocity);
  } else if (m_p1Controller.getXButton()) {
    intakePercent = -0.6; // reverse
    m_shooterPID.setReference(kShooterRPM, ControlType.kVelocity);
  } else if (m_p1Controller.getAButton()) {
    m_shooterPID.setReference(kShooterRPM, ControlType.kVelocity);
  } else if (m_p1Controller.getBButton()) {
    intakePercent = -0.6;  // reverse intake
    m_shooterPID.setReference(-6000, ControlType.kVelocity);
  } else {
    intakePercent = 0;
    m_shooterPID.setReference(0, ControlType.kVelocity);
  }

  m_intakeMotor.set(intakePercent);

  int pov = m_p1Controller.getPOV();
  if (pov >= 280 || pov <= 80) {
    m_hopperPID.setReference(hopper_retract, ControlType.kPosition);
  } else if (pov >= 100 && pov <= 260) {
    m_hopperPID.setReference(hopper_extend, ControlType.kPosition);
  }

  if (m_p1Controller.getLeftBumperPressed()) position[positionIndex] -= 100;
  if (m_p1Controller.getRightBumperPressed()) position[positionIndex] += 100;

    // stall detection (sample current values each loop)
    double currentIntakeVelocity = m_intakeEncoder.getVelocity();
    double currentIntakeCurrent = m_intakeMotor.getOutputCurrent();
    if (Math.abs(m_intakeMotor.getAppliedOutput()) > kMinPower) {
      if (Math.abs(currentIntakeVelocity) < kStallVelocity && currentIntakeCurrent > kStallCurrent) {
        // Start the timer if it's not running
        if (m_stallTimer.get() == 0) {
          m_stallTimer.start();
        }
        // If we've exceeded the time limit, take action
        if (m_stallTimer.get() > kStallTimeLimit) {
          System.out.println("INTAKE STALLED! Reversing.");
          m_intakeMotor.set(-0.5); // reverse
        }
      } else {
        // Not stalling: Reset the timer
        m_stallTimer.stop();
        m_stallTimer.reset();
      }
    } else {
      // Not running: Reset the timer
      m_stallTimer.stop();
      m_stallTimer.reset();
    }
  
    if (m_stallTimer.hasElapsed(0.5)) {
      System.out.println("INTAKE STALLED FOR 0.5 SECONDS!");
      m_intakeMotor.set(-0.5);
    }
  
  } // end teleopPeriodic()
  
  @Override
  public void testInit() { LiveWindow.setEnabled(false); }


  @Override
  public void testPeriodic() {
        // --- Drive ---
    m_robotDrive.arcadeDrive(
        -m_p1Controller.getLeftY() * 0.5,
        -m_p1Controller.getRightX() * 0.5
     );

// Player 1 controls shooting, climbing
    // cycle shooter preset
    if (m_p1Controller.getBackButtonPressed()) {
      positionIndex = (positionIndex + 1) % 4;
      
    }
    kShooterRPM = position[positionIndex];

  // intake percent
    double intakePercent = 0;


    // --- Shooter & Intake button logic ---
    if (m_p1Controller.getYButton()) {
      intakePercent = 0.6 * intake_maxspeed;  // forward
      m_shooterPID.setReference(kShooterRPM * shooter_maxspeed, ControlType.kVelocity);
    } else if (m_p1Controller.getXButton()) {
      intakePercent = -0.6 * intake_maxspeed; // reverse
      m_shooterPID.setReference(kShooterRPM * shooter_maxspeed, ControlType.kVelocity);
    } else if (m_p1Controller.getAButton()) {
      intakePercent = 0;     // stop intake
      m_shooterPID.setReference(kShooterRPM  * shooter_maxspeed, ControlType.kVelocity);
    } else if (m_p1Controller.getBButton()) {
      intakePercent = -0.6 * intake_maxspeed;  // reverse intake
      m_shooterPID.setReference(-6000 * shooter_maxspeed, ControlType.kVelocity);
    } else {
      intakePercent = 0;
      m_shooterPID.setReference(0, ControlType.kVelocity);
    }
    int pov = m_p1Controller.getPOV();
        if (pov >= 280 && pov <= 80) {
            m_hopperPID.setReference(hopper_retract, ControlType.kPosition);
        } else if (pov >= 100 && pov <= 260) {
            m_hopperPID.setReference(hopper_extend, ControlType.kPosition);
        }
    m_intakeMotor.set(intakePercent);


    

    //will happen on its own
    if (Math.abs(m_intakeMotor.getAppliedOutput()) > kMinPower) {
        if (Math.abs(currentIntakeVelocity) < kStallVelocity && currentIntakeCurrent > kStallCurrent) {

            System.out.println("INTAKE STALLED!");
            System.out.println("Shooter RPM: " + m_shooterEncoder.getVelocity());
            System.out.println("Current Shooter RPM: " + position[positionIndex]);
                    m_intakeMotor.set(-0.5); // reverse intake to try to clear the jam
        }
    }


    // all controls above are for player 1 testing, player 2 controls are below for testing

    if (m_p2Controller.getLeftBumperPressed()){position[positionIndex] -= 100;}
    if (m_p2Controller.getRightBumperPressed()){position[positionIndex] += 100;}

    }
}
