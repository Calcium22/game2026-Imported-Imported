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

//Controller Mapping
//Y: Intake/Pickup
//X: Shooter
//B: Reverse intake/shooter (unjam)
//A: Unload fuel
//Analog sticks: Steering
//D-Pad: Hopper control
//Start:Enter callibrate
//Mode: Unusable
//Back: Cycle through shooter arrays
//Left Bumper: 
//  -Normal Cal: Makes variables the same
//  -Shooter Cal:
//  -Intake Cal:
//Right Rumper: 
//  -Normal Cal: Makes variables the same
//  -Shooter Cal:
//  -Intake Cal:

//Left Trigger: Unused
//Right Trigger:Unused

public class Robot extends TimedRobot {

  // Drivetrain Motors
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
  @SuppressWarnings("unused")
  private final RelativeEncoder m_hopperEncoder = m_hopperMotor.getEncoder();
  double hopper_retract = 0;
  double hopper_extend = -0;

  private final SparkMax m_intakemotor = new SparkMax(5, MotorType.kBrushless);
  private final SparkClosedLoopController m_intakePID = m_intakemotor.getClosedLoopController();
  private final RelativeEncoder m_intakeEncoder = m_intakemotor.getEncoder();

  private final SparkFlex m_pusher = new SparkFlex(8, MotorType.kBrushless);
  private final SparkClosedLoopController m_pusherPID = m_pusher.getClosedLoopController();
  private final RelativeEncoder m_pusherEncoder = m_pusher.getEncoder();

  private final SparkFlex m_shooterMotor = new SparkFlex(7, MotorType.kBrushless);
  private final SparkClosedLoopController m_shooterPID = m_shooterMotor.getClosedLoopController();
  private final RelativeEncoder m_shooterEncoder = m_shooterMotor.getEncoder();

  double currentIntakeVelocity = m_intakeEncoder.getVelocity();
  double currentIntakeCurrent = m_intakemotor.getOutputCurrent();

  // auto stall detection variables and timer
  double kStallVelocity = 10.0; // RPM
  double kStallCurrent = 30.0; // Amps
  double kMinPower = 0.1; // Only check if we are actually trying to move
  private final Timer m_stallTimer = new Timer();
  private final double kStallTimeLimit = 0.5; // seconds to wait before reversing

  int callabrationMode = 0; // 0 = normal, 1 = shooter calibration, 2 = intake calibration

  public static int positionIndex = 1;
  public static double position[] = { 2000, 5600, 5000, 6000 }; // Preset shooter RPMs
  public static double kShooterRPM = position[positionIndex]; // Target shooter RPM

  public static double shooter_maxspeed = 0.1; // used for testing,
  public static double intake_maxspeed = 0.1; // used for testing,
  public static double wheel_maxspeed = 0.5;

  // pusher motor variables
  public static double kPusherRPM = 1000; // Target pusher RPM

  // intake speed in percent output
  double intakeRPM = 3406;

  @SuppressWarnings("removal")
  public Robot() {

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

    SparkFlexConfig pusherConfig = new SparkFlexConfig();
    pusherConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0).d(0).outputRange(-1.0, 1.0);
    m_pusher.configure(pusherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig.smartCurrentLimit(60);
    shooterConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0).d(0).velocityFF(0.00018).outputRange(-1.0, 1.0);
    m_shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(40);
    intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0).d(0).outputRange(-1.0, 1.0);
    m_intakemotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

    
  // photonvision code
  PhotonPipelineResult result = Tag.getLatestResult();
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
  }
    // --- Drive ---
    m_robotDrive.arcadeDrive(-m_p1Controller.getLeftY() * -1 * wheel_maxspeed,
        -m_p1Controller.getRightX() * -1 * wheel_maxspeed);

    // callabration mode for shooter and intake

    if (m_p1Controller.getStartButtonPressed()) {
      callabrationMode = (callabrationMode + 1) % 6; // cycle through modes
      if (callabrationMode > 5) {
        callabrationMode = 0; // reset to normal mode after hopper calibration
      }
      if (callabrationMode == 0) {
        System.out.println("Normal Mode");
        wheel_maxspeed = 0.8;
        shooter_maxspeed = 1;
        intake_maxspeed = 1;
      }
      if (callabrationMode == 1) {
        System.out.println("Shooter Calibration Mode");
      }
      if (callabrationMode == 2) {
        System.out.println("Intake Calibration Mode");
      }
      if (callabrationMode == 3) {
        System.out.println("pusher callabration");
      }
      if (callabrationMode == 4) {
        System.out.println("hopper callabration");
      }
      if (callabrationMode == 5) {
        System.out.println("camera callabration");
      }
    }
    if (callabrationMode == 1) {
      if (m_p1Controller.getLeftBumperButtonReleased()) {
        position[positionIndex] -= 100;
        System.out.println("Target Shooter RPM-: " + position[positionIndex]);
      }
      if (m_p1Controller.getLeftBumperButton()) {
        System.out.println("Actual Shooter RPM-: " + m_shooterEncoder.getVelocity());
      }

      if (m_p1Controller.getRightBumperButtonReleased()) {
        position[positionIndex] += 100;
        System.out.println("Target Shooter RPM+: " + position[positionIndex]);
        System.out.println("Actual Shooter RPM+: " + m_shooterEncoder.getVelocity());
      }

    } else if (callabrationMode == 2) {
      if (m_p1Controller.getLeftBumperButtonReleased()) {
        intakeRPM -= 100;
        System.out.println("Target intake RPM-: " + intakeRPM);
      }
      if (m_p1Controller.getRightBumperButtonReleased()) {
        intakeRPM += 100;
        System.out.println("Target intake RPM+: " + intakeRPM);
      }
    } else if (callabrationMode == 3) {
      if (m_p1Controller.getLeftBumperButtonReleased()) {
        kPusherRPM -= 100;
        System.out.println("Pusher RPM: " + kPusherRPM);
        System.out.println("Actual Pusher Velocity-: " + m_pusherEncoder.getVelocity());
      }
      if (m_p1Controller.getRightBumperButtonReleased()) {
        kPusherRPM += 100;
        System.out.println("Pusher RPM: " + kPusherRPM);
        System.out.println("Actual Pusher Velocity+: " + m_pusherEncoder.getVelocity());
      }
      if (m_p1Controller.getLeftStickButton()) {
        wheel_maxspeed = 0;
        intakeRPM = 0;
      }
    } else if (callabrationMode == 4) {
      if (m_p1Controller.getLeftBumperButtonReleased()) {
        hopper_retract -= 0.01;
        System.out.println("Hopper Retract Position: " + hopper_retract);
        System.out.println("Actual Hopper Position-: " + m_hopperEncoder.getPosition());
      }
      if (m_p1Controller.getRightBumperButtonReleased()) {
        hopper_retract += 0.01;
        System.out.println("Hopper Retract Position: " + hopper_retract);
        System.out.println("Actual Hopper Position+: " + m_hopperEncoder.getPosition());
      }
      if (m_p1Controller.getLeftStickButton()) {
        wheel_maxspeed = 0;
        intakeRPM = 0;
      }

    } else if (callabrationMode == 5) {
      if (m_p1Controller.getLeftBumperButtonReleased()) {
        // Capture pre-process camera stream image
        Tag.takeInputSnapshot();

        // Capture post-process camera stream image
        Tag.takeOutputSnapshot();
      }
    }
    // Player 1 controls shooting, inake, and no climbing controls yet
    // --- Cycle shooter preset ---
    if (m_p1Controller.getBackButtonPressed()) {
      positionIndex = (positionIndex + 1) % 4;
      kShooterRPM = position[positionIndex];
    }

    // shootor and intake button logic
    if (m_p1Controller.getYButton()) {
      m_intakePID.setSetpoint(intakeRPM, ControlType.kVelocity); // shooting
      m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
      m_pusherPID.setSetpoint(kPusherRPM, ControlType.kVelocity);

    } else if (m_p1Controller.getXButton()) {
      m_intakePID.setSetpoint(intakeRPM * -1, ControlType.kVelocity); // pickup
      m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
      m_pusherPID.setSetpoint(kPusherRPM, ControlType.kVelocity);

    } else if (m_p1Controller.getAButton()) {
      m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
      m_pusherPID.setSetpoint(kPusherRPM, ControlType.kVelocity);
    } else if (m_p1Controller.getBButton()) {
      m_intakePID.setSetpoint(intakeRPM * -1, ControlType.kVelocity); // unjam
      m_shooterPID.setSetpoint(-5600, ControlType.kVelocity);
      m_pusherPID.setSetpoint(kPusherRPM * -1, ControlType.kVelocity);

    } else {
      m_intakePID.setSetpoint(0, ControlType.kVelocity);
      m_shooterPID.setSetpoint(0, ControlType.kVelocity);
      m_pusherPID.setSetpoint(0, ControlType.kVelocity);
    }

    int pov = m_p1Controller.getPOV();
    if (pov >= 280 || pov <= 80) {
      m_hopperPID.setSetpoint(hopper_retract, ControlType.kPosition);
    } else if (pov >= 100 && pov <= 260) {
      m_hopperPID.setSetpoint(hopper_extend, ControlType.kPosition);
    }

    // System.out.println("Shooter RPM: " + m_shooterEncoder.getVelocity());
    // stall detection (sample current values each loop)
    double currentIntakeVelocity = m_intakeEncoder.getVelocity();
    double currentIntakeCurrent = m_intakemotor.getOutputCurrent();
    if (Math.abs(m_intakemotor.getAppliedOutput()) > kMinPower) {
      if (Math.abs(currentIntakeVelocity) < kStallVelocity && currentIntakeCurrent > kStallCurrent) {
        // Start the timer if it's not running
        if (m_stallTimer.get() == 0) {
          m_stallTimer.start();
        }
        // If we've exceeded the time limit, take action
        if (m_stallTimer.get() > kStallTimeLimit) {
          System.out.println("INTAKE STALLED! Reversing.");
          m_intakemotor.set(-0.5); // reverse
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
      m_intakemotor.set(-0.5);
    }
  } // end teleopPeriodic()

  @Override
  public void testInit() {
    LiveWindow.setEnabled(false);
  }

  @Override
  public void testPeriodic() {
    // --- Drive ---
    m_robotDrive.arcadeDrive(
        -m_p1Controller.getLeftY() * 0.5,
        -m_p1Controller.getRightX() * 0.5);

    // Player 1 controls shooting, climbing
    // cycle shooter preset
    if (m_p1Controller.getBackButtonPressed()) {
      positionIndex = (positionIndex + 1) % 4;
    }
    kShooterRPM = position[positionIndex];

    // --- Shooter & Intake button logic ---
    if (m_p1Controller.getYButton()) {
      m_intakePID.setSetpoint(intakeRPM, ControlType.kVelocity); // shooting
      m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
      m_pusherPID.setSetpoint(kPusherRPM, ControlType.kVelocity);

    } else if (m_p1Controller.getXButton()) {
      m_intakePID.setSetpoint(intakeRPM * -1, ControlType.kVelocity); // pickup
      m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
      m_pusherPID.setSetpoint(kPusherRPM, ControlType.kVelocity);

    } else if (m_p1Controller.getAButton()) {
      m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
      m_pusherPID.setSetpoint(kPusherRPM, ControlType.kVelocity);
    } else if (m_p1Controller.getBButton()) {
      m_intakePID.setSetpoint(intakeRPM * -1, ControlType.kVelocity); // unjam
      m_shooterPID.setSetpoint(-5600, ControlType.kVelocity);
      m_pusherPID.setSetpoint(kPusherRPM * -1, ControlType.kVelocity);

    } else {
      m_intakePID.setSetpoint(0, ControlType.kVelocity);
      m_shooterPID.setSetpoint(0, ControlType.kVelocity);
      m_pusherPID.setSetpoint(0, ControlType.kVelocity);
    }

    int pov = m_p1Controller.getPOV();
    if (pov >= 280 && pov <= 80) {
      m_hopperPID.setSetpoint(hopper_retract, ControlType.kPosition);
    } else if (pov >= 100 && pov <= 260) {
      m_hopperPID.setSetpoint(hopper_extend, ControlType.kPosition);
    }

    // will happen on its own
    if (Math.abs(m_intakemotor.getAppliedOutput()) > kMinPower) {
      if (Math.abs(currentIntakeVelocity) < kStallVelocity && currentIntakeCurrent > kStallCurrent) {

        System.out.println("INTAKE STALLED!");
        m_intakemotor.set(-0.5); // reverse intake to try to clear the jam
      }
    }

    // all controls above are for player 1 testing, player 2 controls are below for
    // testing

  }
}
