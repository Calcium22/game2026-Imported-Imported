package frc.robot;
//wpilib imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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

  // Use the leaders directly for DifferentialDrive
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);
  
  private final XboxController m_p1Controller = new XboxController(0);
  private final Timer m_timer = new Timer();

  // Turret & Shooter & hopper motors

  private final SparkMax m_hopperMotor = new SparkMax(6, MotorType.kBrushless);
private final SparkClosedLoopController m_hopperPID = m_hopperMotor.getClosedLoopController();
private final RelativeEncoder m_hopperEncoder = m_hopperMotor.getEncoder();
double hopper_retract = 0; 
double hopper_extend = -0; 
   private final SparkFlex m_intakeMotor = new SparkFlex(8, MotorType.kBrushless);
  private final SparkClosedLoopController m_intakePID = m_intakeMotor.getClosedLoopController();
  private final RelativeEncoder m_intakeEncoder = m_intakeMotor.getEncoder();
  
  private final SparkFlex m_shooterMotor = new SparkFlex(7, MotorType.kBrushless);
  private final SparkClosedLoopController m_shooterPID = m_shooterMotor.getClosedLoopController();
  private final RelativeEncoder m_shooterEncoder = m_shooterMotor.getEncoder();

  public static int positionIndex = 1;
  public static double position[] = {2000, 6000, 5000, 6000}; // Preset shooter RPMs for different positions, array starts at 0 index
  public static double kShooterRPM = position[positionIndex]; // Target shooter RPM
   public static int positionIndexPID = 1;
  public static double positionP[] = {1, 2, 3, 4}; // these are example values for PID tuning change them for actual
  public static double positionI[] = {1, 2, 3, 4};
  public static double positionD[] = {1, 2, 3, 4};
  public static double positionVelocity[] = {1, 2, 3, 4};
  public static double P = positionP[positionIndexPID];
  public static double I = positionI[positionIndexPID];
  public static double D = positionD[positionIndexPID];
  public static double Velocity = positionVelocity[positionIndexPID];
 
  @SuppressWarnings("removal")
  public Robot() {
    // 1. Configure Drivetrain
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    
    // Set right side to inverted
    rightConfig.inverted(true); //
    
    // Set up followers via config
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.follow(1); // Follow ID 1
    
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(3); // Follow ID 3

    // Apply configs to hardware
    m_leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // 2. Configure Shooter
    SparkMaxConfig hopperConfig = new SparkMaxConfig();
    
    hopperConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(5).i(0.0).d(0.0).outputRange(-.030, .030);;

    m_hopperPID.setReference(hopper_retract, ControlType.kPosition);

    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0.0).d(0.0).velocityFF(0.00018)
        .outputRange(-1.0, 1.0);
      shooterConfig.closedLoopRampRate(0.5); // 0.5 seconds from neutral to full

        SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0.0).d(0.0)
        .outputRange(-1.0, 1.0).velocityFF(0.0003);
    
    m_hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void autonomousInit() { m_timer.restart(); }

  @Override
  public void autonomousPeriodic() {
  
  }
  public void teleopInit() {}


  @Override
  public void teleopPeriodic() {
  m_robotDrive.arcadeDrive(-m_p1Controller.getLeftY() * 0.5, -m_p1Controller.getRightX() * 0.5);

   
    if (m_p1Controller.getBackButtonPressed()) {
        positionIndex = (positionIndex + 1) % 4;
        kShooterRPM = position[positionIndex];
    }
    // hopper to shooter
    if (m_p1Controller.getYButton()) {
        m_intakePID.setReference(6000, ControlType.kVelocity);
        m_shooterPID.setReference(kShooterRPM, ControlType.kVelocity);
    } 
    // into hopper
    else if (m_p1Controller.getXButton()) { 
        m_intakePID.setReference(-6000, ControlType.kVelocity);
        m_shooterPID.setReference(kShooterRPM, ControlType.kVelocity);
    } 
    // ground to shooter
    else if (m_p1Controller.getAButton()) {
        m_shooterPID.setReference(kShooterRPM, ControlType.kVelocity);
        m_intakePID.setReference(0, ControlType.kVelocity);
    }
    // hopper to ground
    else if (m_p1Controller.getBButton()) {
        m_shooterPID.setReference(-6000, ControlType.kVelocity);
        m_intakePID.setReference(-6000, ControlType.kVelocity);
    }
    else {
        m_shooterPID.setReference(0, ControlType.kVelocity);
        m_intakePID.setReference(0, ControlType.kVelocity);
    }

    // Hopper Logic

    int pov = m_p1Controller.getPOV();
    if (pov == 0) {
        m_hopperPID.setReference(hopper_retract, ControlType.kPosition);
    } else if (pov == 180) {
        m_hopperPID.setReference(hopper_extend, ControlType.kPosition);
    }

    // Bumpers to tune RPM
    if (m_p1Controller.getLeftBumperPressed()) { position[positionIndex] -= 100; }
    if (m_p1Controller.getRightBumperPressed()) { position[positionIndex] += 100; }

// Display current shooter RPM
    System.out.println("Shooter RPM: " + m_shooterEncoder.getVelocity());
  }
}

