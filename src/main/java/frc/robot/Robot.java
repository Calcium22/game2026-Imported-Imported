package frc.robot;
//wpilib imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  // Turret & Shooter
  private final SparkMax m_turretMotor = new SparkMax(5, MotorType.kBrushed);
  private final SparkClosedLoopController m_intakePID = m_turretMotor.getClosedLoopController();
  
  private final SparkFlex m_shooterMotor = new SparkFlex(7, MotorType.kBrushless);
  private final SparkClosedLoopController m_shooterPID = m_shooterMotor.getClosedLoopController();
  private final RelativeEncoder m_shooterEncoder = m_shooterMotor.getEncoder();

  public static int positionIndex = 1;
  public static double position[] = {2000, 3900, 5000, 6000}; // Preset shooter RPMs for different positions, array starts at 0 index
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
    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0.0).d(0.0).velocityFF(0.00018)
        .outputRange(-1.0, 1.0);
      shooterConfig.closedLoopRampRate(0.5); // 0.5 seconds from neutral to full

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0.0).d(0.0)
        .outputRange(-1.0, 1.0);
    
    m_shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turretMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void autonomousInit() { m_timer.restart(); }

  @Override
  public void autonomousPeriodic() {
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor();
    }
  }
  public void teleopInit() {}


  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_p1Controller.getLeftY() * 0.5, -m_p1Controller.getRightX() * 0.5);

    if (m_p1Controller.getBackButtonPressed()) {
      positionIndex++;
      if (positionIndex > 3) {
        positionIndex = 0;
      }
      kShooterRPM = position[positionIndex];
    // Use setSetpoint instead of setReference
    //intake mode
    if (m_p1Controller.getYButton()) {
        m_intakePID.setSetpoint(-6000, ControlType.kVelocity);
        m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
    }
    //shooting mode
    else if (m_p1Controller.getXButton()) { 
        m_intakePID.setSetpoint(6000, ControlType.kVelocity);
        m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
    } else if (m_p1Controller.getAButton()) {
      m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
      m_intakePID.setSetpoint(0, ControlType.kVelocity);
    }
    else {
        // Only stop if NO buttons are pressed
        m_shooterPID.setSetpoint(0, ControlType.kVelocity);
        m_intakePID.setSetpoint(0, ControlType.kVelocity);
    }
    
    if (m_p1Controller.getLeftBumper()) {
      position[1] -= 100;
    }
    if (m_p1Controller.getRightBumper()) {
      position[1] += 100;
    }

    // Display current shooter RPM
    System.out.println("Shooter RPM: " + m_shooterEncoder.getVelocity());
  }
}
}
