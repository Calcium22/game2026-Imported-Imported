//righttrigger pickup
//lefttrigger shoot
//x topload
//B unload from hopper to ground

package frc.robot;

//wpilib imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

//revrobotics imports
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
import com.revrobotics.spark.SparkAbsoluteEncoder;
//banebots 48:1  2 inch usable saft

//Controller Mapping
//Y: pusher/Pickup
//X: Shooter
//B: Reverse pusher/shooter (unjam)
//A: Unload fuel
//Analog sticks: Steering
//D-Pad: Hopper control
//Start:Enter callibrate
//Mode: Unusable
//Back: Cycle through shooter arrays
//Left Bumper: 
//  -Normal Cal: Makes variables the same
//  -Shooter Cal:
//  -pusher Cal:
//Right Rumper: 
//  -Normal Cal: Makes variables the same
//  -Shooter Cal:
//  -pusher Cal:

//Left Trigger: Unused
//Right Trigger:Unused

public class Robot extends TimedRobot {
  public static int positionIndex = 0;
  public static double shooterposition[] = { 5600, 5800, 6000, 6500 }; // Preset shooter RPMs
  public static double pusherposition[] = { 5600, 5800, 6000, 6500 }; // Preset shooter RPMs
  public static double intakeposition[] = { 4000, 5600, 5000, 6000 }; // Preset shooter RPMs
  public static double top_load[] = { 2000, 2000, 2000, 2000 }; // Preset loading RPMs for loading with shooter
  public static double kShooterRPM = shooterposition[positionIndex]; // Target shooter RPM
  public static double kpusherRPM = pusherposition[positionIndex]; // Target shooter RPM
  public static double kintakeRPM = intakeposition[positionIndex]; // Target intake RPM
  public static double kTopLoadRPM = top_load[positionIndex]; // Target top load RPM

  public static double shooter_increment = 100; // RPM increment for calibration
  public static double pusher_increment = 100; // RPM increment for calibration
  public static double intake_increment = 100; // RPM increment for calibration
  public static double top_load_increment = 100; // RPM increment for calibration
  public static double hopper_increment = 5; // degree increment for hopper calibration
  // public static double hopper_increment = 100; // degree increment for hopper
  // calibration

  public static double shooter_factor = 0.1; // used for testing,
  public static double pusher_factor = 0.1; // used for testing,
  public static double wheel_factor = .8;

  // hopper position variables, hopper is in degrees
  double hopperRetract = 115; // in degrees
  double hopperExtend = 58; // in degrees
  double hopperTarget = hopperRetract; // initialization to valid value of retracted

  // Simple proportional speed
  double kSpeed = 0.5; // base speed
  double output = 0;
  // climber variables
  double inchesPerMotorRotation = 0.3927; // (inch diameter * Math.PI) / 16

  private final Timer autotimer = new Timer();
  private final Timer spin_up = new Timer();
  // Drivetrain Motors
  private final SparkMax m_leftLeader = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax m_leftFollower = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax m_rightLeader = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax m_rightFollower = new SparkMax(4, MotorType.kBrushed);
  // Use the leaders directly for DifferentialDrive
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

  private final XboxController m_p1Controller = new XboxController(0);
  private final XboxController m_p2Controller = new XboxController(1);

  private final SparkMax m_pushermotor = new SparkMax(5, MotorType.kBrushless);
  private final SparkClosedLoopController m_pusherPID = m_pushermotor.getClosedLoopController();
  private final RelativeEncoder m_pusherEncoder = m_pushermotor.getEncoder();

  // Turret & Shooter & hopper motors
  private final SparkMax m_hopperMotor = new SparkMax(6, MotorType.kBrushless);
  // private final SparkMax m_hopperMotor = new SparkMax(6, MotorType.kBrushless);
  private final SparkClosedLoopController m_hopperPID = m_hopperMotor.getClosedLoopController();

  // Assuming the White wire pigtail is in DIO Port 0
  private final DutyCycleEncoder hopperEncoder = new DutyCycleEncoder(0);

  private final SparkFlex m_intake = new SparkFlex(7, MotorType.kBrushless);
  private final SparkClosedLoopController m_intakePID = m_intake.getClosedLoopController();
  private final RelativeEncoder m_intakeEncoder = m_intake.getEncoder();

  private final SparkFlex m_shooterMotor = new SparkFlex(8, MotorType.kBrushless);
  private final SparkClosedLoopController m_shooterPID = m_shooterMotor.getClosedLoopController();
  private final RelativeEncoder m_shooterEncoder = m_shooterMotor.getEncoder();

  double currentpusherVelocity = m_pusherEncoder.getVelocity();
  double currentpusherCurrent = m_pushermotor.getOutputCurrent();

  // auto stall detection variables and timer
  double kStallVelocity = 10.0; // RPM
  double kStallCurrent = 30.0; // Amps
  double kMinPower = 0.1; // Only check if we are actually trying to move
  private final Timer m_stallTimer = new Timer();
  private final double kStallTimeLimit = 0.5; // seconds to wait before reversing

  int callabrationMode = 0; // 0 = normal, 1 = shooter calibration, 2 = pusher calibration, 3 = top load
                            // calibration, 4 = intake calibration, 5 = hopper calibration

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

    // 2. Configure Shooter & pusher & Hopper
    SparkMaxConfig hopperConfig = new SparkMaxConfig();
    hopperConfig.smartCurrentLimit(30);
    hopperConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0).d(0).outputRange(-1.0, 1.0);
    m_hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.smartCurrentLimit(60);
    intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0).d(0).outputRange(-1.0, 1.0);
    m_intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig.smartCurrentLimit(60);
    shooterConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0).d(0).velocityFF(0.00018).outputRange(-1.0, 1.0);
    m_shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig pusherConfig = new SparkMaxConfig();
    pusherConfig.smartCurrentLimit(40);
    pusherConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.00025).i(0).d(0).outputRange(-1.0, 1.0);
    m_pushermotor.configure(pusherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // climber config, same for left and right except for motor ID

    /*
     * SparkMaxConfig winchleft_Config = new SparkMaxConfig();
     * winchleft_Config.smartCurrentLimit(30);
     * winchleft_Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
     * .p(0.1).i(0).d(0).outputRange(-1.0, 1.0);
     * winchleft_Config.encoder
     * .positionConversionFactor(inchesPerMotorRotation);
     * winchleft_Config.closedLoop.maxMotion
     * .cruiseVelocity(400) // Replaces maxVelocity
     * .maxAcceleration(200) // Replaces maxAcceleration
     * .allowedClosedLoopError(1);
     * m_winch_left.configure(winchleft_Config, ResetMode.kResetSafeParameters,
     * PersistMode.kPersistParameters);
     * SparkMaxConfig winchright_Config = new SparkMaxConfig();
     * winchright_Config.smartCurrentLimit(30);
     * winchright_Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
     * .p(0.1).i(0).d(0).outputRange(-1.0, 1.0);
     * winchright_Config.encoder
     * .positionConversionFactor(inchesPerMotorRotation);
     * winchright_Config.closedLoop.maxMotion
     * .cruiseVelocity(400)
     * .maxAcceleration(200)
     * .allowedClosedLoopError(1);
     * m_winch_right.configure(winchright_Config, ResetMode.kResetSafeParameters,
     * PersistMode.kPersistParameters);
     * // Configure closed-loop motion parameters for both winch configs
     */
  }

  @Override
  public void autonomousInit() {
    autotimer.restart();
  }

  @Override
  public void autonomousPeriodic() {
    autotimer.start();
    // Drive forward at half speed for 2 seconds, then stop
    if (autotimer.get() < 1.75)

    {
      m_robotDrive.arcadeDrive(-0.5, 0.0);
    } else {
      m_pusherPID.setSetpoint(kpusherRPM * -1, ControlType.kVelocity);
      m_intakePID.setSetpoint(kintakeRPM, ControlType.kVelocity);
    }
    if (autotimer.get() < 10) {
      m_shooterPID.setSetpoint(kShooterRPM * -1, ControlType.kVelocity);
    }
    // Drive forward at half speed for 1.75 seconds, then stop
    /*
     * if (autotimer.get() < 1.75) {
     * m_robotDrive.arcadeDrive(-0.5, 0.0);
     * }
     * if (autotimer.get() >= 1.75 && autotimer.get() < 10.0) {
     * m_pusherPID.setSetpoint(kpusherRPM * -1, ControlType.kVelocity);
     * m_intakePID.setSetpoint(kintakeRPM, ControlType.kVelocity);
     * }
     * if (autotimer.get() < 10) {
     * m_shooterPID.setSetpoint(kShooterRPM * -1, ControlType.kVelocity);
     * }
     * if (autotimer.get() > 10 && autotimer.get() < 11.5) {
     * m_robotDrive.arcadeDrive(0, -.6);
     * m_shooterMotor.stopMotor();
     * }
     * if (autotimer.get() > 11.5 && autotimer.get() < 13.25) {
     * m_robotDrive.arcadeDrive(-.5, 0);
     * }
     * if (autotimer.get() > 13.25 && autotimer.get() < 14.75) {
     * m_robotDrive.arcadeDrive(0, -.6);
     * }
     * if (autotimer.get() > 14.75 && autotimer.get() < 17.25) {
     * m_robotDrive.arcadeDrive(-1, 0);
     * }
     * if (autotimer.get() >= 17.25 && autotimer.get() < 0.0) {
     * }
     */
    // comment about what the code does
    // if (autotimer.get() >= 0.00 && autotimer.get() < 0.0) {
    // //what action needs to happen;
    // }

    /*
     * if (autotimer.get() > 10) {
     * m_robotDrive.stopMotor();
     * m_pushermotor.stopMotor();
     * m_shooterMotor.stopMotor();
     * m_intake.stopMotor();
     * }
     */
  }

  @Override
  public void disabledInit() {
    autotimer.reset();
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

    // --- Drive ---
    m_robotDrive.arcadeDrive(-m_p2Controller.getLeftY() * -1 * wheel_factor,
        -m_p2Controller.getRightX() * -1 * 1);

    // callabration mode for shooter and pusher

    if (m_p2Controller.getStartButtonPressed()) {
      callabrationMode = (callabrationMode + 1) % 6; // cycle through modes
      if (callabrationMode > 5) {
        callabrationMode = 0; // reset to normal mode after hopper calibration
      }
      if (callabrationMode == 0) {
        System.out.println("Competition Mode");
        wheel_factor = 1;
        shooter_factor = 1;
        pusher_factor = 1;
      }
      if (callabrationMode == 1) {
        System.out.println("Shooter Calibration Mode");
      }
      if (callabrationMode == 2) {
        System.out.println("pusher Calibration Mode");
      }
      if (callabrationMode == 3) {
        System.out.println("top load calibration mode");
      }
      if (callabrationMode == 4) {
        System.out.println("intake callabration");
      }
      if (callabrationMode == 5) {
        System.out.println("hopper callabration");
      }
    }
    if (callabrationMode == 1) {
      if (m_p2Controller.getLeftBumperButtonReleased()) {
        shooterposition[positionIndex] -= shooter_increment; // increment = 100 RPM, can be changed at the top of the
                                                             // code
        System.out.println("Target Shooter RPM-: " + shooterposition[positionIndex]);
      }
      if (m_p2Controller.getLeftBumperButton()) {
        System.out.println("Actual Shooter RPM-: " + m_shooterEncoder.getVelocity());
      }

      if (m_p2Controller.getRightBumperButtonReleased()) {
        shooterposition[positionIndex] += shooter_increment; // increment = 100 RPM, can be changed at the top of the
                                                             // code
        System.out.println("Target Shooter RPM+: " + shooterposition[positionIndex]);
        System.out.println("Actual Shooter RPM+: " + m_shooterEncoder.getVelocity());
      }

    } else if (callabrationMode == 2) {
      if (m_p2Controller.getLeftBumperButtonReleased()) {
        pusherposition[positionIndex] -= pusher_increment;// increment = 100 RPM, can be changed at the top of the code
        System.out.println("Target pusher RPM-: " + pusherposition[positionIndex]);
      }
      if (m_p2Controller.getRightBumperButtonReleased()) {
        pusherposition[positionIndex] += pusher_increment;// increment = 100 RPM, can be changed at the top of the code
        System.out.println("Target pusher RPM+: " + pusherposition[positionIndex]);
      }
    } else if (callabrationMode == 3) {
      if (m_p2Controller.getLeftBumperButtonReleased()) {
        top_load[positionIndex] -= top_load_increment;// increment = 100 RPM, can be changed at the top of the code
        System.out.println("Top Load RPM-: " + top_load[positionIndex]);
      }
      if (m_p2Controller.getRightBumperButtonReleased()) {
        top_load[positionIndex] += top_load_increment;// increment = 100 RPM, can be changed at the top of the code
        System.out.println("Top Load RPM+: " + top_load[positionIndex]);
      }
    } else if (callabrationMode == 4) {
      if (m_p2Controller.getLeftBumperButtonReleased()) {
        intakeposition[positionIndex] -= intake_increment;// increment = 100 RPM, can be changed at the top of the code
        System.out.println("intake RPM: " + intakeposition[positionIndex]);
        System.out.println("Actual intake Velocity-: " + m_intakeEncoder.getVelocity());
      }
      if (m_p2Controller.getRightBumperButtonReleased()) {
        intakeposition[positionIndex] += intake_increment;// increment = 100 RPM, can be changed at the top of the code
        System.out.println("intake RPM: " + intakeposition[positionIndex]);
        System.out.println("Actual intake Velocity+: " + m_intakeEncoder.getVelocity());
      }
    } else if (callabrationMode == 5) {
      if (m_p2Controller.getLeftBumperButtonReleased()) {
        hopperExtend -= hopper_increment;
        double hopperDegrees = hopperEncoder.get() * 360.0;
        System.out.println("Hopper Angle: " + hopperDegrees);
        System.out.println("Actual Hopper Position-: " + (hopperEncoder.get() * 360.0));
      }
      if (m_p2Controller.getRightBumperButtonReleased()) {
        hopperExtend += hopper_increment;
        System.out.println("Actual Hopper Position+: " + (hopperEncoder.get() * 360.0));
        double hopperDegrees = hopperEncoder.get() * 360.0;
        System.out.println("Hopper Angle: " + hopperDegrees);
      }
    }

    // Player 1 controls shooting, inake, hopper, and climbing controls
    // --- Cycle shooter preset ---
    if (m_p1Controller.getBackButtonPressed()) {
      positionIndex = (positionIndex + 1) % 4;
      kShooterRPM = shooterposition[positionIndex];
      kpusherRPM = pusherposition[positionIndex];
      kintakeRPM = intakeposition[positionIndex];
    }
    // shootor and pusher button logic
    if (m_p1Controller.getLeftTriggerAxis() > 0.1) { // shooting
      m_pusherPID.setSetpoint(kpusherRPM * -1, ControlType.kVelocity);
      m_shooterPID.setSetpoint(kShooterRPM * -1, ControlType.kVelocity);
      m_intakePID.setSetpoint(kintakeRPM, ControlType.kVelocity);

    } else if (m_p1Controller.getRightTriggerAxis() > 0.1) { // pickup
      m_pusherPID.setSetpoint(kpusherRPM, ControlType.kVelocity);
      m_shooterPID.setSetpoint(kTopLoadRPM * -1, ControlType.kVelocity);
      m_intakePID.setSetpoint(kintakeRPM, ControlType.kVelocity);

    } else if (m_p1Controller.getXButton()) { // top load into hopper
      m_shooterPID.setSetpoint(kTopLoadRPM * -1, ControlType.kVelocity);
      m_intakePID.setSetpoint(kintakeRPM, ControlType.kVelocity);

    } else if (m_p1Controller.getBButton()) {// unlaod fuel from hopper
      m_pusherPID.setSetpoint(kpusherRPM * -1, ControlType.kVelocity);
      m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
      m_intakePID.setSetpoint(kintakeRPM * -1, ControlType.kVelocity);
    } else {
      m_pushermotor.stopMotor();// causes it to coast instead of brake, which is better for the shooter and
      // pusher
      m_shooterMotor.stopMotor();
      m_intake.stopMotor();
      m_hopperMotor.stopMotor();
    }
    // climber below
    // Distance = Rotations × (Spool Diameter × π) / Gear Ratio
    // CALCULATION: (Circumference / Gear Ratio)
    // (2.0 * Math.PI) / 16 = 0.3927
    // our diamiter is .7500 inches
    // circumfrence 2.36 inches
    /*
     * if (m_p1Controller.getLeftTriggerAxis() > 0.1) {
     * // left and right winch are set the same, if you use differetn gearings or
     * // spools you can change the target position for each independently.
     * m_winch_leftPID.setReference(30, ControlType.kMAXMotionPositionControl);
     * m_winch_rightPID.setReference(30, ControlType.kMAXMotionPositionControl);
     * } else if (m_p1Controller.getRightTriggerAxis() > 0.1) {
     * m_winch_leftPID.setReference(0, ControlType.kMAXMotionPositionControl);
     * m_winch_rightPID.setReference(0, ControlType.kMAXMotionPositionControl);
     * }
     */
    // Choose target based on D-Pad 0 = up, 90 = right, 180 = down, 270 = left, -1 =
    // not pressed

    if (m_p1Controller.getAButton()) {
      System.out.println("Extend");
      m_hopperPID.setSetpoint(500, ControlType.kVelocity);
    }
    if (m_p1Controller.getYButton()) {
      System.out.println("Retracting");
      m_hopperPID.setSetpoint(-500, ControlType.kVelocity);
    }

    double currentpusherVelocity = m_pusherEncoder.getVelocity();
    double currentpusherCurrent = m_pushermotor.getOutputCurrent();
    if (Math.abs(m_pushermotor.getAppliedOutput()) > kMinPower) {
      if (Math.abs(currentpusherVelocity) < kStallVelocity && currentpusherCurrent > kStallCurrent) {
        // Start the timer if it's not running
        if (m_stallTimer.get() == 0) {
          m_stallTimer.start();
        }
        // If we've exceeded the time limit, take action
        if (m_stallTimer.get() > kStallTimeLimit) {
          System.out.println("pusher STALLED! Reversing.");
          m_pusherPID.setSetpoint(-3000, ControlType.kVelocity); // reverse
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
      System.out.println("pusher STALLED FOR 0.5 SECONDS!");
      m_pusherPID.setSetpoint(-3000, ControlType.kVelocity); // makes motor reverse to try to clear the jam
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
        -m_p1Controller.getLeftY() * 1.0,
        -m_p1Controller.getRightX() * 1.0);

    // Player 1 controls shooting, climbing
    // cycle shooter preset
    if (m_p1Controller.getBackButtonPressed()) {
      positionIndex = (positionIndex + 1) % 4;
    }
    kShooterRPM = shooterposition[positionIndex];
    kpusherRPM = pusherposition[positionIndex];
    kintakeRPM = intakeposition[positionIndex];

    // --- Shooter & pusher button logic ---
    if (m_p1Controller.getYButton()) {
      m_pusherPID.setSetpoint(kpusherRPM, ControlType.kVelocity); // shooting
      m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
      m_intakePID.setSetpoint(kintakeRPM, ControlType.kVelocity);

    } else if (m_p1Controller.getXButton()) {
      m_pusherPID.setSetpoint(kpusherRPM * -1, ControlType.kVelocity); // pickup
      m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
      m_intakePID.setSetpoint(kintakeRPM, ControlType.kVelocity);

    } else if (m_p1Controller.getAButton()) {
      m_shooterPID.setSetpoint(kShooterRPM, ControlType.kVelocity);
      m_intakePID.setSetpoint(kintakeRPM, ControlType.kVelocity);
    } else if (m_p1Controller.getBButton()) {
      m_pusherPID.setSetpoint(kpusherRPM * -1, ControlType.kVelocity); // unjam
      m_shooterPID.setSetpoint(-5600, ControlType.kVelocity);
      m_intakePID.setSetpoint(kintakeRPM * -1, ControlType.kVelocity);

    } else {
      m_pushermotor.stopMotor();
      m_shooterMotor.stopMotor();
      m_intake.stopMotor();
      m_hopperMotor.stopMotor();
    }

    int pov = m_p1Controller.getPOV();
    if (pov >= 280 && pov <= 80) {
      m_hopperPID.setSetpoint(hopperRetract, ControlType.kPosition);
    } else if (pov >= 100 && pov <= 260) {
      m_hopperPID.setSetpoint(hopperExtend, ControlType.kPosition);
    }

    // will happen on its own
    if (Math.abs(m_pushermotor.getAppliedOutput()) > kMinPower) {
      if (Math.abs(currentpusherVelocity) < kStallVelocity && currentpusherCurrent > kStallCurrent) {

        System.out.println("pusher STALLED!");
        m_pushermotor.set(-0.5); // reverse pusher to try to clear the jam
      }
    }

    // all controls above are for player 1 testing, player 2 controls are below for
    // testing

  }
}
