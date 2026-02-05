package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * IntakeSubsystem - FRC Intake Subsystem with 2 motors spinning opposite directions
 * Configured for REV SPARK motor controllers with RPM control
 * Uses modern REVLib configuration API
 */
public class IntakeSubsystem extends SubsystemBase {
  
  // ========== MOTOR CONTROLLERS ==========
  private final SparkMax kickerMotor;
  private final SparkMax starMotor;
  
  // ========== ENCODERS ==========
  private final RelativeEncoder kickerEncoder;
  private final RelativeEncoder starEncoder;
  
  // ========== CLOSED LOOP CONTROLLERS ==========
  private final SparkClosedLoopController kickerPID;
  private final SparkClosedLoopController starPID;
  

  // ========== Member Variables =========
  private Debouncer currentSpikeDebouncer= new Debouncer(0.5) ; 
  /**
   * Creates a new IntakeSubsystem
   */
  public IntakeSubsystem() {
    // Initialize kicker motor (spins one direction) - motor type set in constructor
    kickerMotor = new SparkMax(Constants.Intake.KICKER_MOTOR_CAN_ID, MotorType.kBrushless);
    
    // Initialize star motor (spins opposite direction) - motor type set in constructor
    starMotor = new SparkMax(Constants.Intake.STAR_MOTOR_CAN_ID, MotorType.kBrushless);
    
    // Get encoders
    kickerEncoder = kickerMotor.getEncoder();
    starEncoder = starMotor.getEncoder();
    
    // Get closed loop controllers
    kickerPID = kickerMotor.getClosedLoopController();
    starPID = starMotor.getClosedLoopController();
    
    // Configure both motors using modern API
    configureMotors();
  }
  
  /**
   * Configure both motors using modern REVLib configuration API
   * https://docs.revrobotics.com/revlib/spark/configuring-a-spark
   */
  private void configureMotors() {
    // Create configuration object for kicker motor
    SparkMaxConfig kickerConfig = new SparkMaxConfig();
    kickerConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.Intake.CURRENT_LIMIT)
      .inverted(false)
      .voltageCompensation(12.0)
      .closedLoopRampRate (Constants.Intake.CLOSED_LOOP_RAMP_RATE);
    
    // Configure closed loop control for kicker motor
    kickerConfig.closedLoop
      .p(Constants.Intake.kP)
      .i(Constants.Intake.kI)
      .d(Constants.Intake.kD)
      .outputRange(-1, 1);
    
    // Configure feedforward separately using the new API
    kickerConfig.closedLoop.feedForward
      .kV(Constants.Intake.kFF);
    
    // Apply configuration to kicker motor and persist parameters
    kickerMotor.configure(
      kickerConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
    
    // Create configuration object for star motor
    SparkMaxConfig starConfig = new SparkMaxConfig();
    starConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.Intake.CURRENT_LIMIT)
      .inverted(true) // Inverted to spin opposite direction
      .voltageCompensation(12.0)
      .closedLoopRampRate (Constants.Intake.CLOSED_LOOP_RAMP_RATE);
    
    // Configure closed loop control for star motor
    starConfig.closedLoop
      .p(Constants.Intake.kP)
      .i(Constants.Intake.kI)
      .d(Constants.Intake.kD)
      .outputRange(-1, 1);
    
    // Configure feedforward separately using the new API
    starConfig.closedLoop.feedForward
      .kV(Constants.Intake.kFF);
    
    // Apply configuration to star motor and persist parameters
    starMotor.configure(
      starConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
    
    System.out.println("Intake motors configured successfully for RPM control");
  }
  
  /**
   * Turn on both motors to intake balls at 1600 RPM
   */
  public void intakeOn() {
    kickerPID.setSetpoint(Constants.Intake.INTAKE_SPEED_RPM, ControlType.kVelocity);
    starPID.setSetpoint(Constants.Intake.INTAKE_SPEED_RPM, ControlType.kVelocity);
    System.out.println("Intake ON at " + Constants.Intake.INTAKE_SPEED_RPM + " RPM");
  }
  
  /**
   * Turn off both motors
   */
  public void intakeOff() {
    kickerMotor.stopMotor();
    starMotor.stopMotor();
    System.out.println("Intake OFF");
  }
  
  /**
   * Reverse both motors (for ejecting balls)
   */
  public void intakeReverse() {
    kickerPID.setSetpoint(-Constants.Intake.INTAKE_SPEED_RPM, ControlType.kVelocity);
    starPID.setSetpoint(-Constants.Intake.INTAKE_SPEED_RPM, ControlType.kVelocity);
    System.out.println("Intake REVERSE at -" + Constants.Intake.INTAKE_SPEED_RPM + " RPM");
  }
  
  /**
   * Set custom RPM for both motors
   * @param rpm Target RPM (positive or negative)
   */
  public void setIntakeRPM(double rpm) {
    kickerPID.setSetpoint(rpm, ControlType.kVelocity);
    starPID.setSetpoint(rpm, ControlType.kVelocity);
  }
  
  /**
   * Get the current RPM of the kicker motor
   * @return RPM
   */
  public double getKickerMotorRPM() {
    return kickerEncoder.getVelocity();
  }
  
  /**
   * Get the current RPM of the star motor
   * @return RPM
   */
  public double getStarMotorRPM() {
    return starEncoder.getVelocity();
  }
  
  /**
   * Get the current draw of the kicker motor
   * @return Current in amps
   */
  public double getKickerMotorCurrent() {
    return kickerMotor.getOutputCurrent();
  }
  
  /**
   * Get the current draw of the star motor
   * @return Current in amps
   */
  public double getStarMotorCurrent() {
    return starMotor.getOutputCurrent();
  }
  
  /**
   * Check if motors are drawing excessive current (ball jammed)
   * @return True if current is too high
   */
  public boolean isCurrentSpiking() {
    return getKickerMotorCurrent() > Constants.Intake.CURRENT_THRESHOLD ||
           getStarMotorCurrent() > Constants.Intake.CURRENT_THRESHOLD;
  }
  
  /**
   * Check if motors are at target RPM
   * @return True if both motors are within tolerance
   */
  public boolean isAtTargetSpeed() {
    double kickerError = Math.abs(getKickerMotorRPM() - Constants.Intake.INTAKE_SPEED_RPM);
    double starError = Math.abs(getStarMotorRPM() - Constants.Intake.INTAKE_SPEED_RPM);
    return kickerError < Constants.Intake.RPM_TOLERANCE && starError < Constants.Intake.RPM_TOLERANCE;
  }
  
  /**
   * Periodic function called every 20ms
   */
  @Override
  public void periodic() {
    // Monitor motor currents and log warnings
    boolean spiking = isCurrentSpiking();
    if (currentSpikeDebouncer.calculate(spiking)) {
      System.out.println("WARNING: Intake current spike detected!");
    }
    // SmartDashboard updates for drive 
    SmartDashboard.putBoolean("Intake On", kickerMotor.get() != 0 || starMotor.get() != 0); 
    SmartDashboard.putBoolean("Intake Off", kickerMotor.get() == 0 && starMotor.get() == 0); 
  }
  

}