package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends SubsystemBase {
    
    // Hardware - single motor driving two gearboxes
    private final SparkMax climberMotor;
    
    // Encoder
    private final RelativeEncoder encoder;
    
    // Closed Loop Controller
    private final SparkClosedLoopController controller;
    
    // Constants
 // Change to your CAN ID
    
    private static final double GEAR_RATIO = 125.0;
    private static final int CURRENT_LIMIT = 60; // Amps
    
    // PID Constants (tune these!)
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 0.0;
    
    // Position limits (in rotations of the output shaft)
    private static final double MAX_HEIGHT = 100.0; // Adjust based on your mechanism
    private static final double MIN_HEIGHT = 0.0;
    
    // Speeds
    private static final double MAX_SPEED = 0.8; // 80% max speed
    
    public ClimberSubsystem() {
        // Initialize motor
        climberMotor = new SparkMax(Constants.Climber.climberMotorPort, MotorType.kBrushless);
        
        // Create configuration
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Configure motor
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(CURRENT_LIMIT)
            .inverted(false); // Change if needed
        
        // Configure encoder conversion factors
        // Since one motor drives two gearboxes, the encoder tracks both sides
        config.encoder
            .positionConversionFactor(1.0 / GEAR_RATIO)
            .velocityConversionFactor(1.0 / GEAR_RATIO);
        
        // Configure PID
        config.closedLoop
            .pid(kP, kI, kD)
            .velocityFF(kFF)
            .outputRange(-MAX_SPEED, MAX_SPEED);
        
        // Apply configuration
        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Get encoder and controller
        encoder = climberMotor.getEncoder();
        controller = climberMotor.getClosedLoopController();
        
        // Reset encoder position
        resetEncoder();
    }
    
    /**
     * Set climber speed manually (open loop control)
     * @param speed -1.0 to 1.0 (negative = down, positive = up)
     */
    public void setSpeed(double speed) {
        // Clamp speed
        speed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, speed));
        
        // Safety check: don't go past limits
        if (speed > 0 && getPosition() >= MAX_HEIGHT) {
            stop();
            return;
        }
        if (speed < 0 && getPosition() <= MIN_HEIGHT) {
            stop();
            return;
        }
        
        climberMotor.set(speed);
    }
    
    /**
     * Move to a specific position (closed loop control)
     * @param position Target position in output shaft rotations
     */
    public void setPosition(double position) {
        // Clamp to limits
        position = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, position));
        
        controller.setReference(position, SparkMax.ControlType.kPosition);
    }
    
    /**
     * Extend climber to maximum height
     */
    public void extend() {
        setSpeed(MAX_SPEED);
    }
    
    /**
     * Retract climber to minimum height
     */
    public void retract() {
        setSpeed(-MAX_SPEED);
    }
    
    /**
     * Stop the climber
     */
    public void stop() {
        climberMotor.stopMotor();
    }
    
    /**
     * Get current position of the climber
     * @return Position in output shaft rotations
     */
    public double getPosition() {
        return encoder.getPosition();
    }
    
    /**
     * Get current velocity of the climber
     * @return Velocity in output shaft RPM
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }
    
    /**
     * Check if climber is at maximum height
     */
    public boolean atMaxHeight() {
        return getPosition() >= MAX_HEIGHT - 0.5; // 0.5 rotation tolerance
    }
    
    /**
     * Check if climber is at minimum height
     */
    public boolean atMinHeight() {
        return getPosition() <= MIN_HEIGHT + 0.5;
    }
    
    /**
     * Reset encoder position to zero
     */
    public void resetEncoder() {
        encoder.setPosition(0);
    }
    
    /**
     * Get motor current draw
     */
    public double getCurrent() {
        return climberMotor.getOutputCurrent();
    }
    
    /**
     * Get motor temperature
     */
    public double getTemperature() {
        return climberMotor.getMotorTemperature();
    }
    
    /**
     * Check if motor is drawing excessive current (possible jam/collision)
     * Note: With one motor driving two gearboxes, binding on one side will show up here
     */
    public boolean isStalling() {
        return getCurrent() > CURRENT_LIMIT * 0.9;
    }
    
    /**
     * Check if motor is overheating
     */
    public boolean isOverheating() {
        return getTemperature() > 80.0; // Celsius
    }
    
    @Override
    public void periodic() {
        // Update telemetry
        SmartDashboard.putNumber("Climber Position", getPosition());
        SmartDashboard.putNumber("Climber Velocity", getVelocity());
        SmartDashboard.putNumber("Climber Current", getCurrent());
        SmartDashboard.putNumber("Climber Temperature", getTemperature());
        SmartDashboard.putBoolean("Climber At Max", atMaxHeight());
        SmartDashboard.putBoolean("Climber At Min", atMinHeight());
        SmartDashboard.putBoolean("Climber Stalling", isStalling());
        SmartDashboard.putBoolean("Climber Overheating", isOverheating());
        
        // Safety: Stop if stalling or overheating
        if (isStalling() || isOverheating()) {
            stop();
        }
    }
}