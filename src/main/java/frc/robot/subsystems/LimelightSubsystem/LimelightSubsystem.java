package frc.robot.subsystems.LimelightSubsystem;

import java.util.OptionalDouble;
import frc.robot.subsystems.LimelightSubsystem.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final String m_tableName = "limelight";

    // LED mode constants (same values as the old network table API)
    public static final int LED_DEFAULT = 0;
    public static final int LED_OFF = 1;
    public static final int LED_BLINK = 2;
    public static final int LED_ON = 3;

    public LimelightSubsystem() {
        // give the LEDs a known state
        setLED(false);
    }

    // Raw values from Limelight
    public double getX() {
        return LimelightHelpers.getTX(m_tableName);
    }

    public double getY() {
        return LimelightHelpers.getTY(m_tableName);
    }

    public double getArea() {
        return LimelightHelpers.getTA(m_tableName);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(m_tableName);
    }

    // Controls
    public void setLED(boolean on) {
        if (on) {
            LimelightHelpers.setLEDMode_ForceOn(m_tableName);
        } else {
            LimelightHelpers.setLEDMode_ForceOff(m_tableName);
        }
    }

    /**
     * Direct LED mode write.  See {@link LimelightHelpers} for
     * predefined helpers.
     */
    public void setLEDMode(int mode) {
        switch (mode) {
            case LED_DEFAULT:
                LimelightHelpers.setLEDMode_PipelineControl(m_tableName);
                break;
            case LED_OFF:
                LimelightHelpers.setLEDMode_ForceOff(m_tableName);
                break;
            case LED_BLINK:
                LimelightHelpers.setLEDMode_ForceBlink(m_tableName);
                break;
            case LED_ON:
                LimelightHelpers.setLEDMode_ForceOn(m_tableName);
                break;
            default:
                // fall back to pipeline control
                LimelightHelpers.setLEDMode_PipelineControl(m_tableName);
        }
    }

    public void setPipeline(int id) {
        LimelightHelpers.setPipelineIndex(m_tableName, id);
    }

    public int getPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(m_tableName);
    }

    /**
     * Estimates horizontal distance (meters) to a vision target using camera mounting
     * geometry.
     *
     * @param targetHeightMeters      height of the target (meters)
     * @param limelightHeightMeters   height of the limelight (meters)
     * @param limelightAngleDegrees   mounting angle of the limelight relative to horizontal (degrees)
     * @return OptionalDouble.empty() if no valid target or calculation not possible, otherwise distance in meters
     */
    public OptionalDouble getDistanceMeters(double targetHeightMeters, double limelightHeightMeters, double limelightAngleDegrees) {
        if (!hasTarget()) {
            return OptionalDouble.empty();
        }
        double angleToTargetDeg = getY() + limelightAngleDegrees;
        double angleRad = Math.toRadians(angleToTargetDeg);
        double deltaHeight = targetHeightMeters - limelightHeightMeters;

        double tan = Math.tan(angleRad);
        if (Math.abs(tan) < 1e-6) {
            return OptionalDouble.empty();
        }
        double distance = deltaHeight / tan;
        return OptionalDouble.of(distance);
    }

    @Override
    public void periodic() {
        // Publish useful values to SmartDashboard for tuning/debug
        SmartDashboard.putNumber("Limelight/X", getX());
        SmartDashboard.putNumber("Limelight/Y", getY());
        SmartDashboard.putNumber("Limelight/Area", getArea());
        SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget());
        SmartDashboard.putNumber("Limelight/Pipeline", getPipeline());
        
    }
}