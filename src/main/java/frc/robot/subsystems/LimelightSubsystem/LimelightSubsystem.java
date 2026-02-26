package frc.robot.subsystems.LimelightSubsystem;

import java.util.OptionalDouble;
import frc.robot.Constants;
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
        if (!LimelightHelpers.getTV(m_tableName)) {
            return OptionalDouble.empty();
        }
        double angleToTargetDeg = LimelightHelpers.getTY(m_tableName) + limelightAngleDegrees;
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
        // --- SIM-INJECTION: only run in simulation to verify helpers ---
    if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
        // fake botpose_targetspace: [x, y, z, roll, pitch, yaw]
        double[] fakePose = new double[] { 1.5, 0.2, 2.0, 0.0, 0.0, 30.0 };
        LimelightHelpers.setLimelightNTDoubleArray(m_tableName, "botpose_targetspace", fakePose);
        LimelightHelpers.setLimelightNTDouble(m_tableName, "tv", 1.0);        // target valid
        LimelightHelpers.setLimelightNTDouble(m_tableName, "tx", 5.0);        // horizontal offset (degrees)
        LimelightHelpers.setLimelightNTDouble(m_tableName, "ty", 5.0);
        LimelightHelpers.setLimelightNTDouble(m_tableName, "tid", 42.0);     // fiducial id
    }

        // Read via helpers and publish to dashboard
        SmartDashboard.putBoolean("Limelight/HasTarget", LimelightHelpers.getTV(m_tableName));
        SmartDashboard.putNumber("Limelight/Tx", LimelightHelpers.getTX(m_tableName));
        SmartDashboard.putNumberArray("Limelight/PoseTargetSpace", LimelightHelpers.getBotPose_TargetSpace(m_tableName));
        SmartDashboard.putNumber("Limelight/FiducialID", LimelightHelpers.getFiducialID(m_tableName));

        // compute distance (OptionalDouble) and publish if present
        OptionalDouble distanceOpt = getDistanceMeters(
            Constants.Limelight.TARGET_HEIGHT_METERS,
            Constants.Limelight.CAMERA_HEIGHT_METERS,
            Constants.Limelight.CAMERA_PITCH_DEGREES
        );
        if (distanceOpt.isPresent()) {
            SmartDashboard.putNumber("Limelight/DistanceMeters", distanceOpt.getAsDouble());
        } else {
            SmartDashboard.putString("Limelight/DistanceMeters", "N/A");
        }

    }
}