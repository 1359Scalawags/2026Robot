package frc.robot.subsystems.LimelightSubsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.networktables.LimelightSettings.LEDMode;

public class LimelightSubsystem extends SubsystemBase {

    private final String limelightName;
    Limelight limelight = new Limelight("limelight-top");

    public LimelightSubsystem(String name) {
        this.limelightName = name;
        Pose3d cameraOffset = new Pose3d(Units.inchesToMeters(10.5),
                            Units.inchesToMeters(13.5),
                            Units.inchesToMeters(6.5),
                            new Rotation3d(0, 0, Units.degreesToRadians(45)));
        
        limelight.getSettings()  //Limelight stuff
             .withLimelightLEDMode(LEDMode.PipelineControl)
             .withCameraOffset(cameraOffset)
             .save();
    }

    /** Returns true if the Limelight sees any valid target (AprilTag). */
    public boolean seesAprilTag() {
        return LimelightHelpers.getTV(limelightName);
    }

    /** 2D horizontal offset in degrees. */
    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    /** 2D vertical offset in degrees. */
    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    /** ID of the primary fiducial in view. */
    public int getTagId() {
        return (int) LimelightHelpers.getFiducialID(limelightName);
    }
    /**
     * Returns the horizontal offset to the AprilTag in degrees.
     * Positive = tag is to the right, Negative = tag is to the left.
     * 0 = tag is centered (robot facing the tag head-on).
     */
    public void setPipeline(int pipeline) {
            // Clamp to a safe range (Limelight supports pipelines 0..9 typically)
            int p = Math.max(0, Math.min(9, pipeline));
            NetworkTableInstance.getDefault()
                            .getTable(limelightName)
                            .getEntry("pipeline")
                            .setNumber(p);
            SmartDashboard.putNumber("Limelight/Pipeline", p);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight/SeesTag", seesAprilTag());
        SmartDashboard.putNumber("Limelight/TagID", getTagId());
        SmartDashboard.putNumber("Limelight/TX", getTX());
        SmartDashboard.putNumber("Limelight/TY", getTY());

        if (seesAprilTag()) {
            Pose3d botPose = LimelightHelpers.getBotPose3d_TargetSpace(limelightName);
            SmartDashboard.putNumber("Limelight/3D_X", botPose.getX());
            SmartDashboard.putNumber("Limelight/3D_Y", botPose.getY());
            SmartDashboard.putNumber("Limelight/3D_Z", botPose.getZ());
            SmartDashboard.putNumber("Limelight/3D_Yaw", Math.toDegrees(botPose.getRotation().getZ()));
        }
    }
}