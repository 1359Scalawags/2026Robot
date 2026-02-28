package frc.robot.subsystems.LimelightSubsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.networktables.LimelightPoseEstimator;

public class LimelightSubsystem extends SubsystemBase {

    private final String limelightName;

    public LimelightSubsystem(String name) {
        this.limelightName = name;
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
     * Returns the robot's yaw relative to the AprilTag face in degrees.
     * 0 = facing the tag head-on.
     */
    public double getYawToTag() {
        Pose3d botPose = LimelightHelpers.getBotPose3d_TargetSpace(limelightName);
        return Math.toDegrees(botPose.getRotation().getZ());
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