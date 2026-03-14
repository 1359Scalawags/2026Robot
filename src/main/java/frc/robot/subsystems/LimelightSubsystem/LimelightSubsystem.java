package frc.robot.subsystems.LimelightSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings.LEDMode;

public class LimelightSubsystem extends SubsystemBase {

    private final String limelightName;
    Limelight limelight;

    private Optional<LimelightResults> latestResults = Optional.empty();
    public LimelightSubsystem(String name) {
        this.limelightName = name;
        limelight = new Limelight(limelightName);
        Pose3d cameraOffset = new Pose3d(Units.inchesToMeters(10.5),
                            Units.inchesToMeters(13.5),
                            Units.inchesToMeters(6.5),
                            new Rotation3d(0, 0, Units.degreesToRadians(45)));
        
        limelight.getSettings()  //Limelight stuff
             .withLimelightLEDMode(LEDMode.PipelineControl)
             .withCameraOffset(cameraOffset)
             .save();
    }


    private boolean checkResultPresent(){
        if (latestResults.isPresent()) {
            if (latestResults.get().targets_Fiducials != null && latestResults.get().targets_Fiducials.length > 0){
                return true;
            }
        }
        return false;
    }

    /** Returns true if the Limelight sees any valid target (AprilTag). */
    public boolean seesAprilTag() {
        return latestResults.isPresent() && latestResults.get().valid;
    }

    /** 2D horizontal offset in degrees. */
    public double getTX() {
        if (latestResults.isPresent()) {
            if (latestResults.get().targets_Fiducials != null && latestResults.get().targets_Fiducials.length > 0) {
                return latestResults.get().targets_Fiducials[0].tx; 
            }
        }
        return 0.0;
    }

    /** 2D vertical offset in degrees. */
    public double getTY() {
        if (latestResults.isPresent()){
            if (latestResults.get().targets_Fiducials != null && latestResults.get().targets_Fiducials.length > 0){
                return latestResults.get().targets_Fiducials[0].ty;
            }
        }
        return 0.0;
    }

    /** ID of the primary fiducial in view. */
    public int getTagId() {
        if (seesAprilTag() && latestResults.get().targets_Fiducials != null && latestResults.get().targets_Fiducials.length > 0){
            return (int) latestResults.get().targets_Fiducials[0].fiducialID;
        }
        return -1;
    }
    /**
     * Returns the horizontal offset to the AprilTag in degrees.
     * Positive = tag is to the right, Negative = tag is to the left.
     * 0 = tag is centered (robot facing the tag head-on).
     */
    public void setPipeline(int pipeline) {
            // Clamp to a safe range (Limelight supports pipelines 0..9 typically)
            int p = Math.max(0, Math.min(9, pipeline));
            limelight.getSettings().withPipelineIndex(p).save();
            SmartDashboard.putNumber("Limelight/Pipeline", p);
    }

/**
     * Scans the camera, finds the TWO most dominant whitelisted tags based on Target Area (ta).
     * Locks onto the largest tag, and optionally locks onto the second largest ONLY IF 
     * it passes the area threshold (indicating we are looking at a corner).
     */
    public List<Integer> getRelevantWhitelistedTags(List<Integer> whitelist) {
        List<Integer> lockedIds = new ArrayList<>();
        
        if (latestResults.isPresent() && latestResults.get().targets_Fiducials != null) {
            
            int bestId = -1;
            double bestTa = -1.0;
            
            int secondBestId = -1;
            double secondBestTa = -1.0;
            
            //Find the top 2 largest whitelisted tags in a single pass
            // for (var tag : latestResults.get().targets_Fiducials) {
            //     if (whitelist.contains((int) tag.fiducialID)) {
            //         if (tag.ta > bestTa) {
            //             // Move the old best down to second place
            //             secondBestTa = bestTa;
            //             secondBestId = bestId;
                        
            //             // Set the new best
            //             bestTa = tag.ta;
            //             bestId = (int) tag.fiducialID;
            //         } else if (tag.ta > secondBestTa) {
            //             // Set the new second best
            //             secondBestTa = tag.ta;
            //             secondBestId = (int) tag.fiducialID;
            //         }
            //     }
            // }
            
            //Lock onto the tags
            // if (bestId != -1) {
            //     lockedIds.add(bestId); // Always lock the best tag
                
            //     // Only lock the second tag if it is at least 70% as big as the best tag
            //     // (If it's smaller, we are just looking flush at the first tag)
            //     if (secondBestId != -1 && secondBestTa >= (bestTa * 0.70)) {
            //         lockedIds.add(secondBestId);
            //     }
            // }
        }
        return lockedIds;
    }


    /**
     * Takes a list of locked target IDs, finds them on the camera, 
     * and returns the exact mathematical average of their TX values.
     */
    public OptionalDouble getAverageTxOfTags(List<Integer> targetIds) {
        if (latestResults.isPresent() && latestResults.get().targets_Fiducials != null) {
            double totalTx = 0.0;
            int count = 0;
            
            for (var tag : latestResults.get().targets_Fiducials) {
                if (targetIds.contains((int) tag.fiducialID)) {
                    totalTx += tag.tx;
                    count++;
                }
            }
            
            if (count > 0) {
                return OptionalDouble.of(totalTx / count); 
            }
        }
        return OptionalDouble.empty();
    }

    @Override
    public void periodic() {
        latestResults = limelight.getLatestResults();

        SmartDashboard.putBoolean("Limelight/SeesTag", seesAprilTag());
        SmartDashboard.putNumber("Limelight/TagID", getTagId());
        SmartDashboard.putNumber("Limelight/TX", getTX());
        SmartDashboard.putNumber("Limelight/TY", getTY());

        // 2. Extract the 3D pose directly from the target data instead of using the old Helper
        if (seesAprilTag() && latestResults.get().targets_Fiducials != null && latestResults.get().targets_Fiducials.length > 0) {
            
            // Get Robot Pose in Target Space from the primary Fiducial
            Pose3d botPose = latestResults.get().targets_Fiducials[0].getRobotPose_TargetSpace();
            
            SmartDashboard.putNumber("Limelight/3D_X", botPose.getX());
            SmartDashboard.putNumber("Limelight/3D_Y", botPose.getY());
            SmartDashboard.putNumber("Limelight/3D_Z", botPose.getZ());
            SmartDashboard.putNumber("Limelight/3D_Yaw", Math.toDegrees(botPose.getRotation().getZ()));
        }
    }
}