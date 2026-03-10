// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class AlignToTag extends Command {
    private SwerveSubsystem swerve;
    private LimelightSubsystem limelight;
    private DoubleSupplier forwardSupplier;
    private DoubleSupplier strafeSupplier;

    // private static final double MAX_ROT_SPEED = 0.5;
    private static final double MAX_OMEGA_RAD_PER_SEC = 6.0; // start 3-6
    private static final double MAX_ALPHA_RAD_PER_SEC2 = 16.0; // start 8-16
    
    // Profiled PID gains (start conservative, then tune)
    private static final double kP = 4.0;
    private static final double kI = 0.0;
    private static final double kD = 0.2;
    private static final List<Integer> whitelist = new ArrayList<>();

    private final ProfiledPIDController headingController = new ProfiledPIDController(
            kP, kI, kD,
            new TrapezoidProfile.Constraints(MAX_OMEGA_RAD_PER_SEC, MAX_ALPHA_RAD_PER_SEC2));

    private Rotation2d desiredHeading = new Rotation2d();
    private double lastTagTimeSec = -999.0;
    private boolean hasEverSeenTag = false;

    private static final double TAG_LOSS_HOLD_SEC = 0.35; // "sticky" time
    private static final double TX_DEADBAND_DEG = 0.5; // ignore tiny jitter

    // public AlignToTag(SwerveSubsystem swerve, LimelightSubsystem limelight,
    // DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier) {
    // this.swerve = swerve;
    // this.limelight = limelight;
    // this.forwardSupplier = forwardSupplier;
    // this.strafeSupplier = strafeSupplier;

    // // PID to drive yaw to 0 (facing the tag head-on)
    // this.yawPID = new PIDController(1, 0.0, 0.003);
    // this.yawPID.setSetpoint(0.0);
    // this.yawPID.setTolerance(1.5);

    // addRequirements(swerve);
    // }
    public AlignToTag(
            SwerveSubsystem swerve,
            LimelightSubsystem limelight,
            DoubleSupplier forwardSupplier,
            DoubleSupplier strafeSupplier) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;

        // Angles wrap around: -pi and +pi are the same heading
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        // "Close enough" thresholds
        headingController.setTolerance(Math.toRadians(1.5), Math.toRadians(10.0));

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        whitelist.add(10);
        whitelist.add(3);
        whitelist.add(20);
        whitelist.add(25);
        Rotation2d currentHeading = swerve.getHeading();
        desiredHeading = currentHeading;

        hasEverSeenTag = false;
        lastTagTimeSec = -999.0;

        // Reset the internal motion profile to “start from where we are right now”
        headingController.reset(currentHeading.getRadians());
        headingController.setGoal(desiredHeading.getRadians());
    }

    @Override
    public void execute() {
        Rotation2d currentHeading = swerve.getHeading();

        // Update desired heading when we see a tag
        if (limelight.seesAprilTag()) {
            if (whitelist.contains(limelight.getTagId())){
                double txDeg = limelight.getTX();

                if (Math.abs(txDeg) < TX_DEADBAND_DEG)
                    txDeg = 0.0;

                // tx + means tag is to the right -> rotate right -> subtract degrees from
                // current heading
                desiredHeading = currentHeading.minus(Rotation2d.fromDegrees(txDeg));

                lastTagTimeSec = Timer.getFPGATimestamp();
                hasEverSeenTag = true;

                headingController.setGoal(desiredHeading.getRadians());
            }
        }

        // Should we keep holding the last known good goal?
        double now = Timer.getFPGATimestamp();
        boolean shouldHold = hasEverSeenTag && (now - lastTagTimeSec) <= TAG_LOSS_HOLD_SEC;

        double rotCommand = 0.0;

        if (shouldHold) {
            // ProfiledPIDController will automatically “ease in” near the goal
            rotCommand = headingController.calculate(currentHeading.getRadians());

            // Safety clamp (usually not needed because constraints already limit it)
            rotCommand = MathUtil.clamp(rotCommand, -MAX_OMEGA_RAD_PER_SEC, MAX_OMEGA_RAD_PER_SEC);

            if (headingController.atGoal()) {
                rotCommand = 0.0;
            }
        }

        // IMPORTANT NOTE ABOUT UNITS:
        // - If swerve.drive expects radians/sec for rotation: pass rotCommand directly.
        // - If it expects -1..1: use (rotCommand / MAX_OMEGA_RAD_PER_SEC) instead.
        if (whitelist.contains(limelight.getTagId())){
            swerve.drive(
                    new Translation2d(forwardSupplier.getAsDouble(), strafeSupplier.getAsDouble()),
                    rotCommand,
                    true);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until button is released
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true);
    }
}