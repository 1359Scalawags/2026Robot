// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
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

    private final ProfiledPIDController headingController = new ProfiledPIDController(
            kP, kI, kD,
            new TrapezoidProfile.Constraints(MAX_OMEGA_RAD_PER_SEC, MAX_ALPHA_RAD_PER_SEC2));

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
        // Reset the internal motion profile to “start from where we are right now”
        headingController.reset(swerve.getHeading().getRadians());
    }

    @Override
    public void execute() {
        Rotation2d currentHeading = swerve.getHeading();

        //Get the absolute X/Y coordinates of the Hub (Red or Blue)
        Translation2d hubPos = swerve.getTargetHub();

        //Get our robot's exact X/Y coordinates on the field
        Translation2d robotPos = swerve.getPose().getTranslation();

        //Calculate the exact angle from the robot to the center of the Hub
        Rotation2d desiredHeading = hubPos.minus(robotPos).getAngle();

        //Feed the angle to the ProfiledPIDController
        headingController.setGoal(desiredHeading.getRadians());

        double rotCommand = headingController.calculate(currentHeading.getRadians());
        rotCommand = MathUtil.clamp(rotCommand, -MAX_OMEGA_RAD_PER_SEC, MAX_OMEGA_RAD_PER_SEC);

        if (headingController.atGoal()) {
            rotCommand = 0.0;
        }

        swerve.drive(
                new Translation2d(forwardSupplier.getAsDouble(), strafeSupplier.getAsDouble()),
                rotCommand,
                true);
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