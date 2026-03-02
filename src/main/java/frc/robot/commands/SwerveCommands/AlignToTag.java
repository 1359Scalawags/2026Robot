// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightSubsystem;

import java.util.function.DoubleSupplier;

public class AlignToTag extends Command {
    private final SwerveSubsystem swerve;
    private final LimelightSubsystem limelight;
    private final PIDController yawPID;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;

    private static final double MAX_ROT_SPEED = 0.5;

    public AlignToTag(SwerveSubsystem swerve, LimelightSubsystem limelight,
                      DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;

        // PID to drive yaw to 0 (facing the tag head-on)
        this.yawPID = new PIDController(0.03, 0.0, 0.003);
        this.yawPID.setSetpoint(0.0);
        this.yawPID.setTolerance(1.5);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        yawPID.reset();
    }

    @Override
    public void execute() {
        double rotSpeed = 0.0;

        if (limelight.seesAprilTag()) {
            double yaw = limelight.getYawToTag();
            rotSpeed = yawPID.calculate(yaw, 0.0);
            rotSpeed = Math.max(-MAX_ROT_SPEED, Math.min(MAX_ROT_SPEED, rotSpeed));

            SmartDashboard.putNumber("AlignTag/YawError", yaw);
            SmartDashboard.putBoolean("AlignTag/Locked", yawPID.atSetpoint());
        }

        // Driver keeps full translation control, PID overrides rotation
        swerve.drive(
            new Translation2d(forwardSupplier.getAsDouble(), strafeSupplier.getAsDouble()),
            rotSpeed,
            true
        );
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