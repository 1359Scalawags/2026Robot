// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class AimAtObject extends Command {
    private final SwerveSubsystem m_SwerveSubsystem;
    private DoubleSupplier translationX;
    private DoubleSupplier translationY;
    private Translation2d targetSpeaker;
    private double absoluteHeading;

    @SuppressWarnings("PMD.UnusedPrivateField")

    /**
     * Creates a new ExampleCommand.
     * 
     * @param subsystem The subsystem used by this command.
     */
    public AimAtObject(SwerveSubsystem m_SwerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY) {
        this.m_SwerveSubsystem = m_SwerveSubsystem;
        this.translationX = translationX;
        this.translationY = translationY;
        targetSpeaker = new Translation2d();

        addRequirements(m_SwerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        targetSpeaker = m_SwerveSubsystem.getTargetHub();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    //no clue if this works, hell if I know
    //this somehow works actually, no clue how tbh
    double distanceX = m_SwerveSubsystem.getPose().getX() - targetSpeaker.getX();
    double distanceY = m_SwerveSubsystem.getPose().getY() - targetSpeaker.getY();
    double relativeHeading = Math.toDegrees(Math.atan2(distanceY, distanceX)); 

    absoluteHeading = Math.toRadians(-relativeHeading+270); //the magic sauce
    m_SwerveSubsystem.headingDrive(translationX, translationY, ()-> Math.cos(absoluteHeading), ()-> Math.sin(absoluteHeading));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
