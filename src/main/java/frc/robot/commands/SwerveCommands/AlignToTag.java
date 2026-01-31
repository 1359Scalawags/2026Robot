// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import frc.robot.subsystems.SwerveSubsystem;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;


public class AlignToTag extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private final SwerveSubsystem m_SwerveSubsystem;
  private double tagID = -1;

  public AlignToTag(boolean isRightScore, SwerveSubsystem m_SwerveSubsystem) {
    xController = new PIDController(Constants.swerveDrive.autoAlign.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.swerveDrive.autoAlign.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.swerveDrive.autoAlign.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    addRequirements(m_SwerveSubsystem);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.swerveDrive.autoAlign.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.swerveDrive.autoAlign.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.swerveDrive.autoAlign.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.swerveDrive.autoAlign.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.swerveDrive.autoAlign.Y_SETPOINT_REEF_ALIGNMENT : -Constants.swerveDrive.autoAlign.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.swerveDrive.autoAlign.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      m_SwerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      m_SwerveSubsystem.drive(new Translation2d(), 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.swerveDrive.autoAlign.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.swerveDrive.autoAlign.POSE_VALIDATION_TIME);
  }
}