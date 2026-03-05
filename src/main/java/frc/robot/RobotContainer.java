// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.Climber;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveCommands.AlignToTag;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Kicker;
import frc.robot.subsystems.ShooterSubsystem.Shooter;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Star;
import frc.robot.subsystems.IntakeSubsystem.Sushi;
import frc.robot.subsystems.LimelightSubsystem.LimelightSubsystem;
import frc.robot.commands.SwerveCommands.AlignToTag;
import swervelib.SwerveInputStream;

import java.io.File;
import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.Finishings;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), Constants.swerveDrive.flipper2026));
        private final Star m_IntakeStar = new Star();
        private final Sushi m_IntakeSushi = new Sushi();
        private final Shooter m_Shooter = new Shooter();
        private final Kicker m_Kicker = new Kicker();
        private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
        private final LimelightSubsystem m_limelight = new LimelightSubsystem(Constants.Limelight.limelight_Name);
        private final HopperSubsystem m_HopperSubsystem = new HopperSubsystem();


        private final CommandJoystick m_DriverJoystick = new CommandJoystick(
                        Constants.OperatorConstants.DriverJoystick);
        private final CommandJoystick m_AssistantJoystick = new CommandJoystick(
                        Constants.OperatorConstants.AssistJoystick);

        private boolean isFieldCentric = true;

        private final SendableChooser<Command> autoChooser;
        
        private final DoubleSupplier throttleSupplier = () -> {
                double raw = m_DriverJoystick.getRawAxis(3) * -1;
                double scaled = (raw + 1) / 2.0;
                return MathUtil.clamp(scaled, 0.25, 1.0);
        };

        public RobotContainer() {
                configureBindings();

                // set limelight pipeline (use double-quoted string for Java)
                // Add these imports to your LimelightSubsystem class

                 //Set the Limelight pipeline index.
                 m_limelight.setPipeline(0);

                // Have the autoChooser pull in all PathPlanner autos as options
                autoChooser = AutoBuilder.buildAutoChooser();

                // Set the default auto (do nothing)
                autoChooser.setDefaultOption("Do Nothing", Commands.none());

                SmartDashboard.putData("Auto Chooser", autoChooser);
                SmartDashboard.putData(CommandScheduler.getInstance());


                m_IntakeStar.setDefaultCommand(m_IntakeStar.setStarDutyCylce(0));
                m_IntakeSushi.setDefaultCommand(m_IntakeSushi.setSushiDutyCycle(0));

                m_Shooter.setDefaultCommand(m_Shooter.setShooterDutyCycle(0));
                m_Kicker.setDefaultCommand(m_Kicker.setKickerDutyCylce(0));

                // Default: hold current position with closed-loop (doesn't fight closed-loop mode)
                m_ClimberSubsystem.setDefaultCommand(m_ClimberSubsystem.set(0).withName("ClimberDefault"));

                m_HopperSubsystem.setDefaultCommand(m_HopperSubsystem.set(0));
                SmartDashboard.putData("Field", m_SwerveSubsystem.getSwerveDrive().field);
                SmartDashboard.putBoolean("Field Centric Mode", isFieldCentric);
        }

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is
         * controlled by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_SwerveSubsystem.getSwerveDrive(),
                        () -> m_DriverJoystick.getY() * -1 * throttleSupplier.getAsDouble(),
                        () -> m_DriverJoystick.getX() * -1 * throttleSupplier.getAsDouble())
                        .withControllerRotationAxis(() -> m_DriverJoystick.getZ() * -1 * throttleSupplier.getAsDouble())
                        .deadband(Constants.OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .robotRelative(false)
                        .allianceRelativeControl(true);
        /**
         * Clone's the angular velocity input stream and converts it to a
         * fieldRelative input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(m_DriverJoystick::getX,
                                        m_DriverJoystick::getY)
                        .headingWhile(true);
        /**
         * Clone's the angular velocity input stream and converts it to a
         * robotRelative input stream.
         */
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_SwerveSubsystem.getSwerveDrive(),
                        () -> -m_DriverJoystick.getY(),
                        () -> -m_DriverJoystick.getX())
                        .withControllerRotationAxis(() -> m_DriverJoystick.getRawAxis(
                                        2))
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8);
        SwerveInputStream driveRobotOrientedKeyboard = driveAngularVelocityKeyboard.copy().robotRelative(true)
                        .allianceRelativeControl(false);
        
        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(
                                () -> Math.sin(m_DriverJoystick.getRawAxis(2) * Math.PI)* (Math.PI * 2),
                                () -> Math.cos(m_DriverJoystick.getRawAxis(2) * Math.PI) * (Math.PI * 2))
                        .headingWhile(true)
                        .translationHeadingOffset(true)
                        .translationHeadingOffset(Rotation2d.fromDegrees(0));

   
                                        
        private void configureBindings() {
                // Command driveFieldOrientedDirectAngle = m_SwerveSubsystem.driveFieldOriented(driveDirectAngle);
                // Command driveFieldOrientedAnglularVelocityKeyboard = m_SwerveSubsystem
                //                 .driveFieldOriented(driveAngularVelocityKeyboard);
                // Command driveSetpointGenKeyboard = m_SwerveSubsystem.driveWithSetpointGeneratorFieldRelative(
                //                 driveDirectAngleKeyboard);
                Command driveRobotOrientedAngularVelocity = m_SwerveSubsystem.driveFieldOriented(driveRobotOriented);
                Command driveFieldOrientedAngularVelocity = m_SwerveSubsystem.driveFieldOriented(driveAngularVelocity);

                Command driveFieldOrientedAngularVelocityKeyboard = m_SwerveSubsystem.driveFieldOriented(driveAngularVelocityKeyboard);
                Command driveRobotOrientedAngularVelocityKeyboard = m_SwerveSubsystem.driveFieldOriented(driveRobotOrientedKeyboard);
                // Command driveSetpointGen = m_SwerveSubsystem.driveWithSetpointGeneratorFieldRelative(
                //                 driveDirectAngle);
                Command driveFieldOrientedDirectAngleKeyboard = m_SwerveSubsystem.driveFieldOriented(driveDirectAngleKeyboard);


                // Unified drive command: Chooses the stream based on the toggle and bypasses double-rotation!
                Command unifiedDriveCommand = Commands.run(() -> {
                        if (isFieldCentric) {
                                m_SwerveSubsystem.drive(driveAngularVelocity.get());
                        } else {
                                m_SwerveSubsystem.drive(driveRobotOriented.get());
                        }
                }, m_SwerveSubsystem);
                
                Command unifiedDriveCommandKeyboard = Commands.run(() -> {
                        if (isFieldCentric) {
                                m_SwerveSubsystem.drive(driveAngularVelocityKeyboard.get());
                        } else {
                                m_SwerveSubsystem.drive(driveRobotOrientedKeyboard.get());
                        }
                }, m_SwerveSubsystem);

                // m_DriverJoystick.button(2).onTrue(Commands.runOnce(
                //         () -> m_SwerveSubsystem.zeroGyro()));
              
                // m_AssistantJoystick.button(2).whileTrue(Commands.parallel(
                //                 m_IntakeStar.setStarVelocity(RPM.of(500)),
                //                 m_IntakeSushi.setSushiVelocity(RPM.of(500)).withName("IntakeFuel")));
                

                                
                Command shootFuel = Commands.parallel(
                        m_Shooter.setShooterVelocity(Constants.Shooter.shooterVelocity),
                        m_HopperSubsystem.set(0.75),
                                Commands.sequence(
                                        new WaitCommand(Seconds.of(0.5)),
                                        m_Kicker.setKickerVelocity(Constants.Shooter.kickerVelocity)));

                Command intakeFuel = Commands.parallel(
                                m_IntakeStar.setStarVelocity(Constants.Intake.starVelocity),
                                m_IntakeSushi.setSushiVelocity(Constants.Intake.sushiVelocity))
                                .withName("IntakeFuel");
                                
                Command outtakeFuel = Commands.parallel(
                                m_IntakeStar.setStarVelocity(Constants.Intake.starVelocity.times(1.25).unaryMinus()),
                                m_IntakeSushi.setSushiVelocity(Constants.Intake.sushiVelocity.times(1.25).unaryMinus()))
                                .withName("ReverseIntake");

                Command alignToTag =  new AlignToTag(m_SwerveSubsystem, m_limelight,
                                () -> m_DriverJoystick.getY() * -1 * throttleSupplier.getAsDouble(),
                                () -> m_DriverJoystick.getX() * -1 * throttleSupplier.getAsDouble());

                Command unclogKicker = m_Kicker.setKickerVelocity(Constants.Shooter.kickerVelocity.times(1.5).unaryMinus());
                // Stow: go to stowed height, stop when limit switch is hit
                Command stowSafe = m_ClimberSubsystem.set(-.25).until(m_ClimberSubsystem.limitSwitchSupplier);
                // =========== Set Default Command for swerve ============
                if (RobotBase.isSimulation()) {
                        m_SwerveSubsystem.setDefaultCommand(unifiedDriveCommandKeyboard);
                        m_ClimberSubsystem.setDefaultCommand(m_ClimberSubsystem.set(0));

                        m_IntakeStar.setDefaultCommand(m_IntakeStar.setStarDutyCylce(0));
                        m_IntakeSushi.setDefaultCommand(m_IntakeSushi.setSushiDutyCycle(0));

                        m_Shooter.setDefaultCommand(m_Shooter.setShooterDutyCycle(0));
                        m_Kicker.setDefaultCommand(m_Kicker.setKickerDutyCylce(0));
                        m_HopperSubsystem.setDefaultCommand(m_HopperSubsystem.set(0));

                        // Configure driveToPose on the stream backing the default command
                        Pose2d simTarget = new Pose2d(new Translation2d(2, 2),
                                        Rotation2d.fromDegrees(180));
                        m_SwerveSubsystem.getSwerveDrive().field.getObject("targetPose").setPose(simTarget);

                        driveRobotOrientedKeyboard.driveToPose(
                                        () -> simTarget,
                                        new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
                                        new ProfiledPIDController(5, 0, 0,
                                                new Constraints(Units.degreesToRadians(360),
                                                                Units.degreesToRadians(180))));

                        // Button 2: hold to drive-to-pose, release to resume joystick
                        m_DriverJoystick.button(2).whileTrue(Commands.runEnd(
                                        () -> driveRobotOrientedKeyboard.driveToPoseEnabled(true),
                                        () -> driveRobotOrientedKeyboard.driveToPoseEnabled(false)));

                        m_DriverJoystick.trigger().onTrue(Commands.runOnce(
                                        () -> m_SwerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));

                        m_DriverJoystick.button(11).onTrue(Commands.runOnce(
                                        () -> m_SwerveSubsystem.zeroGyro()));
                        m_DriverJoystick.button(6).whileTrue(alignToTag);

                } else if (RobotBase.isReal()) {
                        m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

                        m_IntakeStar.setDefaultCommand(m_IntakeStar.setStarDutyCylce(0));
                        m_IntakeSushi.setDefaultCommand(m_IntakeSushi.setSushiDutyCycle(0));

                        m_Shooter.setDefaultCommand(m_Shooter.setShooterDutyCycle(0));
                        m_Kicker.setDefaultCommand(m_Kicker.setKickerDutyCylce(0));
                        m_HopperSubsystem.setDefaultCommand(m_HopperSubsystem.set(0));
                        //===============================DRIVE TO POSE ===============================
                        m_SwerveSubsystem.setDefaultCommand(unifiedDriveCommand);

                        Pose2d target = new Pose2d(new Translation2d(0, 0),
                                        Rotation2d.fromDegrees(90));

                        m_SwerveSubsystem.getSwerveDrive().field.getObject("targetPose").setPose(target);

                        driveAngularVelocity.driveToPose(() -> target,
                                        new ProfiledPIDController(5,0,0,
                                                new Constraints(5, 2)),
                                        new ProfiledPIDController(5,0,0,
                                                new Constraints(Units.degreesToRadians(360),
                                                                        Units.degreesToRadians(180))));

                        m_DriverJoystick.button(11).onTrue(Commands.runOnce(
                                () -> m_SwerveSubsystem.zeroGyro()));

                        // m_DriverJoystick.button(5).whileTrue(Commands.runEnd(
                        //         () -> driveAngularVelocity.driveToPoseEnabled(true),
                        //         () -> driveAngularVelocity.driveToPoseEnabled(false)));

                } else if (DriverStation.isTest()) {
        }
                m_AssistantJoystick.button(2).whileTrue(intakeFuel);

                // Hold button 4 to reverse the intake

                m_AssistantJoystick.button(4).whileTrue(outtakeFuel);
                
                m_AssistantJoystick.trigger().whileTrue(shootFuel);


                // Climber: button 8 = retract (stow), button 9 = extend (climb)
                // Uses closed-loop position control with soft limits
                // m_AssistantJoystick.button(7).whileTrue(stowSafe);
                // m_AssistantJoystick.button(8).whileTrue(m_ClimberSubsystem.extend());
                m_AssistantJoystick.button(16).whileTrue(stowSafe);
                m_AssistantJoystick.button(15).whileTrue(m_ClimberSubsystem.set(.25));
                // Run while held (reliable; stops when released)
                // m_AssistantJoystick.button(9).onTrue(m_ClimberSubsystem.sysId().withName("ClimberSysId"));
                // m_AssistantJoystick.button(14).whileTrue(m_ClimberSubsystem.setHeight(Meters.of(Inches.of(5).in(Meters))));

                m_AssistantJoystick.button(14).whileTrue(m_HopperSubsystem.set(0.5));
                m_AssistantJoystick.button(3).whileTrue(unclogKicker);

                // m_DriverJoystick.button(5).whileTrue(driveRobotOrientedAngularVelocity);
                m_DriverJoystick.button(5).onTrue(Commands.runOnce(() -> {
                        isFieldCentric = !isFieldCentric;
                        SmartDashboard.putBoolean("Field Centric Mode", isFieldCentric);
                }));

                // m_DriverJoystick.button(6).whileTrue(m_SwerveSubsystem.driveToPose(new Pose2d(new Translation2d(Meter.of(1.524), Meter.of(0)), Rotation2d.fromDegrees(0))));
                // m_DriverJoystick.trigger().onTrue(Commands.runOnce(
                //         () -> m_SwerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));

                  // =========== Set Default Command for swerve ===========

                // if (RobotBase.isReal()) {

                //         // m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

                //         // Pose2d target = new Pose2d(new Translation2d(0, 0),
                //         //                 Rotation2d.fromDegrees(90));

                //         // m_SwerveSubsystem.getSwerveDrive().field.getObject("targetPose").setPose(target);

                //         // driveAngularVelocity.driveToPose(() -> target,
                //         //                 new ProfiledPIDController(5,0,0,
                //         //                         new Constraints(5, 2)),
                //         //                 new ProfiledPIDController(5,0,0,
                //         //                         new Constraints(Units.degreesToRadians(360),
                //         //                                                 Units.degreesToRadians(180))));



                //         // m_DriverJoystick.button(11).onTrue(Commands.runOnce(
                //         //         () -> m_SwerveSubsystem.zeroGyro()));

                //         // m_DriverJoystick.button(5).whileTrue(Commands.runEnd(
                //         //         () -> driveAngularVelocity.driveToPoseEnabled(true),
                //         //         () -> driveAngularVelocity.driveToPoseEnabled(false)));

                // }
        }

        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return autoChooser.getSelected();
        }
}
