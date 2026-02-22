// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.Climber;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveCommands.AimAtObject;
import frc.robot.commands.SwerveCommands.AlignToTag;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Kicker;
import frc.robot.subsystems.ShooterSubsystem.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Star;
import frc.robot.subsystems.IntakeSubsystem.Sushi;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

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

        // private final IntakeCommandFactory m_IntakeCommandFactory = new
        // IntakeCommandFactory(m_IntakeSubsystem);
        // TODO: Gavan or Alec; Add Shooter Subystem
        // TODO: Gavn or c; Add Climber Subsystem

        // private final Shooter m_Shooter = new Shooter();
        // private final Kicker m_Kicker = new Kicker();

        private final CommandJoystick m_DriverJoystick = new CommandJoystick(
                        Constants.OperatorConstants.DriverJoystick);
        private final CommandJoystick m_AssistantJoystick = new CommandJoystick(
                        Constants.OperatorConstants.AssistJoystick);

        private final DoubleSupplier throttleSupplier = () -> {
                double raw = m_DriverJoystick.getRawAxis(3);
                double scaled = (raw + 1) / 2.0;
                return MathUtil.clamp(scaled, 0.1, 1.0);
        };
        
        private final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and
         * commands.
         */
        public RobotContainer() {
                configureBindings();

                // Have the autoChooser pull in all PathPlanner autos as options
                autoChooser = AutoBuilder.buildAutoChooser();

                // Set the default auto (do nothing)
                autoChooser.setDefaultOption("Do Nothing", Commands.none());

                SmartDashboard.putData("Auto Chooser", autoChooser);
                SmartDashboard.putData(CommandScheduler.getInstance());
                // SmartDashboard.putBoolean("limitSwitch state", m_ClimberSubsystem.getlimitSwitchState());
                // SmartDashboard.putData("LimSwitch", m_ClimberSubsystem.lim);
                // TODO: Gavan or Alec; You need to set default commands
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
                        () -> -m_DriverJoystick.getY() * throttleSupplier.getAsDouble(),
                        () -> -m_DriverJoystick.getX() * throttleSupplier.getAsDouble())
                        .withControllerRotationAxis(() -> m_DriverJoystick.getRawAxis(
                                        2) * throttleSupplier.getAsDouble())
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        m_DriverJoystick.getRawAxis(
                                                        2)
                                                        * Math.PI)
                                        * (Math.PI
                                                        * 2),
                                        () -> Math.cos(
                                                        m_DriverJoystick.getRawAxis(
                                                                        2)
                                                                        * Math.PI)
                                                        * (Math.PI
                                                                        * 2))
                        .headingWhile(true)
                        .translationHeadingOffset(true)
                        .translationHeadingOffset(Rotation2d.fromDegrees(
                                        0));

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
         * with an arbitrary predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {
                Command driveFieldOrientedDirectAngle = m_SwerveSubsystem.driveFieldOriented(driveDirectAngle);
                Command driveFieldOrientedAnglularVelocity = m_SwerveSubsystem.driveFieldOriented(driveAngularVelocity);
                Command driveRobotOrientedAngularVelocity = m_SwerveSubsystem.driveFieldOriented(driveRobotOriented);
                Command driveSetpointGen = m_SwerveSubsystem.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngle);
                Command driveFieldOrientedDirectAngleKeyboard = m_SwerveSubsystem
                                .driveFieldOriented(driveDirectAngleKeyboard);
                Command driveFieldOrientedAnglularVelocityKeyboard = m_SwerveSubsystem
                                .driveFieldOriented(driveAngularVelocityKeyboard);
                Command driveSetpointGenKeyboard = m_SwerveSubsystem.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngleKeyboard);

                // =========== Set Default Command for swerve ============
                if (RobotBase.isSimulation()) {
                        m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);

                        m_IntakeStar.setDefaultCommand(m_IntakeStar.setStarDutyCylce(0));
                        m_IntakeSushi.setDefaultCommand(m_IntakeSushi.setSushiDutyCycle(0));

                        m_Shooter.setDefaultCommand(m_Shooter.setShooterDutyCycle(0));
                        m_Kicker.setDefaultCommand(m_Kicker.setKickerDutyCylce(0));

                } else if (RobotBase.isReal()) {
                        m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                        m_IntakeStar.setDefaultCommand(m_IntakeStar.setStarDutyCylce(0));
                        m_IntakeSushi.setDefaultCommand(m_IntakeSushi.setSushiDutyCycle(0));

                        m_Shooter.setDefaultCommand(m_Shooter.setShooterDutyCycle(0));
                        m_Kicker.setDefaultCommand(m_Kicker.setKickerDutyCylce(0));

                        m_ClimberSubsystem.setDefaultCommand(m_ClimberSubsystem.set(0));

                } else if (DriverStation.isTest()) {



                }

                m_DriverJoystick.button(8).onTrue(m_SwerveSubsystem.sysIdDriveMotorCommand());
                m_DriverJoystick.button(9).onTrue(m_SwerveSubsystem.sysIdAngleMotorCommand());

                m_AssistantJoystick.button(2).whileTrue(Commands.parallel(
                                m_IntakeStar.setStarVelocity(RPM.of(2000)),
                                m_IntakeSushi.setSushiVelocity(RPM.of(1500)).withName("IntakeFuel")));
                
                m_AssistantJoystick.trigger().whileTrue(Commands.parallel((m_Shooter.setShooterDutyCycle(0.7)),m_Kicker.setKickerDutyCylce(0.5)));

                // m_AssistantJoystick.button(2).whileTrue(Commands.parallel(m_IntakeStar.setStarDutyCylce(0.9), m_IntakeSushi.setSushiDutyCycle(0.5)));
              

                // m_AssistantJoystick.button(3).whileTrue(m_IntakeStar.setStarVelocity());
                // m_AssistantJoystick.button(4).whileTrue(m_IntakeSushi.setSushiVelocity());

                m_AssistantJoystick.button(5).onTrue(m_Shooter.setShooterVelocity());
                m_AssistantJoystick.button(6).onTrue(m_Kicker.setKickerVelocity());

                m_AssistantJoystick.button(8).onTrue(m_ClimberSubsystem.set(0.1));
                m_AssistantJoystick.button(9).onTrue(m_ClimberSubsystem.set(-0.1));

                // m_AssistantJoystick.button(10).onTrue(m_ClimberSubsystem.setHeightAndStop(Meters.of(0.25)));

                // m_AssistantJoystick.button(12).onTrue(m_ClimberSubsystem.sysId());

                m_AssistantJoystick.button(11).onTrue(m_IntakeStar.sysId());
                m_AssistantJoystick.button(14).onTrue(m_IntakeSushi.sysId());

                m_AssistantJoystick.button(15).onTrue(m_Shooter.sysId());
                m_AssistantJoystick.button(16).onTrue(m_Kicker.sysId());

               
                // ----------------------

                // Command AimAtObject = new AimAtObject(m_SwerveSubsystem,
                // m_SwerveSubsystem::getX, m_SwerveSubsystem::getY);

                if (RobotBase.isReal()) {
                        m_DriverJoystick.trigger().onTrue(Commands.runOnce(
                                        () -> m_SwerveSubsystem.zeroGyro()));

                        Pose2d target = new Pose2d(new Translation2d(1, 4),
                                        Rotation2d.fromDegrees(90));
                        m_SwerveSubsystem.getSwerveDrive().field.getObject("targetPose").setPose(target);

                        driveDirectAngle.driveToPose(() -> target,
                                        new ProfiledPIDController(5,
                                                        0,
                                                        0,
                                                        new Constraints(5, 2)),
                                        new ProfiledPIDController(5,
                                                        0,
                                                        0,
                                                        new Constraints(Units.degreesToRadians(360),
                                                                        Units.degreesToRadians(180))));

                        m_DriverJoystick.trigger().onTrue(Commands.runOnce(
                                        () -> m_SwerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));

                        m_DriverJoystick.button(11).onTrue(Commands.runOnce(
                                        () -> m_SwerveSubsystem.zeroGyro()));

                        // m_DriverJoystick.button(1).whileTrue(m_SwerveSubsystem.sysIdDriveMotorCommand());

                        m_DriverJoystick.button(2)
                                        .whileTrue(Commands.runEnd(
                                                        () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                        () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

                }

                if (Robot.isSimulation()) {
                        Pose2d target = new Pose2d(new Translation2d(1, 4),
                                        Rotation2d.fromDegrees(90));
                        m_SwerveSubsystem.getSwerveDrive().field.getObject("targetPose").setPose(target);

                        driveDirectAngleKeyboard.driveToPose(() -> target,
                                        new ProfiledPIDController(5,
                                                        0,
                                                        0,
                                                        new Constraints(5, 2)),
                                        new ProfiledPIDController(5,
                                                        0,
                                                        0,
                                                        new Constraints(Units.degreesToRadians(360),
                                                                        Units.degreesToRadians(180))));

                        m_DriverJoystick.trigger().onTrue(Commands.runOnce(
                                        () -> m_SwerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));

                        m_DriverJoystick.button(11).onTrue(Commands.runOnce(
                                        () -> m_SwerveSubsystem.zeroGyro()));

                        m_DriverJoystick.button(1).whileTrue(m_SwerveSubsystem.sysIdDriveMotorCommand());

                        m_DriverJoystick.button(2)
                                        .whileTrue(Commands.runEnd(
                                                        () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                        () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
                        // m_DriverJoystick.button(3).whileTrue(AimAtObject);

                        // TODO: Gavan or Alec; Bind buttons for Intake system
                        // TODO: Gavan or Alec; Bind buttons for Climber system
                        // TODO: Gavan or Alec; Bind buttons for Shooter system
                        // driverXbox.b().whileTrue(
                        // drivebase.driveToPose(
                        // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                        // );
                }

                if (DriverStation.isTest()) {
                        m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive
                                                                                                 // command above!

                        // m_DriverJoystick.x().whileTrue(Commands.runOnce(m_SwerveSubsystem::lock,
                        // m_SwerveSubsystem).repeatedly());
                        // m_DriverJoystick.start().onTrue((Commands.runOnce(m_SwerveSubsystem::zeroGyro)));
                        // m_DriverJoystick.back().whileTrue(m_SwerveSubsystem.centerModulesCommand());
                        // m_DriverJoystick.leftBumper().onTrue(Commands.none());
                        // m_DriverJoystick.rightBumper().onTrue(Commands.none());
                } else {
                        // m_DriverJoystick.a().onTrue((Commands.runOnce(m_SwerveSubsystem::zeroGyro)));
                        // m_DriverJoystick.start().whileTrue(Commands.none());
                        // m_DriverJoystick.back().whileTrue(Commands.none());
                        // m_DriverJoystick.leftBumper().whileTrue(Commands.runOnce(m_SwerveSubsystem::lock,
                        // m_SwerveSubsystem).repeatedly());
                        // m_DriverJoystick.rightBumper().onTrue(Commands.none());
                }
        }

        public double driverGetThrottle() {
                return MathUtil.clamp((m_DriverJoystick.getThrottle() +1)/2, 0.1, 1);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return autoChooser.getSelected();
        }
}
