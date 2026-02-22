// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Kicker;
import frc.robot.subsystems.ShooterSubsystem.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Star;
import frc.robot.subsystems.IntakeSubsystem.Sushi;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.CancelCommandEvent;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Inches;
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


        private final CommandJoystick m_DriverJoystick = new CommandJoystick(
                        Constants.OperatorConstants.DriverJoystick);
        private final CommandJoystick m_AssistantJoystick = new CommandJoystick(
                        Constants.OperatorConstants.AssistJoystick);

        private final DoubleSupplier throttleSupplier = () -> {
                double raw = m_DriverJoystick.getRawAxis(3) * -1;
                double scaled = (raw + 1) / 2.0;
                return MathUtil.clamp(scaled, 0.25, 1.0);
        };
        
        private final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and
         * commands.
         */
        public RobotContainer() {
                configureBindings();

                // NamedCommands.registerCommand("ShootFuel", Commands.race(
                //         new WaitCommand(Seconds.of(10)).andThen(Commands.sequence(m_Shooter.setShooterVelocity, m_Shooter.setShooterVelocity)),
                //         m_Shooter.setShooterVelocity(Constants.Shooter.shooterVelocity),
                //         Commands.sequence(
                //                 new WaitCommand(Seconds.of(1.5)),
                //                 m_Kicker.setKickerVelocity(Constants.Shooter.kickerVelocity)
                //         )
                // ));
                
                // Commands.ParallelRaceGroup.addCommands(new WaitCommand(Seconds.of(7)));
                // Commands.ParallelRaceGroup (
                //         (m_Shooter.setShooterVelocity(Constants.Shooter.shooterVelocity)),
                //                 Commands.sequence(
                //                         new WaitCommand(Seconds.of(1.5)),
                //                         m_Kicker.setKickerVelocity(Constants.Shooter.kickerVelocity))));

                // Have the autoChooser pull in all PathPlanner autos as options
                autoChooser = AutoBuilder.buildAutoChooser();

                // Set the default auto (do nothing)
                autoChooser.setDefaultOption("Do Nothing", Commands.none());

                SmartDashboard.putData("Auto Chooser", autoChooser);
                SmartDashboard.putData(CommandScheduler.getInstance());

                

                
                // SmartDashboard.putBoolean("limitSwitch state", m_ClimberSubsystem.getlimitSwitchState());
                // SmartDashboard.putData("LimSwitch", m_ClimberSubsystem.lim);
        }
        /**
         * Converts driver input into a field-relative ChassisSpeeds that is
         * controlled by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_SwerveSubsystem.getSwerveDrive(),
                        () -> m_DriverJoystick.getY() * -1 * throttleSupplier.getAsDouble(),
                        () -> m_DriverJoystick.getX() * -1 *  throttleSupplier.getAsDouble())
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
                        () -> -m_DriverJoystick.getY(),
                        () -> -m_DriverJoystick.getX())
                        .withControllerRotationAxis(() -> m_DriverJoystick.getRawAxis(
                                        2))
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
                // Command driveFieldOrientedDirectAngle = m_SwerveSubsystem.driveFieldOriented(driveDirectAngle);
                // Command driveRobotOrientedAngularVelocity = m_SwerveSubsystem.driveFieldOriented(driveRobotOriented);
                // Command driveSetpointGen = m_SwerveSubsystem.driveWithSetpointGeneratorFieldRelative(
                //                 driveDirectAngle);
                // Command driveFieldOrientedDirectAngleKeyboard = m_SwerveSubsystem
                //                 .driveFieldOriented(driveDirectAngleKeyboard);
                // Command driveFieldOrientedAnglularVelocityKeyboard = m_SwerveSubsystem
                //                 .driveFieldOriented(driveAngularVelocityKeyboard);
                // Command driveSetpointGenKeyboard = m_SwerveSubsystem.driveWithSetpointGeneratorFieldRelative(
                //                 driveDirectAngleKeyboard);
                Command shootFuel = Commands.parallel(
                        (m_Shooter.setShooterVelocity(Constants.Shooter.shooterVelocity)),
                                Commands.sequence(
                                        new WaitCommand(Seconds.of(1.5)),
                                        m_Kicker.setKickerVelocity(Constants.Shooter.kickerVelocity)));

                Command intakeFuel = Commands.parallel(
                                m_IntakeStar.setStarVelocity(Constants.Intake.starVelocity),
                                m_IntakeSushi.setSushiVelocity(Constants.Intake.sushiVelocity))
                                .withName("IntakeFuel");

                Command driveFieldOrientedAngularVelocity = m_SwerveSubsystem.driveFieldOriented(driveAngularVelocity);

                // =========== Set Default Command for swerve ============
                if (RobotBase.isSimulation()) {
                        m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

                        m_IntakeStar.setDefaultCommand(m_IntakeStar.setStarDutyCylce(0));
                        m_IntakeSushi.setDefaultCommand(m_IntakeSushi.setSushiDutyCycle(0));

                        m_Shooter.setDefaultCommand(m_Shooter.setShooterDutyCycle(0));
                        m_Kicker.setDefaultCommand(m_Kicker.setKickerDutyCylce(0));

                } else if (RobotBase.isReal()) {
                        m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

                        m_IntakeStar.setDefaultCommand(m_IntakeStar.setStarDutyCylce(0));
                        m_IntakeSushi.setDefaultCommand(m_IntakeSushi.setSushiDutyCycle(0));

                        m_Shooter.setDefaultCommand(m_Shooter.setShooterDutyCycle(0));
                        m_Kicker.setDefaultCommand(m_Kicker.setKickerDutyCylce(0));

                        m_ClimberSubsystem.setDefaultCommand(m_ClimberSubsystem.set(0));

                } else if (DriverStation.isTest()) {
        }


                m_AssistantJoystick.button(2).whileTrue(intakeFuel);
                
                m_AssistantJoystick.trigger().whileTrue(shootFuel);

                // m_AssistantJoystick.button(3). whileTrue();
                        
                // Commands.sequence(new WaitCommand(Seconds.of(10.0))), m_Kicker.setKickerVelocity(Constants.Shooter.kickerVelocity));


                // m_AssistantJoystick.button(2).whileTrue(Commands.parallel(m_IntakeStar.setStarDutyCylce(0.9), m_IntakeSushi.setSushiDutyCycle(0.5)));


                m_AssistantJoystick.button(8).whileTrue(m_ClimberSubsystem.set(0.5));
                m_AssistantJoystick.button(9).whileTrue(m_ClimberSubsystem.set(-0.5));
                m_AssistantJoystick.button(14).whileTrue(m_ClimberSubsystem.setHeight(Meters.of(Inches.of(5).in(Meters))));

                // m_AssistantJoystick.button(10).onTrue(m_ClimberSubsystem.setHeightAndStop(Meters.of(0.25)));


                // m_AssistantJoystick.button(12).onTrue(m_ClimberSubsystem.sysId());

                // m_AssistantJoystick.button(11).onTrue(m_IntakeStar.sysId());
                // m_AssistantJoystick.button(14).onTrue(m_IntakeSushi.sysId());

                // m_AssistantJoystick.button(15).onTrue(m_Shooter.sysId());
                // m_AssistantJoystick.button(16).onTrue(m_Kicker.sysId());
                
                // m_DriverJoystick.button(8).onTrue(m_SwerveSubsystem.sysIdDriveMotorCommand());
                // m_DriverJoystick.button(9).onTrue(m_SwerveSubsystem.sysIdAngleMotorCommand());

               
                // ----------------------

                // Command AimAtObject = new AimAtObject(m_SwerveSubsystem,
                // m_SwerveSubsystem::getX, m_SwerveSubsystem::getY);

                if (RobotBase.isReal()) {
                        m_DriverJoystick.button(1).onTrue(Commands.runOnce(
                                        () -> m_SwerveSubsystem.zeroGyro()));
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
                }

                if (DriverStation.isTest()) {
                        m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive
                                                                                                 // command above!

                } else {

                }
        }

        public double driverGetThrottle() {
                return MathUtil.clamp((m_DriverJoystick.getThrottle() +1)/2, 0.1, 1);
        }

        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return autoChooser.getSelected();
        }
}
