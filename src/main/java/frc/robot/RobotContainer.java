// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.Climber;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveCommands.AimAtObject;
import frc.robot.commands.SwerveCommands.AlignToTag;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;
import java.lang.annotation.Target;

import com.pathplanner.lib.auto.AutoBuilder;

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
        private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
        // private final IntakeCommandFactory m_IntakeCommandFactory = new IntakeCommandFactory(m_IntakeSubsystem);
        // TODO: Gavan or Alec; Add Shooter Subystem
        // TODO:  Gavn or c; Add Climber Subsystem      

        // private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
        
        private final CommandJoystick m_DriverJoystick = new CommandJoystick(
                        Constants.OperatorConstants.DriverJoystick);
        private final CommandJoystick m_AssistantJoystick = new CommandJoystick(
                        Constants.OperatorConstants.AssistJoystick);

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

                // TODO: Gavan or Alec; You need to set default commands
        }

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is
         * controlled by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_SwerveSubsystem.getSwerveDrive(),
                        () -> m_DriverJoystick.getY() * -1,
                        () -> m_DriverJoystick.getX() * -1)
                        .withControllerRotationAxis(() -> m_DriverJoystick.getZ() *-1)
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


                //TODO : uncomment when swerve work


                // // Schedule `setVelocity` when the Xbox controller's B button is pressed,
                // // cancelling on release.
                // // Schedule `set` when the Xbox controller's B button is pressed,
                // // cancelling on release.

                //  m_DriverJoystick.button(6).onTrue(m_IntakeSubsystem.setIntakeSpeed(RPM.of(Constants.Intake.sushiIntakeSpeed),RPM.of(Constants.Intake.starIntakeSpeed)));
                //  m_DriverJoystick.button(7).onTrue(m_IntakeSubsystem.setIntakeSpeed(RPM.of(0),RPM.of(0)));
                
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
                

                
                        //=========== Set Default Command  for swerve ============
                    if (RobotBase.isSimulation()) {
                        m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
                        // m_ShooterSubsystem.setDefaultCommand(m_ShooterSubsystem.set(0));

                } else {
                        m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }


                // m_AssistantJoystick.button(1).whileTrue(m_ShooterSubsystem.shootFuel(RPM.of(200),RPM.of(200)));

                // m_AssistantJoystick.button(2).whileTrue(m_ShooterSubsystem.setShooterVelocity(RPM.of(100)));
                // m_AssistantJoystick.button(3).whileTrue(m_ShooterSubsystem.setKickerVelocity(RPM.of(100)));
                m_AssistantJoystick.button(4).whileTrue(m_IntakeSubsystem.setStarVelocity(RPM.of(500)));
                m_AssistantJoystick.button(5).whileTrue(m_IntakeSubsystem.setSushiVelocity(RPM.of(500)));



                // m_AssistantJoystick.button(3).whileTrue(m_ShooterSubsystem.set(0.3));



                //  m_AssistantJoystick.button(6).onTrue(m_IntakeSubsystem.setIntakeSpeed(RPM.of(Constants.Intake.sushiIntakeSpeed),RPM.of(Constants.Intake.starIntakeSpeed)));

                //  m_AssistantJoystick.button(6).onTrue(m_IntakeSubsystem.setIntakeSpeed(RPM.of(Constants.Intake.sushiIntakeSpeed),RPM.of(Constants.Intake.starIntakeSpeed)));
  

//----------------------

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

                        m_DriverJoystick.button(1).whileTrue(m_SwerveSubsystem.sysIdDriveMotorCommand());


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
