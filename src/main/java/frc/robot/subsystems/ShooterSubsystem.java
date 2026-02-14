// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax shooterMotor;
  private final SparkMax kickerMotor;


  private SmartMotorControllerConfig shooterSmcConfig;
  private SmartMotorControllerConfig kickerSmcConfig;


  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController shooterSmartMotorController;
  private SmartMotorController kickerSmartMotorController;


  private final FlyWheelConfig shooterConfig;
  private final FlyWheelConfig kickerConfig;

  private FlyWheel shooterWheel;
  private FlyWheel kickerWheel;

  enum ShooterSpeed {
    off,
    low,
    full
  }

  ShooterSpeed currentSpeed;

  //TODO: create constants whereveer needed.
  public ShooterSubsystem() {

    //Creates the motor objects that control the motors on the real robot
    shooterMotor = new SparkMax(Shooter.flyWheelID, MotorType.kBrushless);
    kickerMotor = new SparkMax(Shooter.fingerWheelID, MotorType.kBrushless);

    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    //TODO: need to confiure the SMC correctly for the values and test values we want to use on the real robot
    shooterSmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // Telemetry name and verbosity level
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        // In this example GearBox.fromReductionStages(3,4) is the same as
        // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
        // your motor.
        // You could also use .withGearing(12) which does the same thing.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 1)))
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    kickerSmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    shooterSmartMotorController = new SparkWrapper(shooterMotor, DCMotor.getNEO(1), shooterSmcConfig);
    kickerSmartMotorController = new SparkWrapper(kickerMotor, DCMotor.getNEO(1), kickerSmcConfig);

    //TODO check these values to see if they are accurate
    shooterConfig = new FlyWheelConfig(shooterSmartMotorController)
        // Diameter of the flywheel.
        .withDiameter(Inches.of(4))
        // Mass of the flywheel.
        .withMass(Pounds.of(1))
        // Maximum speed of the shooter.
        .withUpperSoftLimit(RPM.of(1000))
        // Telemetry name and verbosity for the arm.
        .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);
    kickerConfig = new FlyWheelConfig(kickerSmartMotorController)
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(1))
    .withUpperSoftLimit(RPM.of(1000))
    .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

    shooterWheel = new FlyWheel(shooterConfig);
    kickerWheel = new FlyWheel(kickerConfig);
  }

    /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getShooterVelocity() {
    return shooterWheel.getSpeed();
  }
  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setShooterVelocity(AngularVelocity speed) {
    return shooterWheel.setSpeed(speed);
  }
 /**
   * Set the kicker velocity to feed fuel into the shooter.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setKickerVelocity(AngularVelocity speed) {
    return kickerWheel.setSpeed(speed);
  }
//TODO: needs a wait Commmand
  public Command shootFuel(AngularVelocity shootersSpeed, AngularVelocity kickerSpeed) {
    // return Commands.parallel(setShooterVelocity(shootersSpeed), setKickerVelocity(kickerSpeed));

      //Alternative way to create this command
    return run(() -> {kickerWheel.setSpeed(kickerSpeed);shooterWheel.setSpeed(shootersSpeed);})
            .withName("ShootFuelCommand");
  }

  @Override
  public void periodic() {
    shooterWheel.updateTelemetry();
    // kickerWheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooterWheel.simIterate();
    // kickerWheel.simIterate();
    // This method will be called once per scheduler run during simulation
  }
}
