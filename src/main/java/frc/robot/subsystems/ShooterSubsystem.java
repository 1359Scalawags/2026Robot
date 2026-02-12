// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

import frc.robot.Constants.*;

import edu.wpi.first.math.Pair;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax flyWheelMotor;
  private final SparkMax fingerWheelMotor;


  private SmartMotorControllerConfig smcConfig;
  // private SmartMotorControllerConfig smcConfig2;


  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController flyWheelSmartMotorController;
  // private SmartMotorController fingerSmartMotorController;


  private final FlyWheelConfig flyConfig;
  // private final FlyWheelConfig fingerrConfig;

  private FlyWheel shooter;
  // private FlyWheel shooter2;

  enum ShooterSpeed {
    off,
    low,
    full
  }

  ShooterSpeed currentSpeed;

  public ShooterSubsystem() {

    flyWheelMotor = new SparkMax(Shooter.flyWheelID, MotorType.kBrushless);
    fingerWheelMotor = new SparkMax(Shooter.fingerWheelID, MotorType.kBrushless);

    smcConfig = new SmartMotorControllerConfig(this)
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
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withFollowers(Pair.of(fingerWheelMotor, false));

    // smcConfig2 = new SmartMotorControllerConfig(this)
    // .withControlMode(ControlMode.CLOSED_LOOP)
    // .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90),
    // DegreesPerSecondPerSecond.of(45))
    // .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90),
    // DegreesPerSecondPerSecond.of(45))
    // .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    // .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    // .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
    // .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    // .withMotorInverted(false)
    // .withIdleMode(MotorMode.COAST)
    // .withStatorCurrentLimit(Amps.of(40));

    flyWheelSmartMotorController = new SparkWrapper(flyWheelMotor, DCMotor.getNEO(1), smcConfig);
    // fingerSmartMotorController = new SparkWrapper(fingerWheelMotor,
    // DCMotor.getNEO(2), smcConfig2);

    flyConfig = new FlyWheelConfig(flyWheelSmartMotorController)
        // Diameter of the flywheel.
        .withDiameter(Inches.of(4))
        // Mass of the flywheel.
        .withMass(Pounds.of(1))
        // Maximum speed of the shooter.
        .withUpperSoftLimit(RPM.of(1000))
        // Telemetry name and verbosity for the arm.
        .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);
    // fingerrConfig = new FlyWheelConfig(fingerSmartMotorController)
    // .withDiameter(Inches.of(4))
    // .withMass(Pounds.of(1))
    // .withUpperSoftLimit(RPM.of(1000))
    // .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

    shooter = new FlyWheel(flyConfig);
    // shooter2 = new FlyWheel(fingerrConfig);
  }

    /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {
    return shooter.getSpeed();
  }
  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }
  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return shooter.set(dutyCycle);
  }

  @Override
  public void periodic() {
    shooter.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
