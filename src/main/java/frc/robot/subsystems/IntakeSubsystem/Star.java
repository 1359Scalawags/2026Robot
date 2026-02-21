// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

public class Star extends SubsystemBase {

  private final SparkMax starMotor;

  private SmartMotorControllerConfig starSmcConfig;


  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController starSmartMotorController;


  private final FlyWheelConfig starConfig;

  private FlyWheel starWheel;

  //TODO: create constants whereveer needed.
  public Star() {

    //Creates the motor objects that control the motors on the real robot
    starMotor = new SparkMax(Constants.Intake.starMotorID, MotorType.kBrushless);

    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    //TODO: need to confiure the SMC correctly for the values and test values we want to use on the real robot
    starSmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.Intake.starP, Constants.Intake.starI, Constants.Intake.starD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(Constants.Intake.starP, Constants.Intake.starI, Constants.Intake.starD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withFeedforward(
            new SimpleMotorFeedforward(Constants.Intake.starS,Constants.Intake.starV,Constants.Intake.starA))
        .withSimFeedforward(
            new SimpleMotorFeedforward(Constants.Intake.starS, Constants.Intake.starV, Constants.Intake.starA))
        .withTelemetry("starMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1,1)))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));
        // .withTrapezoidalProfile(RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(1000));

    starSmartMotorController = new SparkWrapper(starMotor, DCMotor.getNEO(1), starSmcConfig);

    starConfig = new FlyWheelConfig(starSmartMotorController)
        .withDiameter(Inches.of(2))
        .withMass(Pounds.of(0.375))
        .withSoftLimit(RPM.of(-2500), RPM.of(2500))
        .withTelemetry("starMech", TelemetryVerbosity.HIGH);

    starWheel = new FlyWheel(starConfig);
  }

  public AngularVelocity getStarVelocity() {
    return starWheel.getSpeed();
  }


  public Command setStarVelocity(AngularVelocity speed) {
    return starWheel.setSpeed(speed);
  }


  public Command setStarDutyCylce(double dutyCycle) {
    return starWheel.set(dutyCycle);
  }

  public Command setVolatage(double volts) {
    return starWheel.setVoltage(Volts.of(volts));
  }

   public Command sysId() {
      return starWheel.sysId(Volts.of(12), Volts.of(0.5).per(Second), Seconds.of(30));
    }

  @Override
  public void periodic() {
    starWheel.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    starWheel.simIterate();
  }
}
