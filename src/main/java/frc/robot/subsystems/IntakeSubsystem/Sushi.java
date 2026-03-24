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
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class Sushi extends SubsystemBase {

  private final SparkMax sushiMotor;


  private SmartMotorControllerConfig sushiSmcConfig;
  private DigitalInput limitSwitch = new DigitalInput(0);
  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sushiSmartMotorController;


  private final FlyWheelConfig sushiConfig;

  private FlyWheel sushiWheel;

  //TODO: create constants whereveer needed.
  public Sushi() {

    //Creates the motor objects that control the motors on the real robot
    sushiMotor = new SparkMax(Constants.Intake.sushiMotorID, MotorType.kBrushless);

    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    //TODO: need to confiure the SMC correctly for the values and test values we want to use on the real robot
    sushiSmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.Intake.sushiP, Constants.Intake.sushiI, Constants.Intake.sushiD,
           RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
        .withSimClosedLoopController(Constants.Intake.sushiP, Constants.Intake.sushiI, Constants.Intake.sushiD,
            DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withFeedforward(new SimpleMotorFeedforward(Constants.Intake.sushiS,Constants.Intake.sushiV,Constants.Intake.sushiA))
        .withSimFeedforward(new SimpleMotorFeedforward(Constants.Intake.sushiS,Constants.Intake.sushiV,Constants.Intake.sushiA))
        .withTelemetry("sushiMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(35))
        .withTrapezoidalProfile(Constants.Intake.sushiMaxVelocity, Constants.Intake.sushiMaxAcceleration);

    sushiSmartMotorController = new SparkWrapper(sushiMotor, DCMotor.getNEO(1), sushiSmcConfig);
    sushiConfig = new FlyWheelConfig(sushiSmartMotorController)
        .withDiameter(Inches.of(2))
        .withMass(Pounds.of(1.07))
        .withSoftLimit(RPM.of(-2000), RPM.of(2000))
        .withTelemetry("sushiMech", TelemetryVerbosity.HIGH);

    sushiWheel = new FlyWheel(sushiConfig);   
  }
  /**
   * @return Shooter velocity.
   */
  public AngularVelocity getSushiVelocity() {
    return sushiWheel.getSpeed();
  }

    public Command setSushiVelocity(AngularVelocity speed) {
    return sushiWheel.setSpeed(speed);
  }

  // Set the dutycycle of the shooter.
  public Command setSushiDutyCycle(double dutyCycle) {
    return sushiWheel.set(dutyCycle);
  }

  public Command setVolatage(double volts) {
    return sushiWheel.setVoltage(Volts.of(volts));
  }

  public Command sysId() {
    return sushiWheel.sysId(Volts.of(12), Volts.of(0.5).per(Second), Seconds.of(30));
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Sushi/VelocityRPM", sushiMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Sushi/Applied", sushiMotor.getAppliedOutput());
    SmartDashboard.putNumber("Sushi/SetpointRPS",
    sushiWheel.getMechanismSetpointVelocity()
        .map(v -> v.in(edu.wpi.first.units.Units.RotationsPerSecond))
        .orElse(0.0));
    sushiWheel.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    sushiWheel.simIterate();
  }
}

