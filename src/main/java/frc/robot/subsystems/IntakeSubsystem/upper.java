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
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
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

public class Upper extends SubsystemBase {

  private final SparkMax upperMotor;


  private SmartMotorControllerConfig upperSmcConfig;
  private SmartMotorController upperSmartMotorController;


  private final FlyWheelConfig upperConfig;

  private FlyWheel upperWheel;

  public Upper() {

    //Creates the motor objects that control the motors on the real robot
    upperMotor = new SparkMax(Constants.Intake.upperMotorID, MotorType.kBrushless);

    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    upperSmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.Intake.upperP, Constants.Intake.upperI, Constants.Intake.upperD,
           RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
        .withSimClosedLoopController(Constants.Intake.upperP, Constants.Intake.upperI, Constants.Intake.upperD,
            DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withFeedforward(new SimpleMotorFeedforward(Constants.Intake.upperS,Constants.Intake.upperV,Constants.Intake.upperA))
        .withSimFeedforward(new SimpleMotorFeedforward(Constants.Intake.upperS,Constants.Intake.upperV,Constants.Intake.upperA))
        .withTelemetry("upperMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromStages("1.36:1")))
        .withMechanismCircumference(Inches.of(3))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(35))
        .withTrapezoidalProfile(Constants.Intake.upperMaxVelocity, Constants.Intake.upperMaxAcceleration);

    upperSmartMotorController = new SparkWrapper(upperMotor, DCMotor.getNEO(1), upperSmcConfig);
    upperConfig = new FlyWheelConfig(upperSmartMotorController)
        .withDiameter(Inches.of(2))
        .withMass(Pounds.of(1.07))
        .withSoftLimit(RPM.of(-3500), RPM.of(3500))
        .withTelemetry("upperMech", TelemetryVerbosity.HIGH);

    upperWheel = new FlyWheel(upperConfig);   
  }
  /**
   * @return Shooter velocity.
   */
  public AngularVelocity getupperVelocity() {
    return upperWheel.getSpeed();
  }

    public Command setupperVelocity(AngularVelocity speed) {
    return upperWheel.setSpeed(speed);
  }

  // Set the dutycycle of the shooter.
  public Command setupperDutyCycle(double dutyCycle) {
    return upperWheel.set(dutyCycle);
  }

  public Command setVolatage(double volts) {
    return upperWheel.setVoltage(Volts.of(volts));
  }

  public Command sysId() {
    return upperWheel.sysId(Volts.of(12), Volts.of(0.5).per(Second), Seconds.of(30));
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("upper/VelocityRPM", upperMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("upper/Applied", upperMotor.getAppliedOutput());
    SmartDashboard.putNumber("upper/SetpointRPS",
    upperWheel.getMechanismSetpointVelocity()
        .map(v -> v.in(edu.wpi.first.units.Units.RotationsPerSecond))
        .orElse(0.0));
    upperWheel.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    upperWheel.simIterate();
  }
}



