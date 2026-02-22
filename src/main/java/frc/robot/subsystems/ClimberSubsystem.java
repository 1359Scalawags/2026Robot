// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.SmartMotorController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;



public class ClimberSubsystem extends SubsystemBase {

  private SparkMax spark = new SparkMax(Constants.Climber.climberID, MotorType.kBrushless);
  
  private SmartMotorControllerConfig smcConfig;
  
   // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController;

  private ElevatorConfig elevconfig;

  // Elevator Mechanism
  private Elevator climber;

  private DigitalInput limitSwitch = new DigitalInput(0);;


public ClimberSubsystem() {

  smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
    .withClosedLoopController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
    .withSimClosedLoopController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
    .withFeedforward(new ElevatorFeedforward(Constants.Climber.ks, Constants.Climber.kg, Constants.Climber.kv))
    .withSimFeedforward(new ElevatorFeedforward( Constants.Climber.ks, Constants.Climber.kg, Constants.Climber.kv))
    .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromStages("125:1")))
    .withMotorInverted(false)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(35))
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25));

  sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  elevconfig = new ElevatorConfig(sparkSmartMotorController)
    .withStartingHeight(Meters.of(Constants.Climber.START_HEIGHT))
    .withHardLimits(Meters.of(Inches.of(Constants.Climber.MIN_HEIGHT).in(Meters)), Meters.of(Inches.of(Constants.Climber.MAX_HEIGHT).in(Meters)))
    .withTelemetry("Climber", TelemetryVerbosity.HIGH)
    .withMass(Pounds.of(16));

  climber = new Elevator(elevconfig);

  Shuffleboard.getTab("Arm").add("LimitSwitch", limitSwitch);

  SmartDashboard.putBoolean("limSwitch2", getlimitSwitchState());
}
/**
   * Set the height of the elevator and does not end the command when reached.
   * @param angle Distance to go to.
   * @return a Command
   */
  public Command setHeight(Distance height) {
     return climber.setHeight(height);
    }
  
  /**
   * Set the height of the elevator and ends the command when reached, but not the closed loop controller.
   * @param angle Distance to go to.
   * @return A Command
   */
  public Command setHeightAndStop(Distance height) { 
    return climber.setHeight(height);
  }
  
  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) {
     return climber.set(dutycycle).withName("MoveClimber");
    }

  public boolean getlimitSwitchState() {
    return limitSwitch.get();
  }

  /**
   * Run sysId on the {@link Elevator}
   */
  public Command sysId() { 
    return climber.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

    @Override
  public void periodic() {
      final boolean lstate = getlimitSwitchState();

      // Publish limit switch state to SmartDashboard for debugging
      SmartDashboard.putBoolean("Climber/LimitSwitch", lstate);

        // Publish current climber height (meters and inches)
        try {
          SmartDashboard.putNumber("Climber/HeightMeters", climber.getHeight().in(Meters));
          SmartDashboard.putNumber("Climber/HeightInches", climber.getHeight().in(Inches));
        } catch (Exception e) {
          // If the mechanism isn't initialized yet or getHeight() isn't available,
          // don't crash — just skip publishing.
        }

      if (lstate == true) {
        spark.getEncoder().setPosition(0);
      }

      if (spark.get() < 0) {
        if (lstate == true) {
          climber.set(0);
          spark.getEncoder().setPosition(0);
        }
      }

      climber.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    climber.simIterate();
    }
}
