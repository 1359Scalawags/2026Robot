// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


public class ClimberSubsystem extends SubsystemBase {

  private SparkMax spark = new SparkMax(Constants.Climber.climberID, MotorType.kBrushless);
  
  private SmartMotorControllerConfig smcConfig;
  
   // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController;

  private ElevatorConfig elevconfig;

  // Elevator Mechanism
  private Elevator climber;

  private DigitalInput limitSwitch = new DigitalInput(0);
  private boolean lastLimitPressed = false;
  private final DIOSim limitSwitchSim = new DIOSim(limitSwitch);
  private boolean simLimitLatched = false;

public ClimberSubsystem(){

  smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
    .withClosedLoopController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD, MetersPerSecond.of(0.1), MetersPerSecondPerSecond.of(0.4))
    .withSimClosedLoopController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD, MetersPerSecond.of(0.1), MetersPerSecondPerSecond.of(0.4))
    .withFeedforward(new ElevatorFeedforward(Constants.Climber.ks, Constants.Climber.kg, Constants.Climber.kv))
    .withSimFeedforward(new ElevatorFeedforward( Constants.Climber.ks, Constants.Climber.kg, Constants.Climber.kv))
    .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromStages("125:1")))
    .withMotorInverted(true)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(Constants.Climber.CURRENT_LIMIT))
    .withClosedLoopRampRate(Seconds.of(0.15))
    .withOpenLoopRampRate(Seconds.of(0.15));

  sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  elevconfig = new ElevatorConfig(sparkSmartMotorController)
    .withStartingHeight(Meters.of(Inches.of(Constants.Climber.START_HEIGHT).in(Meters)))
    .withHardLimits(
        Meters.of(Inches.of(Constants.Climber.MIN_HEIGHT).in(Meters)),
        Meters.of(Inches.of(Constants.Climber.MAX_HEIGHT).in(Meters)))
    .withSoftLimits(
        Meters.of(Inches.of(Constants.Climber.SOFT_MIN).in(Meters)),
        Meters.of(Inches.of(Constants.Climber.SOFT_MAX).in(Meters)))
    .withTelemetry("Climber", TelemetryVerbosity.HIGH)
    .withMass(Pounds.of(16));
  spark.getEncoder().setPosition(5);
  climber = new Elevator(elevconfig);

  Shuffleboard.getTab("Arm").add("LimitSwitch", limitSwitch);

  SmartDashboard.putBoolean("limSwitch2", limitSwitchSupplier.getAsBoolean());
}
/**
   * Set the height of the elevator. Command runs continuously until cancelled.
   * @param height Distance to go to.
   * @return a Command
   */
  public Command setHeight(Distance height) {
     return climber.setHeight(height);
  }

  /* Sets the home height so that we can bring the robot down. */
  public Command home() {
    return Commands.run(() -> spark.getEncoder().setPosition(Inches.of(Constants.Climber.MAX_HEIGHT).in(Meters)));
  }
  
  /**
   * Go to the stowed (retracted) position.
   * @return a Command
   */
  public Command stow() {
    return setHeight(Meters.of(Inches.of(Constants.Climber.STOWED_HEIGHT).in(Meters)))
        .withName("ClimberStow");
  }

  /**
   * Go to the full climb (extended) position.
   * @return a Command
   */
  public Command extend() {
    return setHeight(Meters.of(Inches.of(Constants.Climber.CLIMB_HEIGHT).in(Meters)))
        .withName("ClimberExtend");
  }

  /**
   * Move the elevator up and down with open-loop duty cycle.
   * Bypasses closed-loop control.
   * @param dutycycle [-1, 1] speed to set the elevator to.
   */
  public Command set(double dutycycle) {
     return climber.set(dutycycle).withName("MoveClimber");
  }

  // Supplier for the limit switch state (renamed to avoid clash with the getter method)
  public BooleanSupplier limitSwitchSupplier = () -> {
    return !limitSwitch.get();
  };

  public BooleanSupplier getMaxHeightSupplier = () -> {
    if (climber.getHeight().in(Inches) >= 18) {
      return true;
    } else {
      return false;
    }
  };

  /**
   * Run sysId on the {@link Elevator}.
   * 
   * SAFETY NOTES for 125:1 geared mechanisms:
   * - maxVoltage: Keep LOW (2-3V). With 125:1, even 2V produces significant torque.
   * - step: Keep LOW (0.25-0.5 V/s). Slow ramp = time to react if something goes wrong.
   * - duration: Keep SHORT. The elevator only travels ~17 inches.
   * - ALWAYS have a finger on the disable button (Enter in sim, space on DS).
   * - Start the elevator near the MIDDLE of its travel, not at a hard stop.
   * - Watch the mechanism the entire time. Disable immediately if it hits a stop.
   */
  public Command sysId() {
    return climber.sysId(
        Volts.of(2.5),              // Max voltage — LOW for high-ratio gearbox
        Volts.of(0.25).per(Second), // Ramp rate — very slow ramp
        Seconds.of(10)              // Duration — long enough for slow ramp to collect data
    );
  }

  public Command homeCommand() {
    return Commands.run(() -> spark.getEncoder().setPosition(Inches.of(20).in(Meters)));
  }

  @Override
  public void periodic() {
    final boolean limitPressed = limitSwitchSupplier.getAsBoolean();

    SmartDashboard.putBoolean("Climber/LimitSwitch", limitPressed);
    try {
      SmartDashboard.putNumber("Climber/HeightMeters", climber.getHeight().in(Meters));
      SmartDashboard.putNumber("Climber/HeightInches", climber.getHeight().in(Inches));
      SmartDashboard.putNumber("Climber/DutyCycleCmd", spark.get()); // last commanded percent output (if used)
      SmartDashboard.putNumber("Climber/AppliedOutput", spark.getAppliedOutput()); // what Spark is applying
      SmartDashboard.putNumber("Climber/EncPos", spark.getEncoder().getPosition());
    } catch (Exception e) {
      SmartDashboard.putString("Climber/TelemetryError", e.toString());
    }

    // Zero ONCE when the switch is first pressed (rising edge)
    if (limitPressed) {
      spark.getEncoder().setPosition(0);
    }
    lastLimitPressed = limitPressed;

    climber.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    climber.simIterate();

    double heightM = climber.getHeight().in(Meters);
    double tripM   = Inches.of(Constants.Climber.MIN_HEIGHT + 2).in(Meters);

    double pressM   = tripM + Inches.of(0.10).in(Meters); // latch ON below this
    double releaseM = tripM + Inches.of(0.30).in(Meters); // latch OFF above this

    simLimitLatched = simLimitLatched
        ? (heightM < releaseM)   // stay latched until we rise above releaseM
        : (heightM <= pressM);   // latch when we drop below pressM

    SmartDashboard.putBoolean("Climber/Sim/AutoPressedLogic", simLimitLatched);
    SmartDashboard.putNumber("Climber/Sim/SimHeightIn", climber.getHeight().in(Inches));

    if (simLimitLatched && !lastLimitPressed) {
      spark.getEncoder().setPosition(0);
    }
    lastLimitPressed = simLimitLatched;
    // If your DIO reads false when pressed (common):
    limitSwitchSim.setValue(!simLimitLatched);
    // If your DIO reads true when pressed, use:
  }
  }
