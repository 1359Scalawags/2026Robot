// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Inches;



import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig;

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(Constants.Hopper.jigglerMotorID, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController;

 private final FlyWheelConfig hopperConfig;

  /** Creates a new ExampleSubsystem. */
  public HopperSubsystem() {
    smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  .withClosedLoopController(Constants.Hopper.kP, Constants.Hopper.kI, Constants.Hopper.kD)
  .withSimClosedLoopController(Constants.Hopper.kP, Constants.Hopper.kI, Constants.Hopper.kD)
  .withSimFeedforward(new SimpleMotorFeedforward(Constants.Hopper.kS, Constants.Hopper.kV, Constants.Hopper.kA))
  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  .withGearing(new MechanismGearing(GearBox.fromStages("20:1")))
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(20))
  .withTrapezoidalProfile(Constants.Hopper.hopperMaxVelocity, Constants.Hopper.hopperMaxAcceleration );

  sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  hopperConfig = new FlyWheelConfig(sparkSmartMotorController)
  .withDiameter(Inches.of(4))
  .withMass(Pounds.of(1))
  .withSoftLimit(RPM.of(-2500), RPM.of(2500))
  .withTelemetry("HopperMech", TelemetryVerbosity.HIGH);

  hopper = new FlyWheel(hopperConfig);
  }


    // Shooter Mechanism
  private FlyWheel hopper;
  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() 
  {return hopper.getSpeed();
  }

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return hopper.setSpeed(speed);
  }


  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return hopper.set(dutyCycle);
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command runHopper(AngularVelocity speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {setVelocity(speed);});
  }

    public Command sysId() {
      return hopper.sysId(Volts.of(12), Volts.of(0.5).per(Second), Seconds.of(30));
    }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }



  @Override
  public void periodic() {
    hopper.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hopper.simIterate();
  }
}
