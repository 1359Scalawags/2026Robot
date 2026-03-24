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
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
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

public class Flippy extends SubsystemBase {

  private final SparkMax flippyMotor;

  private SmartMotorControllerConfig starSmcConfig;
  private DigitalInput limitSwitch = new DigitalInput(1);
  private boolean lastLimitPressed = false;

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController flippySmartMotorController;

  //TODO: make these not fly wheels (maybe i dont actualy know)
  private final FlyWheelConfig flippyConfig;

  private FlyWheel flippyWheel;

  private DigitalInput limitSwitch = new DigitalInput(1);
  private boolean lastLimitPressed = false;
  private final DIOSim limitSwitchSim = new DIOSim(limitSwitch);
  private boolean simLimitLatched = false;

  public Flippy() {

    //Creates the motor objects that control the motors on the real robot
    flippyMotor = new SparkMax(Constants.Intake.flippyMotorID, MotorType.kBrushless);

    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    //TODO: need to confiure the SMC correctly for the values and test values we want to use on the real robot
    flippySmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.Intake.flippyP, Constants.Intake.flippyI, Constants.Intake.flippyD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(Constants.Intake.flippyP, Constants.Intake.flippyI, Constants.Intake.flippyD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withFeedforward(
            new SimpleMotorFeedforward(Constants.Intake.flippyS,Constants.Intake.flippyV,Constants.Intake.flippyA))
        .withSimFeedforward(
            new SimpleMotorFeedforward(Constants.Intake.flippyS, Constants.Intake.flippyV, Constants.Intake.flippyA))
        .withTelemetry("FlipperMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1,1)))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withTrapezoidalProfile(Constants.Intake.flippyMaxVelocity, Constants.Intake.flippyMaxAcceleration);

    flippySmartMotorController = new SparkWrapper(flippyMotor, DCMotor.getNEO(1), flippySmcConfig);
    // starSmartMotorController.setEncoderInverted(true);

    //TODO: make sure these are correct too
    flippyConfig = new FlyWheelConfig(flippySmartMotorController)
        .withDiameter(Inches.of(2))
        .withMass(Pounds.of(0.375))
        .withSoftLimit(RPM.of(-2500), RPM.of(2500))
        .withTelemetry("starMech", TelemetryVerbosity.HIGH);

    flippyWheel = new FlyWheel(flippyConfig);
  }

  public BooleanSupplier limitSwitchSupplier = () -> {
    return !limitSwitch.get();
  };
  public AngularVelocity getflippyVelocity() {
    return flippyWheel.getSpeed();
  }

  public Command setflippyVelocity(AngularVelocity speed) {
    return flippyWheel.setSpeed(speed);
  }


  public Command setflippyDutyCylce(double dutyCycle) {
    return flippyWheel.set(dutyCycle);
  }

  public Command setVolatage(double volts) {
    return flippyWheel.setVoltage(Volts.of(volts));
  }

  public Command sysId() {
    return flippyWheel.sysId(Volts.of(12), Volts.of(0.5).per(Second), Seconds.of(30));
  }

  public BooleanSupplier limitSwitchSupplier = () -> {
    return !limitSwitch.get();
  };

  @Override
  public void periodic() {
    final boolean limitPressed = limitSwitchSupplier.getAsBoolean();

    // if (limitPressed) {
    //   starMotor.getEncoder().setPosition(0);
    // }
    lastLimitPressed = limitPressed;

    SmartDashboard.putBoolean("FlipMotor/LimitSwitch", limitPressed);
    SmartDashboard.putNumber("Star/Applied", flippyMotor.getAppliedOutput());
    flippyWheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    flippyWheel.simIterate();
  }
}