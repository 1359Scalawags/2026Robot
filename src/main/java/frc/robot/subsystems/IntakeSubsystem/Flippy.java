// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Degrees;
import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.config.ArmConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Flippy extends SubsystemBase {

  private final SparkMax flippyMotor;
  private final SparkMax flipperMotor;

  private SmartMotorControllerConfig flippySmcConfig;
  private SmartMotorControllerConfig flipperSmConfig;

  private DigitalInput limitSwitch = new DigitalInput(1);
  private SmartMotorController flippySmartMotorController;
  private SmartMotorController flipperSmartMotorController;

  private final ArmConfig flippyConfig;
  private final ArmConfig flipperConfig;

  private Arm flippyArm;
  private Arm flipperArm;

  public Flippy() {

    flippyMotor = new SparkMax(Constants.Intake.flippyMotorID, MotorType.kBrushless);
    flipperMotor = new SparkMax(Constants.Intake.flippyMotorID, MotorType.kBrushless);
    
    flipperSmConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.Intake.flipperP, Constants.Intake.flipperI, Constants.Intake.flipperD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(Constants.Intake.flipperP, Constants.Intake.flipperI, Constants.Intake.flipperD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withTrapezoidalProfile(Intake.intakeMaxVelocity, Intake.intakeMaxAcceleration)
        .withExternalEncoder(flipperMotor.getAbsoluteEncoder())
        .withExternalEncoderGearing(1.0)
        .withExternalEncoderInverted(false)
        .withExternalEncoderZeroOffset(Inches.of(0))
        .withUseExternalFeedbackEncoder(true)
        .withTelemetry("FlipperMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromStages("64:1")))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40));
      
    flipperSmartMotorController = new SparkWrapper(flipperMotor, DCMotor.getNEO(1), flipperSmConfig);
    flipperConfig = new ArmConfig(flipperSmartMotorController)
            .withLength(Inches.of(10))
            .withMass(Pounds.of(15))
            .withStartingPosition(Degrees.of(90))
            .withHardLimit(Constants.Intake.flippyMinAngle, Constants.Intake.flippyMaxAngle)
            .withSoftLimits(Constants.Intake.flippyMinAngle, Constants.Intake.flippyMaxAngle)
            .withTelemetry("flippyMech", TelemetryVerbosity.HIGH);
    
    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    flippySmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.Intake.flippyP, Constants.Intake.flippyI, Constants.Intake.flippyD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(Constants.Intake.flippyP, Constants.Intake.flippyI, Constants.Intake.flippyD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withTrapezoidalProfile(Intake.intakeMaxVelocity, Intake.intakeMaxAcceleration)
        .withExternalEncoder(flipperMotor.getAbsoluteEncoder())
        .withExternalEncoderGearing(1.0)
        .withExternalEncoderInverted(false)
        .withExternalEncoderZeroOffset(Inches.of(0))
        .withUseExternalFeedbackEncoder(true)
        .withTelemetry("FlipperMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromStages("64:1")))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withLooselyCoupledFollowers(flipperSmartMotorController);
      
    
    flippySmartMotorController = new SparkWrapper(flippyMotor, DCMotor.getNEO(1), flippySmcConfig);
    // starSmartMotorController.setEncoderInverted(true);

    //TODO: make sure these are correct too
    flippyConfig = new ArmConfig(flippySmartMotorController)
            .withLength(Inches.of(10))
            .withMass(Pounds.of(15))
            .withStartingPosition(Degrees.of(90))
            .withHardLimit(Constants.Intake.flippyMinAngle, Constants.Intake.flippyMaxAngle)
            .withSoftLimits(Constants.Intake.flippyMinAngle, Constants.Intake.flippyMaxAngle)
            .withTelemetry("flippyMech", TelemetryVerbosity.HIGH);


    flippyArm = new Arm(flippyConfig);
    flipperArm = new Arm(flipperConfig);


    // Ensure the final flipperConfig is initialized to a sensible default to avoid compilation errors.
    // Reuse flippyConfig as the default; adjust if a separate configuration is required later.

  }

  public BooleanSupplier limitSwitchSupplier = () -> {
    return limitSwitch.get();
  };

  public Command setFlippyDutyCycle(double dutyCycle){
    return flippyArm.set(dutyCycle);
  };

  public Angle getFlippyAngle(){
    return flippyArm.getAngle();
  }

  public Command setAngle(Angle angle) {
    return flippyArm.setAngle(angle);
  }



  @Override
  public void periodic() {
    final boolean limitPressed = limitSwitchSupplier.getAsBoolean();

    if (limitPressed) {
      flippySmartMotorController.setEncoderPosition(Degrees.of(0));
    }

    SmartDashboard.putNumber("getFlippyABSEncoder", flippyMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("getFlippyRelativeEncoder", flippyMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("FlipMotor/LimitSwitch", limitPressed);
    SmartDashboard.putNumber("FlipMotor/Angle", getFlippyAngle().in(Degrees));
    flippyArm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    flippyArm.simIterate();
  }
}