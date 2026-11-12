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
import static edu.wpi.first.units.Units.Rotations;
import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
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
    flipperMotor = new SparkMax(Constants.Intake.flipperMotorID, MotorType.kBrushless);

    flipperSmConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.Intake.flipperP, Constants.Intake.flipperI, Constants.Intake.flipperD)
        .withSimClosedLoopController(Constants.Intake.flipperP, Constants.Intake.flipperI, Constants.Intake.flipperD)
        .withTrapezoidalProfile(Intake.intakeMaxVelocity, Intake.intakeMaxAcceleration)
        .withExternalEncoder(flipperMotor.getAbsoluteEncoder())
        .withExternalEncoderGearing(1.0)
        .withExternalEncoderInverted(false)
        .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5))
        .withExternalEncoderZeroOffset(Degrees.of(0))
        .withUseExternalFeedbackEncoder(true)
        .withTelemetry("FlipperMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromStages("64:1")))
        .withStartingPosition(Degrees.of(90))
        .withSoftLimits(Constants.Intake.flippyMinAngle, Constants.Intake.flippyMaxAngle)
        .withMomentOfInertia(Inches.of(10), Pounds.of(15))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40));

    flipperSmartMotorController = new SparkWrapper(flipperMotor, DCMotor.getNEO(1), flipperSmConfig);
    
    flipperConfig = new ArmConfig()
        .withLength(Inches.of(10))
        .withHardLimits(Constants.Intake.flippyMinAngle, Constants.Intake.flippyMaxAngle)
        .withTelemetry("flipperMech", TelemetryVerbosity.HIGH);
    
    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    flippySmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.Intake.flippyP, Constants.Intake.flippyI, Constants.Intake.flippyD)
        .withSimClosedLoopController(Constants.Intake.flippyP, Constants.Intake.flippyI, Constants.Intake.flippyD)
        .withTrapezoidalProfile(Intake.intakeMaxVelocity, Intake.intakeMaxAcceleration)
        .withExternalEncoder(flippyMotor.getAbsoluteEncoder())
        .withExternalEncoderGearing(1.0)
        .withExternalEncoderInverted(false)
        .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5))
        .withExternalEncoderZeroOffset(Degrees.of(0))
        .withUseExternalFeedbackEncoder(true)
        .withTelemetry("FlippyMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromStages("64:1")))
        .withStartingPosition(Degrees.of(90))
        .withSoftLimits(Constants.Intake.flippyMinAngle, Constants.Intake.flippyMaxAngle)
        .withMomentOfInertia(Inches.of(10), Pounds.of(15))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withLooselyCoupledFollowers(flipperSmartMotorController);
      
    
    flippySmartMotorController = new SparkWrapper(flippyMotor, DCMotor.getNEO(1), flippySmcConfig);

    flippyConfig = new ArmConfig()
        .withLength(Inches.of(10))
        .withHardLimits(Constants.Intake.flippyMinAngle, Constants.Intake.flippyMaxAngle)
        .withTelemetry("flipperMech", TelemetryVerbosity.HIGH);
    
    flippyArm = new Arm(flippyConfig, flippySmartMotorController);
    flipperArm = new Arm(flipperConfig, flipperSmartMotorController);


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
      // flippyMotor.getAbsoluteEncoder.setPosition();
      // flipperMotor.getAbsoluteEncoder().setPosition();
    }
    SmartDashboard.putNumber("FlipMotor/Flippy/externalEncoder", flippyMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("FlipMotor/Flippy/relativeEncoder", flippyMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("FlipMotor/Flipper/externalEncoder", flipperMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("FlipMotor/Flipper/relativeEncoder", flipperMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("FlipMotor/Flippy/LimitSwitch", limitPressed);
    SmartDashboard.putNumber("FlipMotor/Angle", getFlippyAngle().in(Degrees));
    flippyArm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    flippyArm.simIterate();
  }
}