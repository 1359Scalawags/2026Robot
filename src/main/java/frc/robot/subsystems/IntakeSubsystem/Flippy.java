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
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  private final SparkMax flippersMotor;

  private SmartMotorControllerConfig flippySmcConfig;
  private SmartMotorControllerConfig flippersSmcConfig;
  private DigitalInput limitSwitch = new DigitalInput(1);
  private SmartMotorController flippySmartMotorController;
  private SmartMotorController flipperSmartMotorController;

  private final ArmConfig flippyConfig;
  private final ArmConfig flipperConfig;

  private Arm flippyArm;
  private Arm flipperArm;

  public Flippy() {

    flippyMotor = new SparkMax(Constants.Intake.flippyMotorID, MotorType.kBrushless);
    
    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    flippySmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.Intake.flippyP, Constants.Intake.flippyI, Constants.Intake.flippyD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(Constants.Intake.flippyP, Constants.Intake.flippyI, Constants.Intake.flippyD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withTelemetry("FlipperMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromStages("64:1")))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40));

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


    //Creates the motor objects that control the motors on the real robot
    flippersMotor = new SparkMax(Constants.Intake.flipperMotorID, MotorType.kBrushless);
    
    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    //TODO: need to confiure the SMC correctly for the values and test values we want to use on the real robot
    flippersSmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.Intake.flipperP, Constants.Intake.flipperI, Constants.Intake.flipperD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(Constants.Intake.flipperP, Constants.Intake.flipperI, Constants.Intake.flipperD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        // .withFeedforward(
            // new SimpleMotorFeedforward(Constants.Intake.flippyS,Constants.Intake.flippyV,Constants.Intake.flippyA))
        // .withSimFeedforward(
            // new SimpleMotorFeedforward(Constants.Intake.flippyS, Constants.Intake.flippyV, Constants.Intake.flippyA))
        // .withExternalEncoder(flippyMotor.getAbsoluteEncoder())
        // .withExternalEncoderInverted(true)
        // .withUseExternalFeedbackEncoder(false)
        // .withExternalEncoderZeroOffset(Degrees.of(45))
        .withTelemetry("FlipperMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromStages("64:1")))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40));

    flipperSmartMotorController = new SparkWrapper(flippersMotor, DCMotor.getNEO(1), flippersSmcConfig);
    // starSmartMotorController.setEncoderInverted(true);

    //TODO: make sure these are correct too
    flipperConfig = new ArmConfig(flipperSmartMotorController)
            .withLength(Inches.of(10))
            .withMass(Pounds.of(15))
            .withStartingPosition(Degrees.of(90))
            .withHardLimit(Constants.Intake.flipperMinAngle, Constants.Intake.flipperMaxAngle)
            .withSoftLimits(Constants.Intake.flipperMinAngle, Constants.Intake.flipperMaxAngle)
            .withTelemetry("flipperMech", TelemetryVerbosity.HIGH);

    flipperArm = new Arm(flipperConfig);

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