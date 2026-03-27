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
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
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

  private SmartMotorControllerConfig flippySmcConfig;
  private DigitalInput limitSwitch = new DigitalInput(1);
  private boolean lastLimitPressed = false;

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController flippySmartMotorController;

  //TODO: make these not fly wheels (maybe i dont actualy know)
  private final ArmConfig flippyConfig;

  // private FlyWheel flippyWheel;
  private Arm flippyArm;

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
        // .withFeedforward(
            // new SimpleMotorFeedforward(Constants.Intake.flippyS,Constants.Intake.flippyV,Constants.Intake.flippyA))
        // .withSimFeedforward(
            // new SimpleMotorFeedforward(Constants.Intake.flippyS, Constants.Intake.flippyV, Constants.Intake.flippyA))
        // .withExternalEncoder(flippyMotor.getAbsoluteEncoder())
        // .withExternalEncoderInverted(true)
        // .withUseExternalFeedbackEncoder(false)
        // .withExternalEncoderZeroOffset(Degrees.of(45))
        .withTelemetry("FlipperMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromStages("36:1")))
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
  }

  public BooleanSupplier limitSwitchSupplier = () -> {
    return !limitSwitch.get();
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
    lastLimitPressed = limitPressed;

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