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

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
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

public class Star extends SubsystemBase {

  private final SparkMax starMotor;

  private SmartMotorControllerConfig starSmcConfig;


  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController starSmartMotorController;


  private final FlyWheelConfig starConfig;

  private FlyWheel starWheel;

  //TODO: create constants whereveer needed.
  public Star() {

    //Creates the motor objects that control the motors on the real robot
    starMotor = new SparkMax(Constants.Intake.starMotorID, MotorType.kBrushless);

    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    //TODO: need to confiure the SMC correctly for the values and test values we want to use on the real robot
    starSmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.Intake.starP, Constants.Intake.starI, Constants.Intake.starD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(Constants.Intake.starP, Constants.Intake.starI, Constants.Intake.starD,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withFeedforward(
            new SimpleMotorFeedforward(Constants.Intake.starS,Constants.Intake.starV,Constants.Intake.starA))
        .withSimFeedforward(
            new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
        .withTelemetry("starMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1,1)))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    starSmartMotorController = new SparkWrapper(starMotor, DCMotor.getNEO(1), starSmcConfig);

    starConfig = new FlyWheelConfig(starSmartMotorController)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(1))
        .withSoftLimit(RPM.of(-1500), RPM.of(1500))
        .withTelemetry("starMech", TelemetryVerbosity.HIGH);

    starWheel = new FlyWheel(starConfig);
  }
   /**
   * @return Shooter velocity.
   */
  public AngularVelocity getStarVelocity() {
    return starWheel.getSpeed();
  }

 /**
   * Set the kicker velocity to feed fuel into the shooter.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setStarVelocity(AngularVelocity speed) {
    return starWheel.setSpeed(speed);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setStarDutyCylce(double dutyCycle) {
    return starWheel.set(dutyCycle);
  }

   public Command sysId() {
      return starWheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
    }

  @Override
  public void periodic() {
    starWheel.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    starWheel.simIterate();
  }
}
