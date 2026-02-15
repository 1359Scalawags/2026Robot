// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
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

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax sushiMotor;
  private final SparkMax starMotor;


  private SmartMotorControllerConfig sushiSmcConfig;
  private SmartMotorControllerConfig starSmcConfig;


  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sushiSmartMotorController;
  private SmartMotorController starSmartMotorController;


  private final FlyWheelConfig sushiConfig;
  private final FlyWheelConfig starConfig;

  private FlyWheel sushiWheel;
  private FlyWheel starWheel;

  //TODO: create constants whereveer needed.
  public IntakeSubsystem() {

    //Creates the motor objects that control the motors on the real robot
    sushiMotor = new SparkMax(Constants.Intake.sushiMotorID, MotorType.kBrushless);
    starMotor = new SparkMax(Constants.Intake.starMotorID, MotorType.kBrushless);

    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    //TODO: need to confiure the SMC correctly for the values and test values we want to use on the real robot
    sushiSmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(Constants.Intake.sushiP, Constants.Intake.sushiI, Constants.Intake.sushiD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(Constants.Intake.sushiP, Constants.Intake.sushiI, Constants.Intake.sushiD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // Telemetry name and verbosity level
        .withTelemetry("sushiMotor", TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        // In this example GearBox.fromReductionStages(3,4) is the same as
        // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
        // your motor.
        // You could also use .withGearing(12) which does the same thing.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    sushiSmartMotorController = new SparkWrapper(sushiMotor, DCMotor.getNEO(1), sushiSmcConfig);

    sushiConfig = new FlyWheelConfig(sushiSmartMotorController)
        // Diameter of the flywheel.
        .withDiameter(Inches.of(4))
        // Mass of the flywheel.
        .withMass(Pounds.of(1))
        // Maximum speed of the shooter.
        .withUpperSoftLimit(RPM.of(1000))
        // Telemetry name and verbosity for the arm.
        .withTelemetry("sushiMech", TelemetryVerbosity.HIGH);

    sushiWheel = new FlyWheel(sushiConfig);

    starSmcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(Constants.Intake.starP, Constants.Intake.starI, Constants.Intake.starD, DegreesPerSecond.of(90),
    DegreesPerSecondPerSecond.of(45))
    .withSimClosedLoopController(Constants.Intake.starP, Constants.Intake.starI, Constants.Intake.starD, DegreesPerSecond.of(90),
    DegreesPerSecondPerSecond.of(45))
    .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    .withTelemetry("starMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40))
    .withLooselyCoupledFollowers(sushiSmartMotorController);

    starSmartMotorController = new SparkWrapper(starMotor, DCMotor.getNEO(1), starSmcConfig);

    //TODO check these values to see if they are accurate
    
    starConfig = new FlyWheelConfig(starSmartMotorController)
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(1))
    .withUpperSoftLimit(RPM.of(1000))
    .withTelemetry("starMech", TelemetryVerbosity.HIGH);

    starWheel = new FlyWheel(starConfig);
  }

    /**
   * @return Shooter velocity.
   */
  public AngularVelocity getSushiVelocity() {
    return sushiWheel.getSpeed();
  }
  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setSushiVelocity(AngularVelocity speed) {
    return sushiWheel.setSpeed(speed);
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

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setSushiDutyCycle(double dutyCycle) {
    return sushiWheel.set(dutyCycle);
  }


  public Command setIntakeSpeed(AngularVelocity sushiSpeed, AngularVelocity starSpeed) {
       return run(() -> {sushiWheel.setSpeed(sushiSpeed);
                        starWheel.setSpeed(starSpeed);})
                        .withName("SetIntakeSpeed");
  }

    public Command stopIntake() {
    return run(() -> {sushiWheel.set(0);starWheel.set(0);})
            .withName("Stop Intake");
  }


  @Override
  public void periodic() {
    sushiWheel.updateTelemetry();
    starWheel.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    sushiWheel.simIterate();
    starWheel.simIterate();
    // This method will be called once per scheduler run during simulation
  }
}
