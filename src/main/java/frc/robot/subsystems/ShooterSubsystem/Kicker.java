// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSubsystem;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
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

public class Kicker extends SubsystemBase {

  private final SparkMax kickerMotor;

  private SmartMotorControllerConfig kickerSmcConfig;


  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController kickerSmartMotorController;


  private final FlyWheelConfig kickerConfig;

  private FlyWheel kickerWheel;

  //TODO: create constants whereveer needed.
  public Kicker() {

    //Creates the motor objects that control the motors on the real robot
    kickerMotor = new SparkMax(Shooter.fingerWheelID, MotorType.kBrushless);

    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    //TODO: need to confiure the SMC correctly for the values and test values we want to use on the real robot
    kickerSmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(Constants.Shooter.kickerP, Constants.Shooter.kickerI, Constants.Shooter.kickerD, DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(Constants.Shooter.kickerP, Constants.Shooter.kickerI, Constants.Shooter.kickerD, DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withFeedforward(new SimpleMotorFeedforward(Constants.Shooter.kickerS, Constants.Shooter.kickerV, Constants.Shooter.kickerA))
        .withSimFeedforward(new SimpleMotorFeedforward(Constants.Shooter.kickerS, Constants.Shooter.kickerV, Constants.Shooter.kickerA))
        .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromStages("4:1")))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(25));
        // .withTrapezoidalProfile(RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(1000));


    kickerSmartMotorController = new SparkWrapper(kickerMotor, DCMotor.getNEO(1), kickerSmcConfig);

    //TODO check these values to see if they are accurate

    kickerConfig = new FlyWheelConfig(kickerSmartMotorController)
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(1))
    .withSoftLimit(RPM.of(-1500), RPM.of(1500))
    .withTelemetry("KickerMech", TelemetryVerbosity.HIGH);

    kickerWheel = new FlyWheel(kickerConfig);
  }


    /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setKickerVelocity() {
    return kickerWheel.setSpeed(RPM.of(Constants.Shooter.kickerSpeed));
  }

    public Command setKickerVelocity(AngularVelocity speed) {
    return kickerWheel.setSpeed(speed);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setKickerDutyCylce(double dutyCycle) {
    return kickerWheel.set(dutyCycle);
  }

  public Command sysId() {
    return kickerWheel.sysId(Volts.of(12), Volts.of(0.5).per(Second), Seconds.of(30));
  }


  @Override
  public void periodic() {
    kickerWheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    kickerWheel.simIterate();
    // This method will be called once per scheduler run during simulation
  }
}
