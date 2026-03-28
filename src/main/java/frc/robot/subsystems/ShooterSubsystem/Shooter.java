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
import static edu.wpi.first.units.Units.Volts;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShootCalculator;
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

public class Shooter extends SubsystemBase {

  private final SparkMax shooterMotor;


  private SmartMotorControllerConfig shooterSmcConfig;


  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController shooterSmartMotorController;


  private final FlyWheelConfig shooterConfig;

  private FlyWheel shooterWheel;

  //TODO: create constants whereveer needed.
  public Shooter() {

    //Creates the motor objects that control the motors on the real robot
    shooterMotor = new SparkMax(Constants.Shooter.flyWheelID, MotorType.kBrushless);    

    //YAMS SmartMotorController generic config to configure the motors, ID, PIDF, gearing, idlemode... etc
    //TODO: need to confiure the SMC correctly for the values and test values we want to use on the real robot
    shooterSmcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(Constants.Shooter.shooterP, Constants.Shooter.shooterI, Constants.Shooter.shooterD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(Constants.Shooter.shooterP, Constants.Shooter.shooterI, Constants.Shooter.shooterD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(Constants.Shooter.shooterS, Constants.Shooter.shooterV, Constants.Shooter.shooterA))
        .withSimFeedforward(new SimpleMotorFeedforward(Constants.Shooter.shooterS, Constants.Shooter.shooterV, Constants.Shooter.shooterA))
        // Telemetry name and verbosity level
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        // In this example GearBox.fromReductionStages(3,4) is the same as
        // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
        // your motor.
        // You could also use .withGearing(12) which does the same thing.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
        // Motor properties to prevent over currenting.
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(35))
        .withTrapezoidalProfile(Constants.Shooter.shooterMaxVelocity,Constants.Shooter.shooterMaxAcceleration);


    shooterSmartMotorController = new SparkWrapper(shooterMotor, DCMotor.getNEO(1), shooterSmcConfig);

    //TODO check these values to see if they are accurate
    shooterConfig = new FlyWheelConfig(shooterSmartMotorController)
        // Diameter of the flywheel.
        .withDiameter(Inches.of(4))
        // Mass of the flywheel.
        .withMass(Pounds.of(1))
        // Maximum speed of the shooter.
        .withSoftLimit(RPM.of(-4000), RPM.of(4000))
        // Telemetry name and verbosity for the arm.
        .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);
        
    shooterWheel = new FlyWheel(shooterConfig);
  }
  
  public AngularVelocity getShooterVelocity() {
    return shooterWheel.getSpeed();
  }

  public double getShooterVelAsDouble() {
    return  shooterWheel.getSpeed().in(RPM);
  }

    public Command setShooterVelocity(AngularVelocity speed) {
    return shooterWheel.setSpeed(speed).withName("Shooter Wheel set Vel");
  }

  // Set the dutycycle of the shooter.
  public Command setShooterDutyCycle(double dutyCycle) {
    return shooterWheel.set(dutyCycle).withName("Shooter Wheel set Duty");
  }

  // Set the dutycycle of the shooter.
  public Command setShooterDutyCylce(double dutyCycle) {
    return shooterWheel.set(dutyCycle);
  }
  
  
  public Command sysId() {
    return shooterWheel.sysId(Volts.of(12), Volts.of(0.5).per(Second), Seconds.of(30));
  }


  @Override
  public void periodic() {
    // AngularVelocity finalRPM = ShootCalculator.CalculateShooterRPM();
    shooterWheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooterWheel.simIterate();
    SmartDashboard.putNumber("ShooterVelocity", getShooterVelAsDouble());
    // This method will be called once per scheduler run during simulation
  }
}
