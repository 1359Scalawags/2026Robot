// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.google.flatbuffers.Constants;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import frc.robot.Constants.*;
public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax flyWheelMotor = new SparkMax(Shooter.flyWheelID, MotorType.kBrushless);
  private final SparkMax fingerWheelMotor = new SparkMax(Shooter.fingerWheelID, MotorType.kBrushless);

  private final SmartMotorControllerConfig flyWheelConfig = new SmartMotorControllerConfig(this) 

    .withClosedLoopController(0,0,0,RPM.of(0),RotationsPerSecondPerSecond.of(0))
    .withIdleMode(MotorMode.COAST);
  
  private final SmartMotorController flyWheelController = new SparkWrapper(flyWheelMotor, DCMotor.getNeo550(1), flyWheelConfig);


  private final SmartMotorControllerConfig fingerWheelConfig = new SmartMotorControllerConfig(this) 

    .withClosedLoopController(0,0,0,RPM.of(0),RotationsPerSecondPerSecond.of(0))
    .withIdleMode(MotorMode.COAST)
    .withGearing(0);

  private final SmartMotorController fingerWheelController = new SparkWrapper(fingerWheelMotor, DCMotor.getNEO(1), fingerWheelConfig);
 
  enum ShooterSpeed {
    off,
    low,
    full
  }

  ShooterSpeed currentSpeed;

  public ShooterSubsystem() {
    
  }

  public void spinShootingMotor() {
    
  }

  public void ampSpinShootingMotor() {

  }

  public void stopSpinShootingMotor() {

  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
