// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkParameters;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {

  private SparkMax flyWheel;
  private SparkMax fingerWheel;

  private double targetSpeed;
  private SparkClosedLoopController flySpeedPID;
  private SparkClosedLoopController fingerSpeedPID;
  SlewRateLimiter shooterLimiter;

  enum ShooterSpeed {
    off,
    low,
    full
  }

  ShooterSpeed currentSpeed;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    flyWheel = new SparkMax(Constants.Shooter.flyWheelID, SparkLowLevel.MotorType.kBrushless);
      flyWheel.configure(SparkMaxConfig.Presets.REV_NEO_2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    fingerWheel = new SparkMax(Constants.Shooter.fingerWheelID, SparkLowLevel.MotorType.kBrushless);
      fingerWheel.configure(SparkMaxConfig.Presets.REV_NEO_550, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    currentSpeed = ShooterSpeed.off;
  }

  public void spinShootingMotor() {
    currentSpeed = ShooterSpeed.full;

    // fingerWheel.set(Constants.Shooter.kShootingspeed);
    // flyWheel.set(-Constants.Shooter.kShootingspeed);
  }

  public void ampSpinShootingMotor() {
    currentSpeed = ShooterSpeed.low;

    // fingerWheel.set(Constants.Shooter.kIdleshootingspeed);
    // flyWheel.set(-Constants.Shooter.kIdleshootingspeed);

  }

  public void stopSpinShootingMotor() {
    currentSpeed = ShooterSpeed.off;

    // fingerWheel.set(Constants.Shooter.kstopshootingspeed);
    // flyWheel.set(-Constants.Shooter.kstopshootingspeed);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
