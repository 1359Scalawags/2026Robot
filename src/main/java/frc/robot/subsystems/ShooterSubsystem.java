// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
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


    speedPIDR = flyWheel.();
      speedPIDR.setP(Constants.shooterSubsystem.kRightMotorP);
      speedPIDR.setI(Constants.shooterSubsystem.kRightMotorI);
      speedPIDR.setD(Constants.shooterSubsystem.kRightMotorD);
      speedPIDR.setFF(Constants.shooterSubsystem.kRightmotorFF);
      speedPIDR.setIZone(Constants.shooterSubsystem.kRightMotorIZ);
    speedPIDL = fingerWheel.getPIDController();
      speedPIDL.setP(Constants.shooterSubsystem.kLeftMotorP);
      speedPIDL.setI(Constants.shooterSubsystem.kLeftMotorI);
      speedPIDL.setD(Constants.shooterSubsystem.kLeftMotorD);
      speedPIDL.setFF(Constants.shooterSubsystem.kLeftmotorFF);
      speedPIDL.setIZone(Constants.shooterSubsystem.kLeftMotorIZ);
    
    shooterLimiter = new SlewRateLimiter
      (Constants.shooterSubsystem.kTwoMotorUsed ? Constants.shooterSubsystem.kShootingspeedlimit : 3500);
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
    if(!DriverStation.isTest()){
      // This method will be called once per scheduler run
      if (currentSpeed == ShooterSpeed.off) {
        targetSpeed = Constants.shooterSubsystem.kstopshootingspeed;
      }
      else if (currentSpeed == ShooterSpeed.low) {
        targetSpeed = Constants.shooterSubsystem.kAmpshootingspeed;
      }
      else {
        targetSpeed = Constants.shooterSubsystem.kShootingspeed;
      }
      double limitSpeed = shooterLimiter.calculate(targetSpeed);
      //speedPIDR.setReference(0, ControlType.kVelocity); //Right motor
      speedPIDR.setReference(Constants.shooterSubsystem.kTwoMotorUsed ? limitSpeed : 0, ControlType.kVelocity); //Right motor
      speedPIDL.setReference(Constants.shooterSubsystem.kTwoMotorUsed ? limitSpeed : 3000, ControlType.kVelocity); // Left motor
    }
    else {
      double joyX = Robot.getRobotContainer().driverGetForward()/2;
      fingerWheel.set(joyX);
      flyWheel.set(joyX);



    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
