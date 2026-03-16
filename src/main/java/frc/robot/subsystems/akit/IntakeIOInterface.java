package frc.robot.subsystems.akit;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIOInterface {

  @AutoLog
  public static class ShooterIOInputs {
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double targetVelocityRotationsPerSec = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setTargetVelocity(double rotationsPerSec) {}

  default void stop() {}
}