// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class FieldConstants {
    // gives a generic translation 2d for the red and blue side to be used on any object.
      // public static final Translation2d kSpeakerPositionBLUE = new Translation2d();
      // public static final Translation2d kSpeakerPositionRED = new Translation2d();
    public static final Translation2d kBlueHubPosition = new Translation2d(4.021328, 4.61137);
    public static final Translation2d kRedHubPosition = new Translation2d(4.021328, 11.901424);
  }

  public static class Shooter {

    public static final int flyWheelID = 0;
    public static final int fingerWheelID = 0;

    public static final int shooterMotorPort = 101;
    public static final int feederMotorPort = 102;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int DriverJoystick = 0;
    public static final int AssistJoystick = 1;
    public static final double DEADBAND = 0.1;
  }

  public static class swerveDrive {
    public static final String robot = "2026robot";
    public static final String testbot = "Pearl/testbot";

    public static final double MAX_SPEED = 4;

    public static class autoAlign {
      public static final double X_REEF_ALIGNMENT_P = 0;
      public static final double Y_REEF_ALIGNMENT_P = 0;
      public static final double ROT_REEF_ALIGNMENT_P = 0;
      public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;
      public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0;
      public static final double X_SETPOINT_REEF_ALIGNMENT = 0;
      public static final double X_TOLERANCE_REEF_ALIGNMENT = 0;
      public static final double Y_SETPOINT_REEF_ALIGNMENT = 0;
      public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0;
      public static final Time DONT_SEE_TAG_WAIT_TIME = null;
      public static final Time POSE_VALIDATION_TIME = null;

    }
  }

  public static class Climber {
    public static final int elevatorMotorPort = 103;
  }

  public static class Intake {
    // ========== CONFIGURATION ==========
    // CAN IDs for the motor controllers
    public static final int KICKER_MOTOR_CAN_ID = 10;
    public static final int STAR_MOTOR_CAN_ID = 11;

    // Motor speed in RPM
    public static final double INTAKE_SPEED_RPM = 1600;

    // PID Constants (tune these based on your robot's performance)
    public static final double kP = 0.0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.000156; // Feed-forward for NEO motors

    // Current and speed thresholds
    public static final int CURRENT_LIMIT = 40; // Amps
    public static final double CURRENT_THRESHOLD = 35.0; // Amps for spike detection
    public static final double RPM_TOLERANCE = 100.0; // RPM
  }

}
