// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
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
    public static final Translation2d kBlueHubPosition = new Translation2d(4.61137, 4.021328 );
    public static final Translation2d kRedHubPosition = new Translation2d(11.901424, 4.021328);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int DriverJoystick = 0;
    public static final int AssistJoystick = 1;
    public static final double DEADBAND = 0.25;
  }

  public static class swerveDrive {
    public static final String robot = "2026robot";
    public static final String testbot = "YAGSLConfigJSON/testbot";
    public static final String flipper2026 = "YAGSLConfigJSON/Flipper2026";

    public static final double MAX_SPEED = 4;

    public static class autoAlign {
      public static final double X_REEF_ALIGNMENT_P = 0.001;
      public static final double Y_REEF_ALIGNMENT_P = 0.001;
      public static final double ROT_REEF_ALIGNMENT_P = 0.001;

      public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;
      public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 5;
      public static final double X_SETPOINT_REEF_ALIGNMENT = 1;
      public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.5;
      public static final double Y_SETPOINT_REEF_ALIGNMENT = 1;
      public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.5;

      public static final double DONT_SEE_TAG_WAIT_TIME = 1;
      public static final double POSE_VALIDATION_TIME = 2;

    }
  }

  public static class Shooter {
    public static final int flyWheelID = 13;
    public static final int fingerWheelID = 14;

    public static final int shooterMotorPort = 101;
    public static final int feederMotorPort = 102;

    public static final AngularVelocity testShooterVelocity = RPM.of(800);
    public static final AngularVelocity testKickerVelocity = RPM.of(800);

    //TODO: set these right
    public static double shooterP = 1;
    public static double shooterI = 0;
    public static double shooterD = 0;

    public static double kickerP  = 1;
    public static double kickerI = 0;
    public static double kickerD = 0;
  }

  public static class Climber {
    public static final int climberID = 15;
    
    // public static final int climberMotorPort = 103;
     // Constants
 // Change to your CAN ID
    
    public static final double GEAR_RATIO = 125.0;
    public static final int CURRENT_LIMIT = 60; // Amps
    
    // PID Constants(tune these!)
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    
    // Position limits (in rotations of the output shaft)
    public static final double MAX_HEIGHT = 100.0; // Adjust based on your mechanism
    public static final double MIN_HEIGHT = 0.0;
    
    // Speeds
    public static final double MAX_SPEED = 0.8; // 80% max speed
  }

  public static class Intake {
    // ========== CONFIGURATION ==========
    // CAN IDs for the motor controllers
    public static final int sushiMotorID = 9;
    public static final int starMotorID = 10;

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
    public static final double CLOSED_LOOP_RAMP_RATE = 0.2;

    //TODO: set these numbers correctly
    public static double sushiP = 0;
    public static double sushiI = 0;
    public static double sushiD = 0;
    public static double sushiIntakeSpeed = 100;

    public static double starP = 0;
    public static double starI = 0;
    public static double starD = 0;
    public static double starIntakeSpeed = 100;

  }

  public static class Hopper {
    // ========== CONFIGURATION ==========
    // CAN IDs for the motor controllers
    public static final int sushiMotorID = 11;

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
    public static final double CLOSED_LOOP_RAMP_RATE = 0.2;

    //TODO: set these numbers correctly
    public static double sushiP = 0;
    public static double sushiI = 0;
    public static double sushiD = 0;
    public static double sushiIntakeSpeed = 0;


  }

}
