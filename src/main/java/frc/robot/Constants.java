// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj.RobotBase;

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

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static boolean disableHAL = false;


  public static class FieldConstants {
    // gives a generic translation 2d for the red and blue side to be used on any object.
    public static final Translation2d kBlueHubPosition = new Translation2d(4.61137, 4.021328 );
    public static final Translation2d kRedHubPosition = new Translation2d(11.901424, 4.021328);

    public static final Translation2d BLUE_NEUTRAL_POSE = new Translation2d(Units.inchesToMeters(99.085), Units.inchesToMeters(158.84));
    public static final Translation2d RED_NEUTRAL_POSE = new Translation2d(Units.inchesToMeters(552.135), Units.inchesToMeters(158.84));
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int DriverJoystick = 0;
    public static final int AssistJoystick = 1;
    public static final double DEADBAND = 0.10;
  }

  public static class swerveDrive {
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
      // ========== CONFIGURATION ==========
      // CAN IDs for the motor controllers
    public static final int flyWheelID = 13;
    public static final int fingerWheelID = 14;

      // ======== Shooter Speed =======
    public static final AngularVelocity shooterVelocity = RPM.of(3000);
    public static final AngularVelocity kickerVelocity = RPM.of(1500);

     // ====== Trapazoidal Profile =======
    public static final AngularVelocity shooterMaxVelocity = RPM.of(3000);
    public static final AngularAcceleration shooterMaxAcceleration = RotationsPerSecondPerSecond.of(20000);
    

    public static final AngularVelocity kickerMaxVelocity = RPM.of(2000);
    public static final AngularAcceleration kickerMaxAcceleration = RotationsPerSecondPerSecond.of(1250);


      // =========  PID & FF values for ShooterWheel ==============
    public static final double shooterP = 0.025;
    public static final double shooterI = 0;
    public static final double shooterD = 0.001;
    public static final double shooterSpeed = 2500;

    public static final double shooterS = 0.16811;
    public static final double shooterV = 0.12113;
    public static final double shooterA = 0.041532;


          // =========  PID & FF values for KickerWheel ==============
      //TODO This mechanisim needs to be coupled tighter, too much backlash in system to get a good tuning,
          //needs to be retuneed once mechanism is fixed and functioning properly
    public static final double kickerP  = 0.03;
    public static final double kickerI = 0.000035;
    public static final double kickerD = 0.5;

    public static final double kickerS = 0.5;
    public static final double kickerV = 0.12113;
    public static final double kickerA = 0.041532;
  }

  public static class Climber {
    public static final int climberID = 15;
    
    public static final double GEAR_RATIO = 125.0;
    public static final int CURRENT_LIMIT = 60; // Amps
    
    // PID Constants — TUNE THESE on the real robot!
    // With 125:1 gearing, max linear speed is only ~0.106 m/s.
    // Feedforward (kv) does the heavy lifting. PID just corrects small errors.
    public static final double kP = 50
    ;
    public static final double kI = 0.0;
    public static final double kD = 0.0;   // Damping to prevent overshoot
    public static final double kFF = 0.0;
    
    // Position limits (in inches)
    public static final double MAX_HEIGHT = 17.5;  // inches — top of travel
    public static final double MIN_HEIGHT = 0.0;   // inches — bottom of travel (limit switch home)
    public static final double START_HEIGHT = 0.0;    // inches — should match STOWED_HEIGHT so default command doesn't move on boot

    // Soft limits (in inches) — the closed-loop controller won't command past these
    public static final double SOFT_MIN = -0.5;     // inches — small buffer above hard bottom
    public static final double SOFT_MAX = 17.5;     // inches — small buffer below hard top

    // Feedforward — TUNE THESE with SysId or manual testing!
    // ks = static friction, kg = gravity compensation, kv = velocity
    // NOTE: kg always pushes UP. In sim there's no gravity so set to 0.
    // On the real robot, slowly increase kg until the elevator holds position with PID off.
    public static final double ks = 0;  // TODO: Tune on real robot
    public static final double kg = 0;   // Set to 0 for sim. Real robot: ~0.3-1.0 (find with SysId)
    public static final double kv = 0;   // Disable feedforward for now — let PID-only control work first

    // Preset heights (in inches)
    public static final double STOWED_HEIGHT = 0.5;    // inches — fully retracted
    public static final double CLIMB_HEIGHT = 17.0;    // inches — fully extended for climb

    // Speeds
    public static final double MAX_SPEED = 0.8; // 80% max speed
  }

  public static class Intake {

    // ========== CONFIGURATION ==========
      // CAN IDs for the motor controllers
    public static final int sushiMotorID = 10;
    public static final int flippyMotorID = 9;


      // ======= Intake Speeds ======
    public static AngularVelocity sushiVelocity = RPM.of(1500);
    public static AngularVelocity flippyVelocity = RPM.of(1000);
    public static Angle flippyMinAngle = Degrees.of(0);
    public static Angle flippyMaxAngle = Degrees.of(590);
    
      // ====== Trapazoidal Profile =======
    public static final AngularVelocity flippyMaxVelocity = RPM.of(2500);
    public static final AngularAcceleration flippyMaxAcceleration = RotationsPerSecondPerSecond.of(5000);

    public static final AngularVelocity sushiMaxVelocity = RPM.of(2000);
    public static final AngularAcceleration sushiMaxAcceleration = RotationsPerSecondPerSecond.of(1000);


      // =========  PID & FF values for SushiWheel ==============
    public static final double sushiP = 0.025; //0.029668
    public static final double sushiI = 0.00000001;
    public static final double sushiD = 0.8;

    public static final double sushiS = 0.41333;
    public static final double sushiV = 0.45;
    public static final double sushiA = 0.024823;


      // =========  PID & FF values for StarWheel ==============
          //Star motor is agrresivly tunned to help stop fuel getting stuck
    public static final double flippyP = 0.035; //From sysID - 0.055968, 0.029853
    public static final double flippyI = 0.000001;
    public static final double flippyD = 1.3;

    public static final double flippyS = 0.66508;
    public static final double flippyV = 0.10476;
    public static final double flippyA = 0.03396;
  }


  public static class Hopper {
    // ========== CONFIGURATION ==========
    // CAN IDs for the motor controllers
    public static final int jigglerMotorID = 11;

    // Motor speed in RPM
    public static final double HOPPER_SPEED_RPM = 2000;

    // Trap profile
    public static final AngularVelocity hopperMaxVelocity = RPM.of(2500);
    public static final AngularAcceleration hopperMaxAcceleration = RotationsPerSecondPerSecond.of(5000);

    // PID Constants (tune these based on your robot's performance)
    public static final double kP = 0.03;
    public static final double kI = 0.0000001;
    public static final double kD = 0.1;
    public static final double kFF = 0.000156; // Feed-forward for NEO motors

    //hopper motorFF
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // Current and speed thresholds
    public static final int CURRENT_LIMIT = 40; // Amps
    public static final double CURRENT_THRESHOLD = 35.0; // Amps for spike detection
    public static final double RPM_TOLERANCE = 100.0; // RPM
    public static final double CLOSED_LOOP_RAMP_RATE = 0.2;
  }

  public static class Limelight {
    public static final double TARGET_HEIGHT_METERS = 1.124;
    public static final double CAMERA_HEIGHT_METERS = 0.3048;
    public static final double CAMERA_PITCH_DEGREES = 0.0;
    public static final String limelight_Name = "limelight-top";
  }

}
