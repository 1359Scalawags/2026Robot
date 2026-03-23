package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import frc.robot.subsystems.ShooterSubsystem.Shooter;
import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightHelpers.PoseEstimate;


public class ShootCalculator {

  // Private Variables
  private              SwerveSubsystem               swerveDrive;
  
  private static final InterpolatingDoubleTreeMap    flywheelSpeedMap = new InterpolatingDoubleTreeMap();

  private       Distance  minDistance      = Feet.of(1);
  private       Distance  maxDistance      = Feet.of(5);

  private Translation2d hubLocation;

  private Translation2d robotTranslation;

  private Distance distanceToHub;

  private AngularVelocity finalRPM;

  static {
      // Puts values for the IDTM for the RPM of the shooter motor.
      //             KEY: feet,    VALUE: RPM
    flywheelSpeedMap.put(1.34, 2100.0);
    flywheelSpeedMap.put(1.78, 2200.0);
    flywheelSpeedMap.put(2.17, 2200.0);
    flywheelSpeedMap.put(2.81, 2300.0);
    flywheelSpeedMap.put(3.82, 2500.0);
    flywheelSpeedMap.put(4.09, 2550.0);
    flywheelSpeedMap.put(4.40, 2600.0);
    flywheelSpeedMap.put(4.77, 2650.0);
    flywheelSpeedMap.put(5.57, 2750.0);
    flywheelSpeedMap.put(5.60, 2900.0);
  }

  public ShootCalculator(SwerveSubsystem swerveDrive) {}

  public AngularVelocity CalculateShooterRPM(Pose2d robotPose) {
    hubLocation = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    robotTranslation = swerveDrive.getPose().getTranslation();


    distanceToHub = Feet.of(robotTranslation.getDistance(hubLocation));


    if (distanceToHub.gte(minDistance) && distanceToHub.lte(maxDistance)) {
      finalRPM = RPM.of(flywheelSpeedMap.get(distanceToHub.in(Feet)));
    }

    return finalRPM;
  }
}

