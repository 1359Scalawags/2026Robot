package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class ShootCalculator {

  // Private Variables
  private              SwerveSubsystem               swerveDrive;
  
  private static final InterpolatingDoubleTreeMap    flywheelSpeedMap = new InterpolatingDoubleTreeMap();

  private       Distance  minDistance      = Meters.of(1);
  private       Distance  maxDistance      = Meters.of(5);

  private Translation2d hubLocation;

  private Translation2d robotTranslation;

  private Distance distanceToHub;

  private AngularVelocity finalRPM;

  static {
      // Puts values for the IDTM for the RPM of the shooter motor.
      //             KEY: Meters,    VALUE: RPM
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

  public ShootCalculator(SwerveSubsystem swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  public AngularVelocity CalculateShooterRPM() {
    hubLocation = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    robotTranslation = swerveDrive.getPose().getTranslation();

    distanceToHub = Meters.of(robotTranslation.getDistance(hubLocation));

    // if (distanceToHub.gte(minDistance) && distanceToHub.lte(maxDistance)) {
    finalRPM = RPM.of(flywheelSpeedMap.get(distanceToHub.in(Meters)));
    // } else {
    //   finalRPM = Constants.Shooter.shooterVelocity;
    // }

    return finalRPM;
  }
  

}

