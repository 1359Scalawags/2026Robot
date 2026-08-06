package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants;


public class ShootCalculator {

  // Private Variables
  
  private static final InterpolatingDoubleTreeMap    flywheelSpeedMap = new InterpolatingDoubleTreeMap();

  private       Distance  minDistance      = Meters.of(1);
  private       Distance  maxDistance      = Meters.of(5);

  private Translation2d hubLocation;

  private Distance distanceToHub;

  private AngularVelocity finalRPM;

  static {
      // Puts values for the IDTM for the RPM of the shooter motor.
      //             KEY: Meters,    VALUE: RPM
    flywheelSpeedMap.put(1.00, 2900.0);
    flywheelSpeedMap.put(1.25, 3000.0);
    flywheelSpeedMap.put(1.50, 3200.0);
    flywheelSpeedMap.put(1.75, 3300.0);
    flywheelSpeedMap.put(2.00, 3500.0);
    flywheelSpeedMap.put(2.25, 3650.0);
    flywheelSpeedMap.put(2.50, 3600.0);
    flywheelSpeedMap.put(2.75, 3650.0);
    flywheelSpeedMap.put(3.00, 4000.0);
    flywheelSpeedMap.put(3.25, 4200.0);
  }

  public ShootCalculator() {}
  

  public AngularVelocity CalculateShooterRPM(Translation2d swerveTranslation) {
    hubLocation = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
   // robotTranslation = swerveDrive.getPose().getTranslation();

    distanceToHub = Meters.of(swerveTranslation.getDistance(hubLocation));

    if (distanceToHub.gte(minDistance) && distanceToHub.lte(maxDistance)) {
    finalRPM = RPM.of(flywheelSpeedMap.get(distanceToHub.in(Meters)));
    } else {
      finalRPM = Constants.Shooter.shooterVelocity;
    }

    return finalRPM;
  }
  

}

