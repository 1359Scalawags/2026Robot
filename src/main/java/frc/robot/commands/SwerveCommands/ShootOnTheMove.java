package frc.robot.commands.SwerveCommands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSubsystem.Kicker;
import frc.robot.subsystems.ShooterSubsystem.Shooter;
import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;


public class ShootOnTheMove extends Command {
    private final Shooter shooter;
    private final Kicker kicker;
    private final HopperSubsystem hopper;
    private final SwerveSubsystem swerveDrive;


     // Private Variables
  
  private static final InterpolatingDoubleTreeMap    flywheelSpeedMap = new InterpolatingDoubleTreeMap();

  private Distance  minDistance = Meters.of(1);
  private Distance  maxDistance = Meters.of(5);

  private Translation2d hubLocation;

  private Translation2d robotTranslation;

  private Distance distanceToHub;

  private AngularVelocity finalRPM;

  static {
      // Puts values for the IDTM for the RPM of the shooter motor.
      //             KEY:    meters, VALUE:RPM
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
  
    public ShootOnTheMove(Shooter shooter, Kicker kicker, HopperSubsystem hopper, SwerveSubsystem swerveDrive) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.hopper = hopper;
        this.swerveDrive = swerveDrive;

        addRequirements(shooter, kicker, hopper);
    }
    


    @Override
    public void initialize() {
        shooter.setShooterDutyCycle(0);
        kicker.setKickerDutyCylce(0);
        hopper.set(0);
    }

    @Override
    public void execute() {
      hubLocation = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
      robotTranslation = swerveDrive.getPose().getTranslation();

      distanceToHub = Meters.of(robotTranslation.getDistance(hubLocation));

      if (distanceToHub.gte(minDistance) && distanceToHub.lte(maxDistance)) {
         finalRPM = RPM.of(flywheelSpeedMap.get(distanceToHub.in(Meters)));
      } else {
         finalRPM = Constants.Shooter.shooterVelocity;
      }

        shooter.setShooterVelocity(finalRPM);

        AngularVelocity shooterRPM = shooter.getShooterVelocity();

        boolean shooterReady = shooterRPM.isNear(finalRPM, RPM.of(150));

        if (shooterReady) {
            kicker.setKickerDutyCylce(0.5);
        } else {
            kicker.setKickerDutyCylce(0);
        }

        hopper.set(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterDutyCycle(0);
        kicker.setKickerDutyCylce(0);
        hopper.set(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}