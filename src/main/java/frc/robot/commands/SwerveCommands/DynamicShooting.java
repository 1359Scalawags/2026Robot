package frc.robot.commands.SwerveCommands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import frc.robot.Constants;
import frc.robot.ShootCalculator;
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


public class DynamicShooting extends Command {
    private final Shooter shooter;
    private final ShootCalculator shootCalculator;

    private AngularVelocity finalRPM;

    public DynamicShooting(Shooter shooter, SwerveSubsystem swerveDrive, ShootCalculator shootCalculator) {
        this.shooter = shooter;
        this.shootCalculator = shootCalculator;

        addRequirements(shooter);
    }
    


    @Override
    public void initialize() {
        shooter.setShooterDutyCycle(0);
    }

    @Override
    public void execute() {

        finalRPM = shootCalculator.CalculateShooterRPM();

        shooter.setShooterVelocity(finalRPM);

        // AngularVelocity shooterRPM = shooter.getShooterVelocity();

        // boolean shooterReady = shooterRPM.isNear(finalRPM, RPM.of(150));

    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterDutyCycle(0);    
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}