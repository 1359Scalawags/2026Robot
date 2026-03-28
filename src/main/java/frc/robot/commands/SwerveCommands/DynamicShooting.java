package frc.robot.commands.SwerveCommands;

import frc.robot.ShootCalculator;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSubsystem.Shooter;
import edu.wpi.first.units.measure.AngularVelocity;
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
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}