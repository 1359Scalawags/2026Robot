package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommandFactory extends SubsystemBase {

    private final IntakeSubsystem IntakeSubsystem;

    
    public IntakeCommandFactory(IntakeSubsystem intakeSubsystem) {
        this.IntakeSubsystem = intakeSubsystem;
    }
    
    // Start the intake command 
    public Command StartIntake() { 
        return runOnce(() -> IntakeSubsystem.intakeOn()); 
    }

    // Stop the intake command 
    public Command StopIntake() { 
        return runOnce(() -> IntakeSubsystem.intakeOff()); 

    }

}
