// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.Hopper;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Star;
import frc.robot.subsystems.IntakeSubsystem.Sushi;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** An example command that uses an example subsystem. */
public class IntakeFuelCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Sushi m_sushi;
  private final Star m_star;
  private final HopperSubsystem m_hopper;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeFuelCommand(Sushi sushi, Star star, HopperSubsystem hopper) {
    m_sushi = sushi;
    m_star = star;
    m_hopper = hopper;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sushi, star);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Commands.parallel(
      m_hopper.set(0.5),
      m_star.setStarVelocity(Constants.Intake.starVelocity),
      m_sushi.setSushiVelocity(Constants.Intake.sushiVelocity))
      .withName("IntakeFuel");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
