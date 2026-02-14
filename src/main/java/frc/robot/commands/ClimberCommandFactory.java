// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants;
// import frc.robot.subsystems.ClimberSubsystem;

// public class ClimberCommandFactory {
  
//   /**
//    * Command factory to extend the climber while button is held
//    * @param climber The ClimberSubsystem instance
//    * @return Command that extends the climber
//    */
//   public static Command extend(ClimberSubsystem climber) {
//     return climber.runEnd(
//       climber::extend,  // Run while scheduled
//       climber::stop     // Stop when interrupted or button released
//     );
//   }
  
//   /**
//    * Command factory to retract the climber while button is held
//    * @param climber The ClimberSubsystem instance
//    * @return Command that retracts the climber
//    */
//   public static Command retract(ClimberSubsystem climber) {
//     return climber.runEnd(
//       climber::retract, // Run while scheduled
//       climber::stop     // Stop when interrupted or button released
//     );
//   }
  
//   /**
//    * Command factory to extend climber to full extension
//    */
//   public static Command extendToMax(ClimberSubsystem climber) {
//     return climber.run(climber::extend)
//      .until(climber::getClimberSwitchState)
//       .andThen(climber::stop);
//   }
  

//   /**
//    * Command factory to retract climber to fully retracted position
//    */
//   public static Command retractToMin(ClimberSubsystem climber) {
//     return climber.run(climber::retract)
//       .until(climber::getClimberSwitchState)
//       // .until(climber::(-Constants.Climber.MAX_SPEED))
//       .andThen(climber::stop);
//   }
  
//   /**
//    * Command factory to hold climber position (brake mode already handles this)
//    * This is mainly for explicit button binding if needed
//    * @param climber The ClimberSubsystem instance
//    * @return Command that stops the climber
//    */
//   public static Command holdPosition(ClimberSubsystem climber) {
//     return climber.runOnce(climber::stop);
//   }
  
//   /**
//    * Command factory to reset encoder position
//    * @param climber The ClimberSubsystem instance
//    * @return Command that resets encoder
//    */
//   public static Command resetEncoder(ClimberSubsystem climber) {
//     return climber.runOnce(climber::resetEncoder);
//   }
  
//   /**
//    * Command factory with custom speed control
//    * @param climber The ClimberSubsystem instance
//    * @param speed Speed from -1.0 to 1.0
//    * @return Command that runs climber at specified speed
//    */
//   public static Command setSpeed(ClimberSubsystem climber, double speed) {
//     return climber.runEnd(
//       () -> climber.setSpeed(speed),
//       climber::stop
//     );
//   }
  
//   /**
//    * Command sequence to deploy climber for endgame
//    * Example: Extends to max, waits briefly, then holds position
//    * @param climber The ClimberSubsystem instance
//    * @return Command sequence for deployment
//    */
//   public static Command deploySequence(ClimberSubsystem climber) {
//     return Commands.sequence(
//       extendToMax(climber),
//       Commands.waitSeconds(0.5),
//       holdPosition(climber)
//     );
//   }
  
//   /**
//    * Command sequence to retract climber after climb
//    * @param climber The ClimberSubsystem instance
//    * @return Command sequence for retraction
//    */
//   public static Command retractSequence(ClimberSubsystem climber) {
//     return Commands.sequence(
//       retractToMin(climber),
//       Commands.waitSeconds(0.5),
//       holdPosition(climber)
//     );
//   }
// }