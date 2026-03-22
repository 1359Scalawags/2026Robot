package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class MatchTimeSubsystem extends SubsystemBase{
  public enum InactiveFirstAlliance {
    UNKNOWN,
    RED,
    BLUE
  }

  private String rawGameData = "";
  private InactiveFirstAlliance inactiveFirstAlliance = InactiveFirstAlliance.UNKNOWN;

  @Override
  public void periodic() {
    rawGameData = DriverStation.getGameSpecificMessage();

    if (!rawGameData.isEmpty()) {
      switch (rawGameData.charAt(0)) {
        case 'R' -> inactiveFirstAlliance = InactiveFirstAlliance.RED;
        case 'B' -> inactiveFirstAlliance = InactiveFirstAlliance.BLUE;
        default -> inactiveFirstAlliance = InactiveFirstAlliance.UNKNOWN;
      }
    }

    SmartDashboard.putString("GameData/Raw", rawGameData);
    SmartDashboard.putString("GameData/InactiveFirstAlliance", inactiveFirstAlliance.name());
    SmartDashboard.putBoolean("GameData/HubActive", isHubActive());
    SmartDashboard.putNumber("GameData/MatchTime", DriverStation.getMatchTime());
  }

  public MatchTimeSubsystem() {
  }

  public boolean hasGameData() {
    return inactiveFirstAlliance != InactiveFirstAlliance.UNKNOWN;
  }

  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      return false;
    }

    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }

    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    double matchTime = DriverStation.getMatchTime();

    if (!hasGameData()) {
      return true;
    }

    boolean redInactiveFirst = inactiveFirstAlliance == InactiveFirstAlliance.RED;

    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      return true;
    } else if (matchTime > 105) {
      return shift1Active;
    } else if (matchTime > 80) {
      return !shift1Active;
    } else if (matchTime > 55) {
      return shift1Active;
    } else if (matchTime > 30) {
      return !shift1Active;
    } else {
      return true;
    }
  }
}