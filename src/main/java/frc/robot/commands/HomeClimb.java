package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class HomeClimb extends Command {
  private final ClimbSubsystem climb;

  public HomeClimb(ClimbSubsystem _climb) {
    climb = _climb;
  }

  @Override
  public void execute() {
    climb.set(-0.1);
  }

  @Override
  public boolean isFinished() {
    if (!climb.hasLimitSwitch()) {
      return true;
    }
    try {
      return climb.getLimitSwitch();
    } catch (Exception e) { }
    return true;
  }
}