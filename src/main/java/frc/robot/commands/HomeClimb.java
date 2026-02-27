package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class HomeClimb extends Command {
  private final ClimbSubsystem climb;
  private final DigitalInput limitSwitch;

  public HomeClimb(ClimbSubsystem _climb) {
    climb = _climb;
    limitSwitch = climb.getLimitSwitch();
  }

  @Override
  public void execute() {
    climb.set(-0.1);
  }

  @Override
  public boolean isFinished() {
    return limitSwitch.get();
  }
}