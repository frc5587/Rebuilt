package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LoadBalls extends Command {
  private ArmSubsystem arm;
  private ShooterSubsystem shooter;
  private IndexerSubsystem indexer;
  private IntakeSubsystem intake;

  private double lastTimestampNotAtGoal = Timer.getFPGATimestamp();
  private boolean isGoingUp = true;

  public LoadBalls(ArmSubsystem _arm, ShooterSubsystem _shooter, IndexerSubsystem _indexer, IntakeSubsystem _intake) {
    arm = _arm;
    shooter = _shooter;
    indexer = _indexer;
    intake = _intake;
  }

  @Override
  public void execute() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    if (!shooter.atGoal()) {
      lastTimestampNotAtGoal = Timer.getFPGATimestamp();
    }

    if (shooter.atGoal()  &&  Timer.getFPGATimestamp()-lastTimestampNotAtGoal > ShooterConstants.SPIN_UP_DELAY) {
      scheduler.schedule(indexer.set(IndexerConstants.DUTY_CYCLE));
      scheduler.schedule(intake.set(1.));
      if (arm.getAngle().in(Degrees) >= ArmConstants.WIGGLE_ANGLE_DOWN.in(Degrees)+5.  &&  !isGoingUp) {
        scheduler.schedule(arm.setAngle(ArmConstants.WIGGLE_ANGLE_DOWN));
        isGoingUp = false;
      }
      else if (arm.getAngle().in(Degrees) <= ArmConstants.WIGGLE_ANGLE_UP.in(Degrees)-5.) {
        scheduler.schedule(arm.setAngle(ArmConstants.WIGGLE_ANGLE_UP));
        isGoingUp = true;
      }
      else {
        isGoingUp = false;
      }
      SmartDashboard.putBoolean("test 1", true);
    }
    else {
      scheduler.schedule(indexer.set(0));
      SmartDashboard.putBoolean("test 1", false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.schedule(arm.setAngle(ArmConstants.BOTTOM_ANGLE));
    scheduler.cancel(intake.getCurrentCommand());
    scheduler.cancel(indexer.getCurrentCommand());
    scheduler.cancel(shooter.getCurrentCommand());
  }
}
