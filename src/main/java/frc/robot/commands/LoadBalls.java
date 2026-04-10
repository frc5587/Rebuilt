package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LoadBalls extends Command {
  private ArmSubsystem arm;
  private ShooterSubsystem shooter;
  private IndexerSubsystem indexer;
  private IntakeSubsystem intake;

  // private double lastTimestampNotAtGoal = Timer.getFPGATimestamp();
  private double lastAngleSwitchTimestamp = Timer.getFPGATimestamp();
  private boolean isGoingUp = true;

  private final Angle wiggleAngleUp;
  private final Angle wiggleAngleDown;
  private final double wiggleTimeUp;
  private final double wiggleTimeDown;

  public LoadBalls(ArmSubsystem _arm, ShooterSubsystem _shooter, IndexerSubsystem _indexer, IntakeSubsystem _intake,
                   Angle _wiggleAngleUp, Angle _wiggleAngleDown, double _wiggleTimeUp ,double _wiggleTimeDown
                   ) {
    arm = _arm;
    shooter = _shooter;
    indexer = _indexer;
    intake = _intake;
    wiggleAngleUp = _wiggleAngleUp;
    wiggleAngleDown = _wiggleAngleDown;
    wiggleTimeDown = _wiggleTimeDown;
    wiggleTimeUp = _wiggleTimeUp;
  }

  @Override
  public void execute() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    // if (!shooter.atGoal()) {
    //   lastTimestampNotAtGoal = Timer.getFPGATimestamp();
    // }

    scheduler.schedule(intake.set(0.3));
    if (arm != null) {
      // DOWN
      if (lastAngleSwitchTimestamp > Timer.getFPGATimestamp()-wiggleTimeDown  &&  !isGoingUp) {
        scheduler.schedule(arm.setAngle(wiggleAngleDown));
      }
        // UP
      else if (lastAngleSwitchTimestamp > Timer.getFPGATimestamp()-wiggleTimeUp) {
        scheduler.schedule(arm.setAngle(wiggleAngleUp));
      }
      else {
        isGoingUp = !isGoingUp;
        lastAngleSwitchTimestamp = Timer.getFPGATimestamp();
      }
    }

    if (shooter.atGoal().getAsBoolean()) {
      if (indexer != null) {
        scheduler.schedule(indexer.start());
      }
      shooter.startIndexing();
    }
    else {
      if (indexer != null) {
        scheduler.schedule(indexer.stop());
      }
      shooter.stopIndexing();
    }
  }

  @Override
  public void end(boolean interrupted) {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    if (arm != null) {
      scheduler.schedule(arm.bottom());
    }
    scheduler.cancel(intake.getCurrentCommand());
    if (indexer != null) {
      scheduler.cancel(indexer.getCurrentCommand());
    }
    scheduler.cancel(shooter.getCurrentCommand());
  }
}

