package frc.robot.commands;

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
  private double lastAngleSwitchTimestamp = Timer.getFPGATimestamp();
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

    scheduler.schedule(intake.set(1.));
    // DOWN
    if (lastAngleSwitchTimestamp > Timer.getFPGATimestamp()-ArmConstants.WIGGLE_TIME_DOWN  &&  !isGoingUp) {
      scheduler.schedule(arm.setAngle(ArmConstants.WIGGLE_ANGLE_DOWN));
    }
    // UP
    else if (lastAngleSwitchTimestamp > Timer.getFPGATimestamp()-ArmConstants.WIGGLE_TIME_UP) {
      scheduler.schedule(arm.setAngle(ArmConstants.WIGGLE_ANGLE_UP));
    }
    else {
      isGoingUp = !isGoingUp;
      lastAngleSwitchTimestamp = Timer.getFPGATimestamp();
    }

    if (shooter.atGoal()  &&  Timer.getFPGATimestamp()-lastTimestampNotAtGoal > ShooterConstants.SPIN_UP_DELAY) {
      scheduler.schedule(indexer.set(IndexerConstants.DUTY_CYCLE));
    }
    else {
      scheduler.schedule(indexer.set(0));
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
