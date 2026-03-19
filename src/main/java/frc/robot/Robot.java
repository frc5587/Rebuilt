// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.LEDController.LEDColor;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer robotContainer;
   private final LEDController ledController;
   private enum LedState { IDLE, READY_TO_SHOOT, INTAKE_STALLING, REVVING }

  private Timer disabledTimer;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    ledController = robotContainer.getLEDController();
    disabledTimer = new Timer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
    ledController.turnOffAll();
    ledController.applyColorSolid(LEDController.LEDColor.BLUE);
    ledController.startSnakeAnimation(LEDColor.BLUE, LEDColor.RED, true);
  }

  @Override
  public void disabledPeriodic() {
    if(disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.setMotorBrake(true);
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    robotContainer.teleopInit();
     // Stop any animations started while disabled so teleop patterns can take over.
    ledController.turnOffAll();

    /* LED Logic */
     final LedState[] currentState = new LedState[] {LedState.IDLE};
      ledController.setDefaultCommand(new FunctionalCommand(
        () -> {
          currentState[0] = LedState.IDLE;
          ledController.stopLimitSwitchProgressLoop();
           // Establish a known idle look at the start of teleop.
          ledController.turnOffAll();
          ledController.applyBlinkColor(LEDColor.YELLOW);
        },
        () -> {
          boolean intakeStalling = robotContainer.intakeIsStalling();
          boolean shooterAtGoal = robotContainer.shooterAtGoal();
          boolean shooterUsingNonDefaultCommand = robotContainer.shooterUsingNonDefaultCommand();

          LedState desiredState = LedState.IDLE;
          if (intakeStalling) {
            desiredState = LedState.INTAKE_STALLING;
          } else if (shooterAtGoal && shooterUsingNonDefaultCommand) { //TODO add additional check to make sure setpoint is above idle velocity, else go to yellow idle 
            desiredState = LedState.READY_TO_SHOOT;
          } else if (shooterUsingNonDefaultCommand) {
            desiredState = LedState.REVVING;
          } else {
            desiredState = LedState.IDLE;
          }
          if (desiredState != currentState[0]) {
            switch (desiredState) {
              case INTAKE_STALLING:
                ledController.applyColorBlink(LEDColor.RED, LEDColor.OFF, 0.0);
                break;
              case READY_TO_SHOOT:
                ledController.startLimitSwitchProgressLoop();
                break;
              case REVVING:
                ledController.applyColorBlink(LEDColor.YELLOW, LEDColor.OFF, 0.0);
              case IDLE:
              default:
                ledController.applyColorSolid(LEDColor.BLUE);
                break;
            }
            currentState[0] = desiredState;
          }
          if (currentState[0] == LedState.READY_TO_SHOOT) {
            ledController.runLimitSwitchProgressLoop();
          }
        },
        interrupted -> {
          currentState[0] = LedState.IDLE;
          ledController.stopLimitSwitchProgressLoop();
        },
        () -> false,
        ledController));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() { }
}
