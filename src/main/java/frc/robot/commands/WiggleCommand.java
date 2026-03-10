package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class WiggleCommand extends Command {
    public ArmSubsystem arm;
    public WiggleCommand(ArmSubsystem arm_) {
        this.arm = arm_;
    }
    
    @Override
    public void execute() {
    arm.set(1).until(() -> arm.getAngle().in(Degree)>=ArmConstants.MIDDLE_ANGLE.in(Degree));
    arm.set(-1).until(() -> arm.getAngle().in(Degree)<=ArmConstants.BOTTOM_ANGLE.in(Degree));
    }
    

    @Override
    public void end(boolean interrupted) {
        arm.setAngle(arm.getLastSetpoint());
    }
}
