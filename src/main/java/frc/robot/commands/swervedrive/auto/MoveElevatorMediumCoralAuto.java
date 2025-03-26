package frc.robot.commands.swervedrive.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.utilities.Constants.AutoConstants;
import frc.robot.utilities.Constants.SpeedConstants;

public class MoveElevatorMediumCoralAuto extends Command {
    ElevatorSubsystem elevatorSubsystem;
    ClawSubsystem clawSubsystem;
    DoubleSupplier velocity;

    public MoveElevatorMediumCoralAuto(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        addRequirements(elevatorSubsystem);
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute(){
        if(clawSubsystem.getEncoderValue() <= -0.14){
            elevatorSubsystem.moveTo(AutoConstants.MEDIUM_CORAL_ELEVATOR_POSITION);
        }

        clawSubsystem.moveTo(AutoConstants.MIDDLE_CORAL_CLAW_POSITION);
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(AutoConstants.MIDDLE_CORAL_CLAW_POSITION - clawSubsystem.getEncoderValue()) < 0.05 && Math.abs(AutoConstants.MEDIUM_CORAL_ELEVATOR_POSITION - elevatorSubsystem.getEncoderValue()) < 2){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.stop();
        System.out.println("stoped moving elevator to medium coral");
    }
}
