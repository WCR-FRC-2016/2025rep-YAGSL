package frc.robot.commands.swervedrive.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.utilities.Constants.AutoConstants;

public class MoveElevatorLowCoral extends Command {
    ElevatorSubsystem elevatorSubsystem;
    ClawSubsystem clawSubsystem;
    DoubleSupplier velocity;

    public MoveElevatorLowCoral(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        addRequirements(elevatorSubsystem);
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.moveTo(AutoConstants.LOW_CORAL_ELEVATOR_POSITION);
        if(elevatorSubsystem.getEncoderValue() < 130){
            clawSubsystem.moveTo(AutoConstants.LOW_CORAL_CLAW_POSITION);
        }

    }

}
