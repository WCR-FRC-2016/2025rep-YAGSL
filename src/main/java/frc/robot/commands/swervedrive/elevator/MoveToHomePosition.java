package frc.robot.commands.swervedrive.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.utilities.Constants.AutoConstants;
import frc.robot.utilities.Constants.LimitConstants;
import frc.robot.utilities.Constants.SpeedConstants;

public class MoveToHomePosition extends Command {
    ElevatorSubsystem elevatorSubsystem;
    ClawSubsystem clawSubsystem;
    DoubleSupplier velocity;
    double p;

    public MoveToHomePosition(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        addRequirements(elevatorSubsystem);
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute() {
        if(AutoConstants.HOME_ELEVATOR_POSITION - elevatorSubsystem.getEncoderValue() < 0)   {
            p = SpeedConstants.ELEVATOR_P_DOWN;
        }
        else{
            p = SpeedConstants.ELEVATOR_P_UP;
        }
        if(elevatorSubsystem.getEncoderValue() < LimitConstants.ELEVATOR_CLAW_UP_HEIGHT_LIMIT){
            clawSubsystem.moveTo(AutoConstants.HOME_CORAL_CLAW_POSITION);
        }
        
        elevatorSubsystem.moveTo(AutoConstants.HOME_ELEVATOR_POSITION);
    }

}
