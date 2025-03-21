package frc.robot.commands.swervedrive.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.utilities.Constants.AutoConstants;
import frc.robot.utilities.Constants.SpeedConstants;

public class MoveElevatorMediumCoral extends Command {
    ElevatorSubsystem elevatorSubsystem;
    ClawSubsystem clawSubsystem;
    DoubleSupplier velocity;
    double p;

    public MoveElevatorMediumCoral(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        addRequirements(elevatorSubsystem);
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute(){
        if(clawSubsystem.getEncoderValue() >= -0.12){
            p = 0;
            
        }
        if(AutoConstants.MEDIUM_CORAL_ELEVATOR_POSITION - elevatorSubsystem.getEncoderValue() < 0)   {
            p = SpeedConstants.ELEVATOR_P_DOWN;
        }
        else{
            p = SpeedConstants.ELEVATOR_P_UP;
        }

        clawSubsystem.moveTo(AutoConstants.MIDDLE_CORAL_CLAW_POSITION);
        elevatorSubsystem.moveTo(AutoConstants.MEDIUM_CORAL_ELEVATOR_POSITION, p);
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.stop();
        System.out.println("stoped moving elevator to medium coral");
    }
}
