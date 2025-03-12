package frc.robot.commands.swervedrive.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;

public class MoveElevator extends Command {
    ElevatorSubsystem elevatorSubsystem;
    ClawSubsystem clawSubsystem;
    DoubleSupplier velocity;
    double speed;

    public MoveElevator(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, DoubleSupplier velocity){
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.velocity = velocity;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute(){
        speed = velocity.getAsDouble();
        //if(elevatorSubsystem.getEncoderValue() < 10){
        if(speed > 0 && elevatorSubsystem.getEncoderValue() >= 130 && clawSubsystem.getEncoderValue() > -0.07){
            speed = 0;
            System.out.println("preventing self destruction");
        }

        if(speed > 0 && elevatorSubsystem.getEncoderValue() >= 330){
            speed = 0;
        }
        elevatorSubsystem.move(speed);
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.stop();
        System.out.println("stoped moving elevator");
    }
}
