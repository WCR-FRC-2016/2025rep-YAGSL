package frc.robot.commands.swervedrive.claw;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;

public class MoveClaw extends Command {
    ClawSubsystem clawSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    double speed;
    DoubleSupplier velocity;


    public MoveClaw(ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, DoubleSupplier velocity){
        this.clawSubsystem = clawSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.velocity = velocity;
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute(){
        speed = velocity.getAsDouble();
        if(velocity.getAsDouble() > 0 && elevatorSubsystem.getEncoderValue() > 120 && clawSubsystem.getEncoderValue() > -0.07){
            speed = 0;
            System.out.println("stopping claw damage");
        }

        clawSubsystem.move(speed);

    }

    @Override
    public void end(boolean interrupted){
        clawSubsystem.stopClaw();
    }
}
