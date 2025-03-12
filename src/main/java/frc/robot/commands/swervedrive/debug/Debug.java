package frc.robot.commands.swervedrive.debug;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.FunnelSubsystem.FunnelSubsystem;

public class Debug extends Command {
    ElevatorSubsystem elevatorSubsystem;
    ClawSubsystem clawSubsystem;
    FunnelSubsystem funnelSubsystem;

    public Debug( ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, FunnelSubsystem funnelSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.funnelSubsystem = funnelSubsystem;
    }

    @Override
    public void execute(){
        System.out.println("claw:" + clawSubsystem.getEncoderValue());
        System.out.println("elevator:" + elevatorSubsystem.getEncoderValue());
        System.out.println("Funnel:" + funnelSubsystem.getAbsoluteEncoderValue());
    }


}
