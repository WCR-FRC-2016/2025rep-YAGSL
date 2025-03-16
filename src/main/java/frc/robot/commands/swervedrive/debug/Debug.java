package frc.robot.commands.swervedrive.debug;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.FunnelSubsystem.FunnelSubsystem;
import frc.robot.utilities.NetworkTables;

public class Debug extends Command {
    ElevatorSubsystem elevatorSubsystem;
    ClawSubsystem clawSubsystem;
    FunnelSubsystem funnelSubsystem;
    private Pigeon2 pigeon2;

    public Debug( ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, FunnelSubsystem funnelSubsystem,  Pigeon2 pigeon2){
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.funnelSubsystem = funnelSubsystem;
        this.pigeon2 = pigeon2;
    }

    @Override
    public void execute(){
        System.out.println("claw:" + clawSubsystem.getEncoderValue());
        System.out.println("elevator:" + elevatorSubsystem.getEncoderValue());
        System.out.println("Funnel:" + funnelSubsystem.getAbsoluteEncoderValue());
        System.out.println("Tx:" + NetworkTables.getTx());
        System.out.println("Pigeon2:" + pigeon2.getRotation3d());
    }


}
