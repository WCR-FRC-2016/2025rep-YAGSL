package frc.robot.commands.swervedrive.claw;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;

public class PlaceCoral extends Command {
    ClawSubsystem clawSubsystem;
    double speed;

    public PlaceCoral(ClawSubsystem clawSubsystem, double speed){
        this.clawSubsystem = clawSubsystem;
        this.speed = speed;
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute(){
        clawSubsystem.setPlacingState(true);
        clawSubsystem.kickerMove(speed);
    }

    @Override
    public void end(boolean interrupted){
        clawSubsystem.setPlacingState(false);
        clawSubsystem.stopKicker();
    }
}
