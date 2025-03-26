package frc.robot.commands.swervedrive.auto;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;

public class OutTakeAuto extends Command {
    ClawSubsystem clawSubsystem;

    Timer timer = new Timer();

    double time;


    public OutTakeAuto(ClawSubsystem clawSubsystem, double time){
        this.clawSubsystem = clawSubsystem;
        this.time = time;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize(){
        timer.start();
    }

    @Override
    public void execute(){
        clawSubsystem.kickerMove(-0.95);
    }

    @Override
    public boolean isFinished(){
        if(timer.get() >= time){
            clawSubsystem.stopKicker();
            timer.stop();
            timer.reset();
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        clawSubsystem.stopKicker();
        timer.stop();
        timer.reset();
    }
}
