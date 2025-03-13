package frc.robot.commands.swervedrive.other;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class Rumble extends Command {
    XboxController xboxController;
    Double intensity;

    public Rumble(XboxController xboxController, Double intensity){
        this.xboxController = xboxController;
        this.intensity = intensity;
    }

    @Override
    public void execute(){
        xboxController.setRumble(RumbleType.kBothRumble, intensity);
    }

    @Override
    public void end(boolean interrupted){
        xboxController.setRumble(RumbleType.kBothRumble, 0);
    }
}
