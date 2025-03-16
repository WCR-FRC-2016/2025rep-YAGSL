package frc.robot.triggers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Triggers{

    public static Trigger povRightX(XboxController controller) {
      return new Trigger(() -> controller.getXButton() && controller.getPOV() == 90);
    }  

    public static Trigger povLeftX(XboxController controller) {
      return new Trigger(() -> controller.getXButton() && controller.getPOV() == 270);
    }  
    
  }
