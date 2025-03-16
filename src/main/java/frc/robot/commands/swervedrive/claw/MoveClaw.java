package frc.robot.commands.swervedrive.claw;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.utilities.Constants.SpeedConstants;

public class MoveClaw extends Command {
    ClawSubsystem clawSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    XboxController xboxController;
    double speed;
    DoubleSupplier velocity;


    public MoveClaw(ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, XboxController xboxController, DoubleSupplier velocity){
        this.clawSubsystem = clawSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.xboxController = xboxController;
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

        if(xboxController.getRightBumperButton()){
            clawSubsystem.kickerMove(SpeedConstants.KICKER_SPEED);
        }

        clawSubsystem.move(speed * 0.7);

    }

    @Override
    public void end(boolean interrupted){
        clawSubsystem.stopClaw();
    }
}
