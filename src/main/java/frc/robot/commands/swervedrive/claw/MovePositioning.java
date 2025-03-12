package frc.robot.commands.swervedrive.claw;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.utilities.Constants.AutoConstants;
import frc.robot.utilities.Constants.LimitConstants;

public class MovePositioning extends Command {
    ClawSubsystem clawSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    Double velocity;
    int level;

    public MovePositioning(ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.clawSubsystem = clawSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(clawSubsystem);
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        level = (int) Math.floor(elevatorSubsystem.actualEncoderValue() / (LimitConstants.ELEVATOR_HEIGHT_LIMIT / 3));

        var currentLevel = (int) elevatorSubsystem.actualEncoderValue();

        if (currentLevel <= 300 && currentLevel > 200) {
            clawSubsystem.moveTo(AutoConstants.HIGH_CORAL_CLAW_POSITION);
        } else if (currentLevel <= 200 && currentLevel > 50) {
            clawSubsystem.moveTo(AutoConstants.MIDDLE_CORAL_CLAW_POSITION);
        } else if (currentLevel <= 50 && currentLevel > 0) {
            if (clawSubsystem.getPlacingState()) {
                clawSubsystem.moveTo(AutoConstants.LOW_CORAL_CLAW_POSITION);
            } else {
                clawSubsystem.moveTo(AutoConstants.HOME_CORAL_CLAW_POSITION);
            }

        }

    }
}
