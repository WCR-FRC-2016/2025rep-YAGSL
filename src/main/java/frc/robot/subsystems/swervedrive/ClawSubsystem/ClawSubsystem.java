package frc.robot.subsystems.swervedrive.ClawSubsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.utilities.Constants.AbsoluteEncoderOffsets;
import frc.robot.utilities.Constants.LimitConstants;;


public class ClawSubsystem extends SubsystemBase {
    private final SparkMax claw;
    private final SparkMax kicker;
    public boolean manualMode = false;
    private boolean placingState;
    PIDController pid = new PIDController(1.5, 0, 0);
    public ClawSubsystem(){
        claw = new SparkMax(10, MotorType.kBrushed);
        kicker = new SparkMax(8, MotorType.kBrushless);
    }

    public void move(Double speed){
         if(speed > 0 && getEncoderValue() >= 0){
             speed = 0.0;
         }
         if(speed < 0 && getEncoderValue() <= LimitConstants.CLAW_LOWER_LIMIT){
            speed = 0.0;
        }
        
        // if(speed < 0 && getAbsoluteEncoderValue() - AbsoluteEncoderOffsets.ELEVATOR_OFFSET <= LimitConstants.CLAW_LOWER_LIMIT){
        //     speed = 0d;
        // }
        claw.set(speed); 
        
    }

    public void moveTo(double setpoint){

        claw.set(pid.calculate(claw.getEncoder().getPosition(), setpoint));
        
    }

    public void kickerMove(double speed){
        kicker.set(speed);
    }

    public double getEncoderValue(){
        return claw.getEncoder().getPosition();
    }

    public double getAbsoluteEncoderValue(){
        return claw.getAlternateEncoder().getPosition();
    }

    public void stopClaw(){
        claw.set(0.0);
    }

    public void stopKicker(){
        kicker.set(0.0);
    }

    public boolean getPlacingState(){
        return placingState;
    }

    public void setPlacingState(boolean state){
        placingState = state;
    }
}
