package frc.robot.subsystems.swervedrive.ElevatorSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Constants.AbsoluteEncoderOffsets;
import frc.robot.utilities.Constants.LimitConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevator;
    PIDController pid = new PIDController(0.1, 0, 0);
    double offset;
    double upperLimit = LimitConstants.ELEVATOR_HEIGHT_LIMIT;
    DigitalInput bottomMagEncoder = new DigitalInput(0);
    public AnalogInput ultrasonicSensor = new AnalogInput(1);
    public ElevatorSubsystem(){
        elevator = new SparkMax(6, MotorType.kBrushless);
        pid.setTolerance(1);
    }

    public void move(double speed){     
        if(speed < 0 && !bottomMagEncoder.get()){
            offset = elevator.getEncoder().getPosition();
            speed = 0;
        }
        if(speed > 0 && actualEncoderValue() >= upperLimit){
            speed = 0;
        }

        elevator.set(speed);

    }

    public void moveTo(double setpoint){
        var currentPoint = elevator.getEncoder().getPosition();

        if(setpoint < currentPoint) {
            pid.setP(0.05);
        } else {
            pid.setP(0.1);
        }

        // pid.setP(p);
        elevator.set(pid.calculate(elevator.getEncoder().getPosition()/* - offset*/, setpoint));
    }

    public double getEncoderValue(){
        return elevator.getEncoder().getPosition() - offset;
    }

    public double getAbsoluteEncoderValue(){
        return elevator.getAlternateEncoder().getPosition();
    }

    public void stop(){
        elevator.set(0.0);
    }

    public double actualEncoderValue(){
        return elevator.getAbsoluteEncoder().getPosition() - AbsoluteEncoderOffsets.ELEVATOR_OFFSET;
    }
}
