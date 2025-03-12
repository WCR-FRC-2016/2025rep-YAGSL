package frc.robot.subsystems.swervedrive.FunnelSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Constants.AbsoluteEncoderOffsets;

public class FunnelSubsystem extends SubsystemBase {
    private final SparkMax funnel;

    public FunnelSubsystem(){
        funnel = new SparkMax(7, MotorType.kBrushed);

    }

    public void moveUp(double x){
        funnel.set(x);
        //System.out.println("encoder:" + funnel.getEncoder().getPosition());
    }

    public void moveDown(double x){
        funnel.set(-x);
        //System.out.println("encoder:" + funnel.getEncoder().getPosition());
    }

    public double getEncoderValue(){
        return funnel.getEncoder().getPosition();
    }

    public double getAbsoluteEncoderValue(){
        return funnel.getAbsoluteEncoder().getPosition();
    }

    public double getActualEncodervalue(){
        return funnel.getAbsoluteEncoder().getPosition() - AbsoluteEncoderOffsets.FUNNEL_OFFSET;
    }

    public void stop(){
        funnel.set(0.0);
    }
}
