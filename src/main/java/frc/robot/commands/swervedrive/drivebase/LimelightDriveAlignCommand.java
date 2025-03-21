package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utilities.NetworkTables;

public class LimelightDriveAlignCommand extends Command {
    private SwerveSubsystem driveBase;
    private double desiredAngle;
    private double offsetX;
    private double offsetZ;
    private double yaw;
    private static double desiredDistanceZ = -1;
    private static double desiredDistanceX = 0;
    private double actualDistanceZ;
    private double actualDistanceX;
    private boolean aligned;

    // Uses offsetX and offsetY to change the position of the alignment in robotcontainer and not have to create multiple commands for alignment
    public LimelightDriveAlignCommand(SwerveSubsystem swerve, double offsetX, double offsetZ) {
        driveBase = swerve;
        this.offsetX = offsetX;
        this.offsetZ = offsetZ;
        addRequirements(driveBase);
    }

    @Override
    public void initialize() {
        aligned = false;
    }

    @Override
    public void execute() {

        var botpose = NetworkTables.getBotPos();

        // Check connection to Network table
        if (botpose.length == 0) {
            System.out.println(botpose.length);
            //System.out.println("No bot Pose");
            driveBase.drive(0, 0, driveBase.getHeading().getRadians());
            return;
        }
        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) != 1){
            System.out.println("no traget");
            driveBase.drive(0, 0, driveBase.getHeading().getRadians());
            return;
        }



        // Check if able to see April Tag and if close to target stop moving and if not
        // seek
        // if (botpose[0] == 0 && botpose[1] == 0 && botpose[2] == 0) {
        //     if (closeToTarget == true) {
        //         driveBase.drive(0, 0, driveBase.getHeading().getRadians());
        //     }
        //     // else{
        //     // driveBase.drive(0, 0, driveBase.getHeading().getRadians() +
        //     // Math.toRadians(25) * turnMagnitude);
        //     // }
        //     return;
        // }
        desiredDistanceX = offsetX;
        desiredDistanceZ = offsetZ;
        System.out.println("distance: " + botpose[2] + " | tx: " + NetworkTables.getTx());
        actualDistanceX = botpose[0];
        actualDistanceZ = botpose[2];
        yaw = botpose[4];
        double distanceToMoveZ = -(actualDistanceZ - desiredDistanceZ);
        double distanceToMoveX = (actualDistanceX - desiredDistanceX);
        desiredAngle = driveBase.getHeading().getDegrees() + (yaw/2); // !!!!!!!CHANGED FROM NetworkTables.getTx() !!!!!!!

        // if (Math.abs(actualDistanceX) <= 0.05d + Math.abs(offsetX) && Math.abs(actualDistanceZ) <= 0.05 + Math.abs(offsetZ) && Math.abs(yaw) <= 1) {
        //     System.out.println("Finished alignment");
        //     aligned = true;
        // }

        // if(!aligned){
        //     driveBase.drive((distanceToMoveZ - offsetZ) * 0.3, (distanceToMoveX - offsetX) * 0.3, Math.toRadians(desiredAngle));
        // }

        // if(aligned){
        //     driveBase.drive((distanceToMoveZ) * 0.3 , (distanceToMoveX) * 0.3, driveBase.getHeading().getRadians());
        // }
        driveBase.drive((distanceToMoveZ) * 0.3 , (distanceToMoveX) * 0.3, Math.toRadians(desiredAngle));
        
        System.out.println("angle: " + desiredAngle);
        
    }

    @Override
    public boolean isFinished() {

        // return Math.abs( currentAngle) <= 0.1d;
        if (Math.abs(actualDistanceX) <= 0.05d + Math.abs(offsetX) && Math.abs(actualDistanceZ) <= 0.05 + Math.abs(offsetZ) && Math.abs(yaw) <= 1) {
            System.out.println("Finished alignment");
            return true;
        }
        return false;
    }

    // public double getTx() {

    //     return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    // }

    // private double getZpos() {
    //     var botpose = NetworkTables.getBotPos();
    //     if (botpose.length == 0) {        
    //         return desiredDistanceZ;
    //     }
    //     return NetworkTables.getBotPos()[2];
    // }       
    
}
