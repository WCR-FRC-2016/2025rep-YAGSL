package frc.robot.subsystems.swervedrive.pathplanner;

import java.util.List;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utilities.Constants;
import frc.robot.utilities.NetworkTables;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PathPlanner{


        double[] botPose = NetworkTables.getBotPos();
        double x = botPose[0];
        double z = botPose[2];

        

    // Create a list of waypoints from poses. Each pose represents one waypoint.
// The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(x, z, Rotation2d.fromDegrees(0))

);

PathConstraints constraints = new PathConstraints(1.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
// PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

// Create the path using the waypoints created above
PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
);

public PathPlannerPath getPathToLimelight(){
        return path;
}

// Prevent the path from being flipped if the coordinates are already correct
//path.preventFlipping = true;


        // public Command followPathCommand(String pathName) {
        // try{
        //         PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        //         return new FollowPathCommand(
        //                 path,
        //                 this::getPose, // Robot pose supplier
        //                 this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //                 this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
        //                 new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        //                         new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        //                         new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        //                 ),
        //                 Constants.robotConfig, // The robot configuration
        //                 () -> {
        //                 // Boolean supplier that controls when the path will be mirrored for the red alliance
        //                 // This will flip the path being followed to the red side of the field.
        //                 // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //                 var alliance = DriverStation.getAlliance();
        //                 if (alliance.isPresent()) {
        //                 return alliance.get() == DriverStation.Alliance.Red;
        //                 }
        //                 return false;
        //                 },
        //                 this // Reference to this subsystem to set requirements
        //         );
        // } catch (Exception e) {
        //         DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        //         return Commands.none();
        // }
        // }

    
}
