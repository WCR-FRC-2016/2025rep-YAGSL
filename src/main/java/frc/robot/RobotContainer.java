// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Test comment, please ignore!

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utilities.Constants.OperatorConstants;
import frc.robot.utilities.Constants.SpeedConstants;
import frc.robot.commands.swervedrive.Led.LedControll;
import frc.robot.commands.swervedrive.Led.MovingRainbow;
import frc.robot.commands.swervedrive.actuator.PullActuator;
import frc.robot.commands.swervedrive.actuator.PushActuator;
import frc.robot.commands.swervedrive.claw.MoveClaw;
import frc.robot.commands.swervedrive.claw.MoveClawHighCoral;
import frc.robot.commands.swervedrive.claw.MoveKicker;
import frc.robot.commands.swervedrive.claw.MovePositioning;
import frc.robot.commands.swervedrive.claw.PlaceCoral;
import frc.robot.commands.swervedrive.debug.Debug;
import frc.robot.commands.swervedrive.drivebase.LimelightAlign;
import frc.robot.commands.swervedrive.drivebase.LimelightDriveAlignCommand;
import frc.robot.commands.swervedrive.elevator.MoveElevator;
import frc.robot.commands.swervedrive.elevator.MoveElevatorHighCoral;
import frc.robot.commands.swervedrive.elevator.MoveElevatorMediumCoral;
import frc.robot.commands.swervedrive.elevator.MoveToHomePosition;
import frc.robot.commands.swervedrive.funnel.MoveFunnelDown;
import frc.robot.commands.swervedrive.funnel.MoveFunnelUp;
import frc.robot.commands.swervedrive.other.Rumble;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.ActuatorSubsystem.ActuatorSubsystem;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.FunnelSubsystem.FunnelSubsystem;
import frc.robot.subsystems.swervedrive.LedSubsystem.LedSubsystem;
import frc.robot.triggers.triggers;

import java.io.File;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverCommandXbox = new CommandXboxController(0);
  final         XboxController driverXbox = new XboxController(0);
  final         CommandXboxController manipulatorCommandXbox = new CommandXboxController(1);
  final         XboxController manipulatorXbox  = new XboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverCommandXbox.getLeftY(),
                                                                () -> driverCommandXbox.getLeftX())
                                                            .withControllerRotationAxis(driverCommandXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverCommandXbox::getRightX,
                                                                                             driverCommandXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverCommandXbox.getLeftY(),
                                                                        () -> -driverCommandXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverCommandXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverCommandXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverCommandXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

   private final ActuatorSubsystem actuatorSubsystem = new ActuatorSubsystem();
   private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
   private final FunnelSubsystem funnelSubsystem = new FunnelSubsystem();
   private final ClawSubsystem clawSubsystem = new ClawSubsystem();
   private final LedSubsystem ledSubsystem = new LedSubsystem(9, 220);

  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("LimelightAlign", new LimelightAlign(drivebase));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    //driverCommandXbox.x().whileTrue(new LimelightDriveAlignCommand(drivebase, 1, 0));
    triggers.povRightX(driverXbox).whileTrue(new LimelightDriveAlignCommand(drivebase, 0.4, -0.9));
    triggers.povLeftX(driverXbox).whileTrue(new LimelightDriveAlignCommand(drivebase, 0, -0.9));
    driverCommandXbox.y().whileTrue(new LimelightAlign(drivebase));
    driverCommandXbox.back().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    driverCommandXbox.start().whileTrue(new Debug(elevatorSubsystem, clawSubsystem, funnelSubsystem));
    driverCommandXbox.rightTrigger(0.1).whileTrue(new Rumble(manipulatorXbox, 0.5));

    manipulatorCommandXbox.povRight().whileTrue(new MoveFunnelUp(funnelSubsystem, SpeedConstants.FUNNEL_SPEED));
    manipulatorCommandXbox.povLeft().whileTrue(new MoveFunnelDown(funnelSubsystem, SpeedConstants.FUNNEL_SPEED));
    manipulatorCommandXbox.povUp().whileTrue(new PullActuator(actuatorSubsystem, SpeedConstants.ACTUATOR_SPEED));
    manipulatorCommandXbox.povDown().whileTrue(new PushActuator(actuatorSubsystem, SpeedConstants.ACTUATOR_SPEED));
    manipulatorCommandXbox.rightBumper().whileTrue(new MoveKicker(clawSubsystem, SpeedConstants.KICKER_SPEED));
    manipulatorCommandXbox.leftBumper().whileTrue(new MoveKicker(clawSubsystem, -SpeedConstants.KICKER_SPEED));
    manipulatorCommandXbox.a().whileTrue(new MoveToHomePosition(elevatorSubsystem, clawSubsystem));
    manipulatorCommandXbox.y().whileTrue(new MoveElevatorHighCoral(elevatorSubsystem, clawSubsystem));
    manipulatorCommandXbox.x().whileTrue(new MoveElevatorMediumCoral(elevatorSubsystem, clawSubsystem));
    manipulatorCommandXbox.rightTrigger(0.1).whileTrue(new Rumble(driverXbox, 0.5));

      // clawSubsystem.setDefaultCommand(
      //   new MovePositioning(clawSubsystem, elevatorSubsystem));

    elevatorSubsystem.setDefaultCommand(
      new MoveElevator(elevatorSubsystem, clawSubsystem, () -> MathUtil.applyDeadband(manipulatorCommandXbox.getLeftY(), 0.3) * -SpeedConstants.ELEVATOR_SPEED));

    clawSubsystem.setDefaultCommand(
      new MoveClaw(clawSubsystem, elevatorSubsystem, () -> MathUtil.applyDeadband(manipulatorCommandXbox.getRightY(), 0.3) * -SpeedConstants.CLAW_SPEED));
      

    
      


    
    
      ledSubsystem.setDefaultCommand(new MovingRainbow(ledSubsystem));

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    

    if (RobotBase.isSimulation())
    {
      //drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      //drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    }

    if (Robot.isSimulation())
    {
      driverCommandXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverCommandXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    // if (DriverStation.isTest())
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    //   driverCommandXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverCommandXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    //   driverCommandXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //   driverCommandXbox.back().whileTrue(drivebase.centerModulesCommand());
    //   driverCommandXbox.leftBumper().onTrue(Commands.none());
    //   driverCommandXbox.rightBumper().onTrue(Commands.none());
    // } else
    // {
    //   driverCommandXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //   driverCommandXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    //   driverCommandXbox.b().whileTrue(
    //       drivebase.driveToPose(
    //           new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           );
    //   driverCommandXbox.start().whileTrue(Commands.none());
    //   driverCommandXbox.back().whileTrue(Commands.none());
    //   driverCommandXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverCommandXbox.rightBumper().onTrue(Commands.none());
    // }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("SmallTestAuto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  // public static Trigger povRightX(XboxController controller) {
  //     return new Trigger(() -> controller.getXButton() && controller.getPOVCount() == 90);
  // }

}
