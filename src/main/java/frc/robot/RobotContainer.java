// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;
import frc.robot.Constants.AutoConstants;
// import frc.robot.commands.*;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandGroupBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import java.io.IOException;
// import java.nio.file.Path;
// import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;




/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Joystick m_controller = new Joystick(0);
  private final XboxController controller = new XboxController(1);
  
  private final IntakeSubsystem intakes = IntakeSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final AirSubSystem air = AirSubSystem.getInstance();

  // Default Trajectory Config
  TrajectoryConfig defaultConfig;

  private static final SendableChooser<String> AutoPath = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Y axis -> forward and backwards movement
    // X axis -> left and right movement
    // Twist -> rotation
    // THE SWITCH TO ROBOT ORIENTED DRIVING IS LOCATED IN DEFAULTDRIVECOMMAND.JAVA
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(controller.getRightX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(controller.getLeftX()*.9) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));



        //This is the unmodified Joystick COntrol below
        // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        //   m_drivetrainSubsystem,
        //   () -> -modifyAxis(m_controller.getY() * .5) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        //   () -> -modifyAxis(m_controller.getX() * .5) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        //   () -> -modifyAxis(m_controller.getTwist() * .5) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // Configure the button bindings  
    configureButtonBindings();

    //All the options for Autos should go here!]
    AutoPath.addOption("BlueTop", "BlueTop");
    AutoPath.addOption("OneMeter", "OneMeter");
    AutoPath.addOption("4Ball", "4Ball");
    AutoPath.addOption("DefaultDance", "DefaultDance");
    AutoPath.setDefaultOption("DefaultDance", "DefaultDance");
    SmartDashboard.putData("Auto Pathing", AutoPath);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Command shoottheballs = new RunCommand(() -> shooter.goboom(.5), shooter);
    Command shoottheballs = new RunCommand(() -> shooter.goboom(m_controller.getThrottle()), shooter);
    Command shootergoslurp = new RunCommand(() -> shooter.goboom(-.2), shooter);
    Command intaketheballs = new RunCommand(() -> intakes.runboth(.3), intakes);
    Command backwardsintake = new RunCommand(() -> intakes.runintake(-.3), intakes);
    Command backwardsconveyor = new RunCommand(() -> intakes.runconveyor(-.75), intakes);
    Command ClimbYouFools = new RunCommand(() -> air.ClimberUp());
    Command ResetClimber = new RunCommand(() -> air.ClimberDown());
    Command intakealone = new RunCommand(() -> intakes.runintake(1));
    Command conveyoralone = new RunCommand(() -> intakes.runconveyor(1));
    Command firing = new RunCommand(() -> air.elevatorUp());
    Command Reloadingcoverme = new RunCommand(() -> air.elevatorDown());
    Command stopball = new RunCommand(() -> shooter.goboom(0), shooter);

    new JoystickButton(m_controller, 1)
        // .whenHeld(shoottheballs)
        // .whenReleased(() -> shooter.goboom(0))
        .whenHeld(firing)
        .whenReleased(Reloadingcoverme)
        ;

    new JoystickButton(m_controller, 2)
        .whenHeld(intaketheballs)
        .whenReleased(() -> intakes.runboth(0));

        new JoystickButton(m_controller, 3)
        .whenHeld(intakealone)
        .whenReleased(() -> intakes.runintake(0));

    new JoystickButton(m_controller, 4)
        .whenHeld(conveyoralone)
        .whenReleased(() -> intakes.runconveyor(0));

    new JoystickButton(m_controller, 5)
        .whenHeld(backwardsintake)
        .whenReleased(() -> intakes.runintake(0));

    new JoystickButton(m_controller, 6)
        .whenHeld(backwardsconveyor)
        .whenReleased(() -> intakes.runboth(0));

    new JoystickButton(m_controller, 7)
//        .whenHeld(ClimbYouFools)
        .whenPressed(shoottheballs)
;

    new JoystickButton(m_controller, 8)
        // .whenHeld(ResetClimber);
       .whenPressed(stopball); 

        // new JoystickButton(m_controller, 9)
        // .whenHeld(shootergoslurp)
        // .whenReleased(() -> shooter.goboom(0));

    new JoystickButton(m_controller, 11)
        // No requirements because we don't need to interrupt anything
        .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command shoottheballs = new RunCommand(() -> shooter.goboom(m_controller.getThrottle()), shooter);
    Command firing = new RunCommand(() -> air.elevatorUp());
    Command Reloadingcoverme = new RunCommand(() -> air.elevatorDown());
    //Command stopball = new RunCommand(() -> shooter.goboom(0), shooter);
    Command intaketheballs = new RunCommand(() -> intakes.runboth(.3), intakes);



    PathPlannerTrajectory ChosenPath = PathPlanner.loadPath(AutoPath.getSelected(), 2, 2);
    m_drivetrainSubsystem.m_field.getObject("traj").setTrajectory(ChosenPath);
    

// 3. Define PID controllers for tracking trajectory
PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
ProfiledPIDController thetaController = new ProfiledPIDController(
  AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

 // 4. Construct command to follow trajectory
 PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand (
  ChosenPath,
  m_drivetrainSubsystem::getPose,
  Constants.m_kinematics,
  xController,
  yController,
  thetaController,
  m_drivetrainSubsystem::setModuleStates,
  m_drivetrainSubsystem);

// 5. Add some init and wrap-up, and return everything
return new SequentialCommandGroup(
  new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(ChosenPath.getInitialPose())),
  new WaitCommand(.5),
  new ParallelDeadlineGroup(
    swerveControllerCommand,
    intaketheballs),
  new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
  new ParallelDeadlineGroup(
    new SequentialCommandGroup(
      new WaitCommand(3), 
      firing),
    shoottheballs
  ),
  Reloadingcoverme);
}


  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
