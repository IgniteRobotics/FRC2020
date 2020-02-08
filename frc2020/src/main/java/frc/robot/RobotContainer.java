/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoForward;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RamseteDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  /* private DriveTrain m_driveTrain = new DriveTrain(Constants.kLeftMasterPort, Constants.kLeftFollowerPort, Constants.kLeftFollowerPort2, 
                                                  Constants.kRightMasterPort, Constants.kRightFollowerPort, Constants.kRightFollowerPort2);
  */
  private RamseteDriveSubsystem m_driveTrain = new RamseteDriveSubsystem();
  private Joystick m_driveController = new Joystick(Constants.kDriveControllerPort);
  private Joystick m_manipController = new Joystick(Constants.kManipControllerPort);
  private AutoForward m_auto = new AutoForward();
  private ArcadeDrive teleDriveCommand = new ArcadeDrive(m_driveController, m_driveTrain);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemCommands();

    /*m_driveTrain.setDefaultCommand(new RunCommand(() -> m_driveTrain
                                        .arcadeDrive(m_driveController.getRawAxis(Constants.AXIS_LEFT_STICK_Y), 
                                        m_driveController.getRawAxis(Constants.AXIS_RIGHT_STICK_X), Constants.kDriveDeadband), m_driveTrain));
    */
    try {
      var straightTrajectory = loadTrajectory("Straight");
      Transform2d transform = new Pose2d(0, 0, Rotation2d.fromDegrees(0)).minus(straightTrajectory.getInitialPose());
      Trajectory newTrajectory = straightTrajectory.transformBy(transform);
      var straightPathCommand = m_driveTrain.createCommandForTrajectory(newTrajectory);
      autoChooser.setDefaultOption("Straight", straightPathCommand);
    }
    catch(IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: Straight", false);
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);

    }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driveController, Constants.AXIS_RIGHT_TRIGGER).whenPressed(teleDriveCommand::toggleSlowMode);
    new JoystickButton(m_driveController, Constants.AXIS_RIGHT_TRIGGER).whenReleased(teleDriveCommand::toggleSlowMode);

  }
  private void configureSubsystemCommands() {
    m_driveTrain.setDefaultCommand(teleDriveCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_auto;
  }

  protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
    return TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", "output", trajectoryName + ".wpilib.json")));
  }

  public void resetOdometry() {
    new InstantCommand(m_driveTrain::resetOdometry, m_driveTrain).schedule();
  }
}
