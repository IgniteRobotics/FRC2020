/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoForward;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunKicker;
import frc.robot.commands.RunSorter;
import frc.robot.commands.Shooterspin;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinNKick;
import frc.robot.commands.SpinSpindexer;
import frc.robot.commands.SpinSpindexerToNext;
import frc.robot.commands.TurnToYaw;
import frc.robot.commands.Velocityshoot;
import frc.robot.commands.Wait;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.RamseteDriveSubsystem;
import frc.robot.subsystems.Sorter;
import frc.robot.subsystems.Spindexer;
import frc.robot.util.Dashboard;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.Shooterspin;
import frc.robot.commands.TargetPositioning;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveTrain m_driveTrain = new DriveTrain(Constants.kLeftMasterPort, Constants.kLeftFollowerPort, Constants.kLeftFollowerPort2, 
                                                  Constants.kRightMasterPort, Constants.kRightFollowerPort, Constants.kRightFollowerPort2);
  // private RamseteDriveSubsystem m_driveTrain = new RamseteDriveSubsystem();
  private Intake m_intake = new Intake();
  private Sorter m_sorter = new Sorter();
  private Spindexer m_spindexer = new Spindexer();
  private Shooter m_shooter = new Shooter();
  private Kicker m_kicker = new Kicker();
  private Joystick m_driveController = new Joystick(Constants.kDriveControllerPort);
  private Joystick m_manipController = new Joystick(Constants.kManipControllerPort);
  private ArcadeDrive teleDriveCommand = new ArcadeDrive(m_driveController, m_driveTrain);
  private AutoForward m_auto = new AutoForward(m_driveTrain, 1000);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // private NetworkTableEntry intakeSpeed = Dashboard.devTab.add("Intake Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
  // private NetworkTableEntry sorterSpeed = Dashboard.devTab.add("Sorter Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
  // private NetworkTableEntry kickerSpeed = Dashboard.devTab.add("Kicker Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
  // private NetworkTableEntry spindexerDirection = Dashboard.devTab.add("Spindexer CounterClockWise?", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
  // private NetworkTableEntry spindexerSpeed = Dashboard.devTab.add("Spindexer Speed", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemCommands();
    
    /* try {
      var straightTrajectory = loadTrajectory("Straight");
      Transform2d transform = new Pose2d(0, 0, Rotation2d.fromDegrees(0)).minus(straightTrajectory.getInitialPose());
      Trajectory newTrajectory = straightTrajectory.transformBy(transform);
      var straightPathCommand = m_driveTrain.createCommandForTrajectory(newTrajectory);
      autoChooser.setDefaultOption("Straight", straightPathCommand);
    }
    catch(IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: Straight", false);
    }
    SmartDashboard.putData("Auto Chooser", autoChooser); */
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driveController, Constants.AXIS_RIGHT_TRIGGER).whileHeld(teleDriveCommand::toggleSlowMode);
    new JoystickButton(m_driveController, Constants.BUTTON_A).whileHeld(new TargetPositioning(m_driveTrain, 138));
    new JoystickButton(m_driveController, Constants.BUTTON_B).whileHeld(new TargetPositioning(m_driveTrain, 222));

    new JoystickButton(m_manipController, Constants.BUTTON_LEFT_BUMPER).whileHeld(new RunIntake(0.6, m_intake));
    new JoystickButton(m_manipController, Constants.BUTTON_A).whileHeld(new ParallelCommandGroup(new RunSorter(0.5, m_sorter), new RunKicker(-0.1, m_kicker)));
    new JoystickButton(m_manipController, Constants.BUTTON_X).whileHeld(new SpinSpindexer(true, 0.15, m_spindexer));
    new JoystickButton(m_manipController, Constants.BUTTON_Y).whenPressed(m_kicker::toggleKicker);
    //new JoystickButton(m_manipController, Constants.BUTTON_LEFT_STICK).whileHeld(new ParallelCommandGroup(new Shooterspin(m_shooter), new RunKicker(0.7, m_kicker), new SpinIntake(0.3, m_intake))); //17 ft
    
    new JoystickButton(m_manipController, Constants.BUTTON_LEFT_STICK).whileHeld(new ParallelCommandGroup(new Velocityshoot(5700, m_shooter), new RunKicker(0.7, m_kicker), new SpinIntake(0.3, m_intake))); //17 ft
    new JoystickButton(m_manipController, Constants.BUTTON_START).whileHeld(new RunKicker(0.5, m_kicker));
    new JoystickButton(m_manipController, Constants.BUTTON_BACK).whenPressed(new SpinSpindexerToNext(m_spindexer));
    // new JoystickButton(m_manipController, Constants.BUTTON_RIGHT_BUMPER).whileHeld(new ParallelCommandGroup(new ParallelCommandGroup(new RunKicker(0.7, m_kicker), new Velocityshoot(5500, m_shooter), new SpinIntake(0.3, m_intake)), 
    //                                                                                 new SequentialCommandGroup(new Wait(1000).withTimeout(5), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), 
    //                                                                                 new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), 
    //                                                                                 new Wait(100).withTimeout(0.1), new SpinSpindexerToNext(m_spindexer), new Wait(1000).withTimeout(1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), 
    //                                                                                 new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), 
    //                                                                                 new Wait(100).withTimeout(0.1), new SpinSpindexerToNext(m_spindexer), new Wait(1000).withTimeout(1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), 
    //                                                                                 new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), 
    //                                                                                 new Wait(100).withTimeout(0.1), new SpinSpindexerToNext(m_spindexer))));
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
    // return autoChooser.getSelected();
    // return m_auto;
    return new ParallelCommandGroup(new ParallelCommandGroup(new RunKicker(0.7, m_kicker), new Velocityshoot(5500, m_shooter), new SpinIntake(0.3, m_intake)), 
    new SequentialCommandGroup(new AutoForward(m_driveTrain, 500).withTimeout(1), new Wait(1000).withTimeout(5), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), 
    new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), 
    new Wait(100).withTimeout(0.1), new SpinSpindexerToNext(m_spindexer), new Wait(1000).withTimeout(1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), 
    new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), 
    new Wait(100).withTimeout(0.1), new SpinSpindexerToNext(m_spindexer), new Wait(1000).withTimeout(1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), 
    new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), new Wait(100).withTimeout(0.1), new InstantCommand(m_kicker::toggleKicker, m_kicker), 
    new Wait(100).withTimeout(0.1), new SpinSpindexerToNext(m_spindexer)));
  }

  protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
    return TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", "output", trajectoryName + ".wpilib.json")));
  }

  /*public void resetOdometry() {
    new InstantCommand(m_driveTrain::resetOdometry, m_driveTrain).schedule();
  }*/
}
