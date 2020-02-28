/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RamseteDriveSubsystem;

public class ArcadeDrive extends CommandBase {
  
  // private final RamseteDriveSubsystem m_driveTrain;
  private final DriveTrain m_driveTrain;
  private final Joystick driverJoystick;

  private boolean isSlowMode = false;
  private boolean isReversed = false;

  /**
   * Creates a new ArcadeDrive.
   */
  public ArcadeDrive(Joystick driveController, /*RamseteDriveSubsystem driveTrain*/DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driverJoystick = driveController;
    this.m_driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_driveTrain.arcadeDrive(getSpeed(), getRotation(), true);
    m_driveTrain.arcadeDrive(getSpeed(), getRotation(), Constants.kDriveDeadband);
  }

  private double getSpeed() {
    double speed = driverJoystick.getRawAxis(Constants.AXIS_LEFT_STICK_Y);
    if(isSlowMode) {
      speed *= Constants.SLOW_MODE_SPEED_MODIFIER;
    }
    if(isReversed) {
      speed *= -1;
    }
    return speed;
  }

  private double getRotation() {
    double rotation = -driverJoystick.getRawAxis(Constants.AXIS_RIGHT_STICK_X);
    if(isSlowMode) {
      rotation *= Constants.SLOW_MODE_SPEED_MODIFIER;
    }
    if(isReversed) {
      rotation *= -1;
    }
    return rotation;
  }

  public void toggleSlowMode() {
    isSlowMode = !(isSlowMode);
  }

  public void toggleReverseMode() {
    isReversed = !(isReversed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
