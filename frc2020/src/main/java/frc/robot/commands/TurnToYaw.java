/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RamseteDriveSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnToYaw extends CommandBase {
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("limelight");
  boolean finished = false;
  private final RamseteDriveSubsystem m_driveTrain;
  /**
   * Creates a new TurnToYaw.
   */
  public TurnToYaw(RamseteDriveSubsystem driveTrain) {
    addRequirements(driveTrain);
    this.m_driveTrain = driveTrain;
    SmartDashboard.putNumber("Kp", 0.02);
    SmartDashboard.putNumber("min_command", 0.05);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double tx = (double) table.getEntry("tx").getNumber(0);
    double heading_error = -tx;
    double steering_adjust = 0.0;
    
    double Kp = (double) SmartDashboard.getEntry("Kp").getNumber(0.02);
    double min_command = (double) SmartDashboard.getEntry("min_command").getNumber(0.02);
    if (tx > 2.0)
    {
             steering_adjust =  Kp*heading_error - min_command;
    }
    else if (tx < -2.0)
    {
            steering_adjust = Kp*heading_error + min_command;
    }
    else {
      finished = true;
    }
    m_driveTrain.arcadeDrive(0,steering_adjust, false);
    System.out.println(tx);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
