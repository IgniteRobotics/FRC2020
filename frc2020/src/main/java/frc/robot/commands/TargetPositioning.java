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
import frc.robot.util.VisionUtils;


public class TargetPositioning extends CommandBase {
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("Vision");
  private static float KpTurn = -0.1f;
  private static float KpDistance = -0.1f;
  private static float min_command = 0.05f;
  // the range you want.
  private double targetDistance;
  //allowed margin of error
  private double marginOfErrorDist = 5.0;
  private double marginOfErrorTurn = 2.0;
  
  
  private final RamseteDriveSubsystem m_driveTrain;
  /**
   * Creates a new TargetRange.
   */
  public TargetPositioning(RamseteDriveSubsystem driveTrain, double targetDistance) {
    addRequirements(driveTrain);
    this.m_driveTrain = driveTrain;
    this.targetDistance = targetDistance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    float ty = (float) table.getEntry("ty").getNumber(0);
    double currentDistance = VisionUtils.getDistanceToTarget(ty);
    double distanceError = this.targetDistance - currentDistance;
    double drivingAdjust = KpDistance * distanceError;

    float tx = (float) table.getEntry("tx").getNumber(0);
    float headingError = -tx;
    double steeringAdjust = 0.0;
    if (tx > 1.0)
    {
             steeringAdjust =  KpTurn*headingError - min_command;
    }
    else if (tx < 1.0)
    {
            steeringAdjust = KpTurn*headingError + min_command;
    }


    m_driveTrain.arcadeDrive(drivingAdjust,steeringAdjust,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    float ty = (float) table.getEntry("ty").getNumber(0);
    float tx = (float) table.getEntry("tx").getNumber(0);
    boolean distanceOK =  Math.abs(VisionUtils.getDistanceToTarget(ty)) <= marginOfErrorDist;
    boolean yawOK = Math.abs(tx) <= marginOfErrorTurn;
    return(distanceOK && yawOK);
  }
}
