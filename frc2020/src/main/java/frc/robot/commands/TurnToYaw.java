/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RamseteDriveSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TurnToYaw extends CommandBase {
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("vision");
  boolean finished = false;
  // private final RamseteDriveSubsystem m_driveTrain;
  private final DriveTrain m_driveTrain;
  private final Joystick driverJoystick;
  /**
   * Creates a new TurnToYaw.
   */
  public TurnToYaw(DriveTrain driveTrain,Joystick driveController) {
    addRequirements(driveTrain);
    this.driverJoystick = driveController;
    this.m_driveTrain = driveTrain;
    SmartDashboard.putNumber("Kp", 0.02);
    SmartDashboard.putNumber("Ki", 0.0);
    SmartDashboard.putNumber("Kd", 0.0);
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
    
    double speed = driverJoystick.getRawAxis(Constants.AXIS_LEFT_STICK_Y);
    
    double tx = (double) table.getEntry("yaw").getNumber(0);
    double heading_error = -tx;
    double steering_adjust = 0.0;
    
    double Kp = (double) SmartDashboard.getEntry("Kp").getNumber(0.02);
    double Kd = (double) SmartDashboard.getEntry("Kd").getNumber(0.0);
    double Ki = (double) SmartDashboard.getEntry("Ki").getNumber(0.0);
    
    double pError = (double) SmartDashboard.getEntry("pError").getNumber(0.02);
    double dError = (double) SmartDashboard.getEntry("dError").getNumber(0.02);
    double iError = (double) SmartDashboard.getEntry("iError").getNumber(0.02);

    double min_command = (double) SmartDashboard.getEntry("min_command").getNumber(0.02);
    /*if (tx > 2.0)
    {
             steering_adjust =  Kp*heading_error - min_command;
    }
    else if (tx < -2.0)
    {
            steering_adjust = Kp*heading_error + min_command;
    }
    else {
      finished = true;
    }*/
    steering_adjust = PDI(iError, dError, heading_error, 0, dError, Kp, Kd, Ki);
    m_driveTrain.arcadeDrive(speed,steering_adjust, Constants.kDriveDeadband);
    System.out.println(tx);
  }
  public double PDI(double iError,double dError, double error,double previous_error ,double derivative, double P, double D, double I){
    iError += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    dError = (error - previous_error) / .02;
    return(P*error + I*iError + D*derivative);
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
