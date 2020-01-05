/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private PWMTalonSRX leftMaster = new PWMTalonSRX(Constants.kLeftMasterPort);
  private PWMVictorSPX leftFollower = new PWMVictorSPX(Constants.kLeftFollowerPort);
  private PWMTalonSRX rightMaster = new PWMTalonSRX(Constants.kRightMasterPort);
  private PWMVictorSPX rightFollower = new PWMVictorSPX(Constants.kRightFollowerPort);

  private SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMaster, leftFollower);
  private SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMaster, rightFollower);

  private DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void arcadeDrive(double fwd, double rot) {
    driveTrain.arcadeDrive(fwd, rot);
  }
}
