/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower;
  private WPI_VictorSPX leftFollower2;
  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower;
  private WPI_VictorSPX rightFollower2;
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain(int leftMasterID, int leftFollowerID, int leftFollower2ID, int rightMasterID, int rightFollowerID, int rightFollower2ID) {
    leftMaster = new WPI_TalonSRX(leftMasterID);
    leftFollower = new WPI_VictorSPX(leftFollowerID);
    leftFollower2 = new WPI_VictorSPX(leftFollower2ID);
    rightMaster = new WPI_TalonSRX(rightMasterID);
    rightFollower = new WPI_VictorSPX(rightFollowerID);
    rightFollower2 = new WPI_VictorSPX(rightFollower2ID);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void arcadeDrive(double throttle, double rotation, double deadband) {
    
  }
}
