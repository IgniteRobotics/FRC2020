/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower;
  // private WPI_VictorSPX leftFollower2;
  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower;
  // private WPI_VictorSPX rightFollower2;
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain(int leftMasterID, int leftFollowerID, int leftFollower2ID, int rightMasterID, int rightFollowerID, int rightFollower2ID) {
    // leftMaster = new WPI_TalonSRX(leftMasterID);
    // leftFollower = new WPI_VictorSPX(leftFollowerID);
    // rightMaster = new WPI_TalonSRX(rightMasterID);
    // rightFollower = new WPI_VictorSPX(rightFollowerID);

    leftMaster = new WPI_TalonSRX(4);
    leftFollower = new WPI_VictorSPX(5);
    rightMaster = new WPI_TalonSRX(1);
    rightFollower = new WPI_VictorSPX(2);

    //leftFollower2 = new WPI_VictorSPX(leftFollower2ID);
    //rightFollower2 = new WPI_VictorSPX(rightFollower2ID);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    //leftFollower2.setNeutralMode(NeutralMode.Brake);
    //rightFollower2.setNeutralMode(NeutralMode.Brake);

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    //leftFollower2.follow(leftMaster);
    //rightFollower2.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);

    //leftFollower2.setInverted(InvertType.FollowMaster);
    //rightFollower2.setInverted(InvertType.FollowMaster);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void arcadeDrive(double throttle, double rotation, double deadband) {
    setOpenLoopLeft(throttle);
    setOpenLoopRight(rotation);
  }

  public void setOpenLoopLeft(double power) {
    leftMaster.set(ControlMode.PercentOutput, power);
  }

  public void setOpenLoopRight(double power) {
    rightMaster.set(ControlMode.PercentOutput, power);
  }
}
