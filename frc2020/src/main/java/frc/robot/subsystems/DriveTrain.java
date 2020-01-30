/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower;
  private WPI_VictorSPX leftFollower2;
  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower;
  private WPI_VictorSPX rightFollower2;

  private double previousTimeStamp;
  private double currentTimeStamp;
  private double currentLeftRevs;
  private double currentRightRevs;

  private double previousLeftRevs = 0.0;
  private double previousRightRevs = 0.0;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain(int leftMasterID, int leftFollowerID, int leftFollower2ID, int rightMasterID, int rightFollowerID, int rightFollower2ID) {
    leftMaster = new WPI_TalonSRX(leftMasterID);
    leftFollower = new WPI_VictorSPX(leftFollowerID);
    rightMaster = new WPI_TalonSRX(rightMasterID);
    rightFollower = new WPI_VictorSPX(rightFollowerID);

    leftFollower2 = new WPI_VictorSPX(leftFollower2ID);
    rightFollower2 = new WPI_VictorSPX(rightFollower2ID);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftFollower2.setNeutralMode(NeutralMode.Brake);
    rightFollower2.setNeutralMode(NeutralMode.Brake);

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    leftFollower2.follow(leftMaster);
    rightFollower2.follow(rightMaster);

    leftMaster.setInverted(true);
    rightMaster.setInverted(false);

    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);

    leftFollower2.setInverted(InvertType.FollowMaster);
    rightFollower2.setInverted(InvertType.FollowMaster);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentTimeStamp = System.currentTimeMillis();
    currentLeftRevs = getLeftEncoderRevolutions();
    currentRightRevs = -getRightEncoderRevolutions();

    double deltaTimeMillis = currentTimeStamp - previousTimeStamp;

    double deltaLRevs = currentLeftRevs - previousLeftRevs;
    double deltaRRevs = currentRightRevs - previousRightRevs;

    double leftRevsPerMil = deltaLRevs / deltaTimeMillis;
    double rightRevsPerMil = deltaRRevs / deltaTimeMillis;

    double leftRPM = leftRevsPerMil * 60000;
    double rightRPM = rightRevsPerMil * 60000;

    SmartDashboard.putNumber("Left RPM", leftRPM);
    SmartDashboard.putNumber("Right RPM", rightRPM);

    previousTimeStamp = currentTimeStamp;
    previousLeftRevs = currentLeftRevs;
    previousRightRevs = currentRightRevs;
  }

  public void arcadeDrive(double throttle, double rotation, double deadband) {
    
    throttle = limit(throttle);
    throttle = Util.applyDeadband(throttle, deadband);

    rotation = limit(-rotation);
    rotation = Util.applyDeadband(rotation, deadband);

    throttle = Math.copySign(throttle * throttle, throttle);
    rotation = Math.copySign(rotation * rotation, rotation);

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(rotation)), throttle);

    if (throttle >= 0.0) {
      // First quadrant, else second quadrant
      if (rotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = throttle - rotation;
      } else {
        leftMotorOutput = throttle + rotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (rotation >= 0.0) {
        leftMotorOutput = throttle + rotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = throttle - rotation;
      }
    }

    // setOpenLoopLeft(limit(leftMotorOutput));
    // setOpenLoopRight(limit(rightMotorOutput));

    setOpenLoopLeft(0.5);
    setOpenLoopRight(0.5);
  }

  public void setOpenLoopLeft(double power) {
    leftMaster.set(ControlMode.PercentOutput, power);
  }

  public void setOpenLoopRight(double power) {
    rightMaster.set(ControlMode.PercentOutput, power);
  }

  public double getLeftEncoderRevolutions() {
    double revolutions = Util.getRevolutionsFromTicks(leftMaster.getSelectedSensorPosition());
    return revolutions;
  }

  public double getRightEncoderRevolutions() {
    double revolutions = Util.getRevolutionsFromTicks(rightMaster.getSelectedSensorPosition());
    return revolutions;
  }

  public void zeroEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  private double limit(double value) {
    if (value > 1.0) {
      return 1.0;
    }
    if (value < -1.0) {
      return -1.0;
    }
    return value;
  }
}
