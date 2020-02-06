/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RamseteDriveSubsystem extends SubsystemBase {

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.kLeftMasterPort);
  private final WPI_VictorSPX leftFollower = new WPI_VictorSPX(Constants.kLeftFollowerPort);
  private final WPI_VictorSPX leftFollower2 = new WPI_VictorSPX(Constants.kLeftFollowerPort2);

  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.kRightMasterPort);
  private final WPI_VictorSPX rightFollower = new WPI_VictorSPX(Constants.kRightFollowerPort);
  private final WPI_VictorSPX rightFollower2 = new WPI_VictorSPX(Constants.kRightFollowerPort2);

  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftMaster, leftFollower, leftFollower2);
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightMaster, rightFollower, rightFollower2);

  private final DifferentialDrive m_driveTrain = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final AHRS navX = new AHRS(SPI.Port.kMXP);

  private final DifferentialDriveOdometry m_odometry;
  
  public RamseteDriveSubsystem() {

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    resetEncoders();
    navX.zeroYaw();

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    talonConfig.slot0.kP = Constants.kPDriveVel;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;

    leftMaster.configAllSettings(talonConfig);
    rightMaster.configAllSettings(talonConfig);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);
    leftFollower2.follow(leftMaster);
    rightFollower2.follow(rightMaster);

    leftMaster.setInverted(true);
    leftFollower.setInverted(InvertType.FollowMaster);
    leftFollower2.setInverted(InvertType.FollowMaster);

    rightMaster.setInverted(false);
    rightFollower.setInverted(InvertType.FollowMaster);
    rightFollower2.setInverted(InvertType.FollowMaster);

    leftMaster.setSensorPhase(true);
    rightMaster.setSensorPhase(true);

    m_driveTrain.setRightSideInverted(false);

    setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorVelocity(), rightMaster.getSelectedSensorVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void arcadeDrive(double fwd, double rot) {
    m_driveTrain.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_driveTrain.feed();
  }

  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (leftMaster.getSelectedSensorPosition(0) + rightMaster.getSelectedSensorPosition(0)) / 2.0;
  }

  public double getLeftEncoderPosition() {
    return leftMaster.getSelectedSensorPosition(0);
  }

  public double getRightEncoderPosition() {
    return rightMaster.getSelectedSensorPosition(0);
  }

  public void setMaxOutput(double maxOutput) {
    m_driveTrain.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    navX.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(navX.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setNeutralMode(NeutralMode neutralMode) {
    leftMaster.setNeutralMode(neutralMode);
    rightMaster.setNeutralMode(neutralMode);
    leftFollower.setNeutralMode(neutralMode);
    rightFollower.setNeutralMode(neutralMode);
    leftFollower2.setNeutralMode(neutralMode);
    rightFollower2.setNeutralMode(neutralMode);
  }
}
