/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
private final WPI_TalonSRX spindexerMotor;
private final WPI_VictorSPX kickerMotor;
private final Solenoid kickerSolenoid;
private boolean isExtended;
  /**
   * Creates a new Spindexer.
   */
  public Spindexer() {
    kickerMotor = new WPI_VictorSPX(Constants.kKickerMotorPort);
    kickerMotor.setInverted(false);
    kickerMotor.setNeutralMode(NeutralMode.Brake);

    isExtended = false;

    kickerSolenoid = new Solenoid(Constants.kKickerSolenoidPort);
    spindexerMotor = new WPI_TalonSRX(Constants.kSpindexerMotorPort);
    spindexerMotor.setInverted(false);
    spindexerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    spindexerMotor.setNeutralMode(NeutralMode.Brake);
    spindexerMotor.setSensorPhase(true);
  }

  public void spinClockwise() {
    spindexerMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void spinstop(){
    spindexerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void spinCounterClockwise() {
    spindexerMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public double getEncoderPosition() {
    return spindexerMotor.getSelectedSensorPosition();
  }
  private void extendKicker() {
    isExtended = true;
    kickerSolenoid.set(true);
  }

  private void retractKicker() {
    isExtended = false;
    kickerSolenoid.set(false);
  }

  public void toggleKicker() {
    if(isExtended) {
      retractKicker();
    }
    else {
      extendKicker();
    }
  }

  public void spinWheel() {
    kickerMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void stopWheel() {
    kickerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void spindex(){
    extendKicker();
    spinWheel();
    spinClockwise();
  }

  public void stopex(){
    retractKicker();
    stopWheel();
    spinstop();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
