/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {
  private final WPI_VictorSPX kickerMotor;
  private final Solenoid kickerSolenoid;

  private boolean isExtended;

  /**
   * Creates a new Kicker.
   */
  public Kicker() {
    kickerMotor = new WPI_VictorSPX(Constants.kKickerMotorPort);
    kickerMotor.setInverted(false);
    kickerMotor.setNeutralMode(NeutralMode.Brake);

    isExtended = false;

    kickerSolenoid = new Solenoid(Constants.kKickerSolenoidPort);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
