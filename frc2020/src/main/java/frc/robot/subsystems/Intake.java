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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final WPI_VictorSPX intakeMotor;
  private final DoubleSolenoid intakePistonSolenoid;
  private final DigitalInput intakeSensor;

  private boolean isExtended;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    intakeMotor = new WPI_VictorSPX(Constants.kIntakeMotorPort);
    intakeMotor.setInverted(false);
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    isExtended = false;

    intakePistonSolenoid = new DoubleSolenoid(Constants.kIntakeSolenoidPort, Constants.kIntakeSolenoidPort2);

    intakeSensor = new DigitalInput(Constants.kIntakeSensorPort);
  }

  private void extendIntake() {
    isExtended = true;
    intakePistonSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  private void retractIntake() {
    isExtended = false;
    intakePistonSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void toggleIntake() {
    if(isExtended) {
      retractIntake();
    }
    else {
      extendIntake();
    }
  }

  public void spinInwards() {
    intakeMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void stop() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void spinOutwards() {
    intakeMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public boolean getIntakeSensor() {
    return intakeSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
