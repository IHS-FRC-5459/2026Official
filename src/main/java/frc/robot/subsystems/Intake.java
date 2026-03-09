// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX motor;

  private double commandedSpeed;
  private final String loggingPrefix = "subsystems/intake/";

  public Intake() {
    motor = new TalonFX(Motors.intakeId, canbus);
  }

  public void setSpeed(double speed) {
    motor.set(-speed);
    commandedSpeed = -speed;
  }

  public double getSpeed() {
    return motor.get();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(loggingPrefix + "commandedSpeed", commandedSpeed);
    Logger.recordOutput(loggingPrefix + "speed", getSpeed());
    // setSpeed(0.1);
    // This method will be called once per scheduler run
  }
}
