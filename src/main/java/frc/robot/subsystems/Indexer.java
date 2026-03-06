// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private TalonFX motor;
  private double commandedVoltage;
  private final String loggingPrefix = "subsystems/indexer/";

  public Indexer() {
    motor = new TalonFX(Motors.indexId, canbus);
  }
  // Note: This subsystem purposely uses voltage rather than power because that is what it was
  // commanded by when we made the interpolation tables
  public void setVoltage(double volts) {
    // motor.setVoltage(volts);
    commandedVoltage = volts;
  }

  public double getVoltage() {
    return -motor.get();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(loggingPrefix + "commandedVoltage", commandedVoltage);
    Logger.recordOutput(loggingPrefix + "actualVoltage", getVoltage());
    // This method will be called once per scheduler run
  }
}
