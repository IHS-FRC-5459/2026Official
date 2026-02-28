// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import org.littletonrobotics.junction.Logger;

public class Belt extends SubsystemBase {
  private SparkMax motor;
  private double commandedSpeed = 0;
  private final String loggingPrefix = "subsystems/belt/";

  public Belt() {
    motor = new SparkMax(Motors.beltId, MotorType.kBrushless);
  }

  public void setSpeed(double power) {
    motor.set(-power);
    commandedSpeed = -power;
    Logger.recordOutput(loggingPrefix + "commandedSpeed", -power);
  }

  public double getSpeed() {
    return -motor.get();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(loggingPrefix + "commandedSpeed", commandedSpeed);
    Logger.recordOutput(loggingPrefix + "actualSpeed", getSpeed());
    // This method will be called once per scheduler run
  }
}
