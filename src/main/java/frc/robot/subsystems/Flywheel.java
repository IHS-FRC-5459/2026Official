// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  TalonFX m_fx, m_fx2;
  private double goal;
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();
  private final String loggingPrefix = "subsystems/flywheel/";

  /** Creates a new Flywheel. */
  public Flywheel() {
    m_fx = new TalonFX(Motors.flywheelId2, canbus);
    m_fx2 = new TalonFX(Motors.flywheelId, canbus);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV =
        0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts
    // / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0.01; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    configs.Slot1.kI = 0.05; // No output for integrated error
    configs.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
        .withPeakReverseTorqueCurrent(Amps.of(-40));

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    // StatusCode status2 = StatusCode.StatusCodeNotInitialized;
    // for (int i = 0; i < 5; ++i) {
    //   status2 = m_fx2.getConfigurator().apply(configs);
    //   if (status2.isOK()) break;
    // }
    // if (!status2.isOK()) {
    //   System.out.println("Could not apply configs, error code: " + status.toString());
    // }
    m_fx2.setControl(new Follower(m_fx.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void setGoal(double goal) {
    this.goal = goal;
    m_fx.setControl(m_velocityVoltage.withVelocity(goal));
    // m_fx2.setControl(m_velocityVoltage.withVelocity(goal));
  }

  public double getGoal() {
    return this.goal;
  }

  public void stop() {
    m_fx.setControl(m_brake);
    // m_fx2.setControl(m_brake);
  }

  public boolean isAtSetpoint() {
    return Math.abs(Math.abs(m_fx.getVelocity().getValueAsDouble()) - getGoal()) < 2;
    // && Math.abs(Math.abs(m_fx2.getVelocity(true).getValueAsDouble()) - getGoal()) < 5;
  }

  private double operatorOffset = 0;

  public void changeOperatorOffset(double amt) {
    this.operatorOffset += amt;
  }

  public double getOperatorOffset() {
    return operatorOffset;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput(loggingPrefix + "flywheelObserved1", m_fx.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        loggingPrefix + "flywheelObserved2", m_fx2.getVelocity().getValueAsDouble());
    Logger.recordOutput(loggingPrefix + "commanded1", m_fx.getAppliedControl().toString());
    Logger.recordOutput(loggingPrefix + "commanded2", m_fx2.getAppliedControl().toString());

    Logger.recordOutput(loggingPrefix + "goal", goal);
    SmartDashboard.putNumber("operatorOffset", getOperatorOffset());
    Logger.recordOutput(loggingPrefix + "operatorOffset", getOperatorOffset());
  }
}
