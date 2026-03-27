// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  TalonFX motor;
  // MAX = 2800

  public static final class Setpoints {
    public static final double MAX = 350; // 2800
    public static final double MIN = 0;
    public static final double SAFE_MAX = 348; // 2650
    public static final double SAFE_MIN = 10; // 200
    public static final double HALFWAY = 125;
    // public static final double CLIMB_HEIGHT = 1000;//Never used
  }
  // Encoder m_encoder;
  private double goal = Setpoints.SAFE_MIN;
  double kMaxVelocity = 1;
  double kS = 1;
  double kMaxAcceleration = 1;
  double kP = 1;
  double kI = 1;
  double kD = 1;
  double kG = 1;
  double kV = 1;
  double kDt = 1;
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);
  private final String loggingPrefix = "subsystems/climb/";
  private Encoder m_encoder;
  /** Creates a new Climb. */
  public Climb() {
    motor = new TalonFX(Motors.climbId, canbus);
    // m_encoder = motor.get//new Encoder(Ports.ElevatorEncoderPort1, Ports.ElevatorEncoderPort2);
    // m_encoder = new Encoder(Ports.ElevatorEncoderPort1, Ports.ElevatorEncoderPort2);
    // m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
    // resetEncoder();
  }

  public double getEncoderDistance() {
    double elevatorPosition = motor.getPosition(true).getValueAsDouble();
    Logger.recordOutput(loggingPrefix + "encoder", elevatorPosition);
    return elevatorPosition;
    // return -m_encoder.getDistance();
  }

  public void setGoal(double goal) {
    m_controller.setGoal(goal);
    this.goal = goal;
    Logger.recordOutput(loggingPrefix + "goal", goal);
  }

  public double getGoal() {
    return this.goal;
  }

  public void updateMotorOutput() {
    if (!SmartDashboard.getBoolean("elevatorManualControl", false)) {
      if (Math.abs(getEncoderDistance() - getGoal()) > 1.192857) {
        double volts = (getGoal() - getEncoderDistance()) / 13;
        int sign = 1;
        if (volts < 0) {
          sign = -1;
        }
        setVoltage(MathUtil.clamp(Math.abs(volts), 0.75, 12) * sign);
      } else {
        // Logger.recordOutput(loggingPrefix + "condition", 3);
        setVoltage(0);
      }
    } else {
      setVoltage(-2);
    }
  }

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
    Logger.recordOutput(loggingPrefix + "volts", volts);
  }

  public void resetEncoder() {
    // m_encoder.reset();
  }

  boolean hasStoppedElevator = false;

  @Override
  public void periodic() {
    // Logger.recordOutput(loggingPrefix + "elevatorController", false);
    // if (!SmartDashboard.getBoolean("elevatorManualControl", false)) {
    //   updateMotorOutput();
    //   Logger.recordOutput(loggingPrefix + "elevatorControlled", true);
    //   hasStoppedElevator = false;
    // } else {
    //   if (!hasStoppedElevator) {
    //     setVoltage(0);
    //     hasStoppedElevator = true;
    //   }
    // }
    updateMotorOutput();
    // motor.setVoltagce(-1);
    // setGoal(SmartDashboard.getNumber("elevatorGoal", 0));
    Logger.recordOutput(loggingPrefix + "goal", getGoal());
    Logger.recordOutput(loggingPrefix + "encoder", getEncoderDistance());
    // motor.setVoltage(-2);
  }
}
