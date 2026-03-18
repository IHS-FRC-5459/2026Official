// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.LED;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorHalfway extends Command {
  Climb s_climb;
  LED s_led;
  /** Creates a new ClimbDown. */
  public ElevatorHalfway(LED s_led, Climb s_climb) {
    addRequirements(s_climb);
    this.s_climb = s_climb;
    this.s_led = s_led;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  boolean elevatorManualControl = false;
  boolean isDone = false;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_led.setElevatorGoingDown(true);
    elevatorManualControl = SmartDashboard.getBoolean("elevatorManualControl", false);
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevatorManualControl) {
      s_climb.setVoltage(-6);
    } else {
      s_climb.setGoal(Climb.Setpoints.HALFWAY);
      isDone = true;
    }

    // s_climb.setVoltage(SmartDashboard.getNumber("climbVolts", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // s_climb.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
