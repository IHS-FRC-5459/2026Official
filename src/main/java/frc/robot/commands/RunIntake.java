// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntake extends Command {
  private Intake s_intake;
  private LED s_led;
  private Pivot s_pivot;
  /** Creates a new Intake. */
  public RunIntake(LED s_led, Intake s_intake, Pivot s_pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_intake, s_pivot);
    this.s_intake = s_intake;
    this.s_led = s_led;
    this.s_pivot = s_pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_led.setIntaking(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_intake.setSpeed(0.2);
    s_pivot.setGoal(-16);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_intake.setSpeed(0);
    s_led.setIntaking(false);
    s_pivot.setGoal(90);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
