// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LED;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BeltBack extends Command {
  private Belt s_belt;
  private LED s_led;
  private Indexer s_indexer;
  /** Creates a new BeltBack. */
  public BeltBack(LED s_led, Belt s_belt, Indexer s_indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_belt);
    this.s_belt = s_belt;
    this.s_led = s_led;
    this.s_indexer = s_indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  //Empty commit
  @Override
  public void execute() {
    s_belt.setSpeed(-0.6);
    s_indexer.setVoltage(-10);
    s_led.setBeltBack(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_belt.setSpeed(0);
    s_indexer.setVoltage(0);
    s_led.setBeltBack(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
