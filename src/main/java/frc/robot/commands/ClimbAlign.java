// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import static frc.robot.Constants.Sensors.Distance.*;
import static frc.robot.commands.DriveCommands.setIsFirstCall;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbAlign extends Command {
  Drive s_drive;
  Climb s_climb;
  private final double xFFKs = 0.2;
  private final double omegaFFKs = 0.2;
  private final double omegaPIDI = 0;
  PIDController yPID = new PIDController(0.7, 0.03, 0.2);
  PIDController xPID = new PIDController(1.5, 0.02, 0.03);
  PIDController omegaPID = new PIDController(2.5, omegaPIDI, 0.02);
  SimpleMotorFeedforward xFF = new SimpleMotorFeedforward(xFFKs, 0, 0);
  SimpleMotorFeedforward omegaFF = new SimpleMotorFeedforward(omegaFFKs, 0, 0);
  SimpleMotorFeedforward yFF = new SimpleMotorFeedforward(0.23, 0, 0);
  private final double stoppingDist = -0.05;
  private final String loggingPrefix = "commands/climb/";
  /** Creates a new Climb. */
  // Climbs the right side of the climb structure(from the perspective of the alliance station)
  public ClimbAlign(Drive s_drive, Climb s_climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(s_drive);
    this.s_drive = s_drive;
    this.s_climb = s_climb;
  }

  // Called when the command is initially scheduled.
  // private boolean doneAligningToStart = false;
  int time = 0;
  boolean isFirstTime = false;
  long startOfTransition = 0;
  ClimbParams climbParams;
  boolean xEnabled, yEnabled, omegaEnabled;

  @Override
  public void initialize() {
    time = 0;
    setIsFirstCall(true);
    isDone = false;
    xPID.reset();
    yPID.reset();
    omegaPID.reset();
    isFirstTime = true;
    xFF.setKs(xFFKs);
    omegaFF.setKs(omegaFFKs);
    omegaPID.setI(omegaPIDI);
    Pose2d currPose = s_drive.getPose();
    climbParams = new ClimbParams(currPose);
    Logger.recordOutput(loggingPrefix + "phase", "start");
    xEnabled = true;
    yEnabled = true;
    omegaEnabled = true;
    // xPID.setP(SmartDashboard.getNumber("xPID_P", 0.1));
    // xPID.setI(SmartDashboard.getNumber("xPID_I", 0));
    // xPID.setD(SmartDashboard.getNumber("xPID_D", 0));
    // xFF.setKs(SmartDashboard.getNumber("xFF_S", 0));
    // yPID.setP(SmartDashboard.getNumber("yPID_P", 0.1));
    // yPID.setI(SmartDashboard.getNumber("yPID_I", 0));
    // yPID.setD(SmartDashboard.getNumber("yPID_D", 0));
    // yFF.setKs(SmartDashboard.getNumber("yFF_S", 0));
    // omegaPID.setP(SmartDashboard.getNumber("omegaPID_P", 0.1));
    // omegaPID.setI(SmartDashboard.getNumber("omegaPID_I", 0));
    // omegaPID.setD(SmartDashboard.getNumber("omegaPID_D", 0));
    // omegaFF.setKs(SmartDashboard.getNumber("omegaFF_S", 0));
    // xEnabled = SmartDashboard.getBoolean("xEnabled", false);
    // yEnabled = SmartDashboard.getBoolean("yEnabled", false);
    // omegaEnabled = SmartDashboard.getBoolean("omegaEnabled", false);
    // Logger.recordOutput(loggingPrefix + "pidConsts/x/P", xPID.getP());
    // Logger.recordOutput(loggingPrefix + "pidConsts/x/I", xPID.getI());
    // Logger.recordOutput(loggingPrefix + "pidConsts/x/D", xPID.getD());
    // Logger.recordOutput(loggingPrefix + "pidConsts/x/ffS", xFF.getKs());
    // Logger.recordOutput(loggingPrefix + "pidConsts/y/P", yPID.getP());
    // Logger.recordOutput(loggingPrefix + "pidConsts/y/I", yPID.getI());
    // Logger.recordOutput(loggingPrefix + "pidConsts/y/D", yPID.getD());
    // Logger.recordOutput(loggingPrefix + "pidConsts/y/ffS", yFF.getKs());
    // Logger.recordOutput(loggingPrefix + "pidConsts/omega/P", omegaPID.getP());
    // Logger.recordOutput(loggingPrefix + "pidConsts/omega/I", omegaPID.getI());
    // Logger.recordOutput(loggingPrefix + "pidConsts/omega/D", omegaPID.getD());
    // Logger.recordOutput(loggingPrefix + "pidConsts/omega/ffS", omegaFF.getKs());
  }

  // Called every time the scheduler runs while the command is scheduled.
  private boolean isDone = false;
  // 0 = x & omega   1 = turn wheels to 90deg   2 = y
  // private boolean omegaPassed1, yPassed, xPassed = false;
  private SwerveModulePosition[] snapshotModulesY = new SwerveModulePosition[4];

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput(loggingPrefix + "phase", "end1");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
