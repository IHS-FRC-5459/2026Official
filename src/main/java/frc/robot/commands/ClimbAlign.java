// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import static frc.robot.Constants.Sensors.Distance.*;
import static frc.robot.commands.DriveCommands.joystickDriveRelativeCustom;
import static frc.robot.commands.DriveCommands.setIsFirstCall;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DistanceCaching;
import frc.robot.subsystems.DistanceSide;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbAlign extends Command {
  Drive s_drive;
  Climb s_climb;
  DistanceSide sideDistCache;
  private final double xFFKs = 0.2;
  private final double omegaFFKs = 0.2;
  private final double omegaPIDI = 0;
  PIDController yPID = new PIDController(0.7, 0.03, 0.2);
  PIDController xPID = new PIDController(1.5, 0.02, 0.03);
  PIDController omegaPID = new PIDController(2.5, omegaPIDI, 0.02);
  SimpleMotorFeedforward xFF = new SimpleMotorFeedforward(xFFKs, 0, 0);
  SimpleMotorFeedforward omegaFF = new SimpleMotorFeedforward(omegaFFKs, 0, 0);
  SimpleMotorFeedforward yFF = new SimpleMotorFeedforward(0.23, 0, 0);
  private final double stoppingDist = 0.12;
  private final String loggingPrefix = "commands/climb/";
  /** Creates a new Climb. */
  // Climbs the right side of the climb structure(from the perspective of the alliance station)
  public ClimbAlign(Drive s_drive, Climb s_climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(s_drive);
    this.s_drive = s_drive;
    this.s_climb = s_climb;
    this.sideDistCache = s_climb.getDistanceSide();
  }

  // Called when the command is initially scheduled.
  // private boolean doneAligningToStart = false;
  int time = 0;
  boolean isFirstTime = false;
  long startOfTransition = 0;
  ClimbParams climbParams;

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  private boolean isDone = false;
  // 0 = x & omega   1 = turn wheels to 90deg   2 = y
  // private boolean omegaPassed1, yPassed, xPassed = false;
  private SwerveModulePosition[] snapshotModulesY = new SwerveModulePosition[4];

  @Override
  public void execute() {
    double passingX, passingY, passingOmega;
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
    // boolean xEnabled = SmartDashboard.getBoolean("xEnabled", false);
    // boolean yEnabled = SmartDashboard.getBoolean("yEnabled", false);
    // boolean omegaEnabled = SmartDashboard.getBoolean("omegaEnabled", false);
    // stoppingDist = SmartDashboard.getNumber("stoppingDist", stoppingDist);
    boolean xEnabled = true;
    boolean yEnabled = true;
    boolean omegaEnabled = true;
    DistanceCaching distCache = s_climb.getDistanceCacheBack();
    if (climbParams.getIsFront()) {
      distCache = s_climb.getDistanceCacheFront();
    }
    Logger.recordOutput(loggingPrefix + "goal", climbParams.getGoal());
    Logger.recordOutput(loggingPrefix + "leftfiltered", distCache.getLeftFiltered());
    // want to go to to make sure we are going in with good alignment
    // Blue alliance

    // Note, the 2ft buffer distance is now gone
    double directionMult = 1;
    Pose2d climbPose = climbParams.getGoal();
    Logger.recordOutput(loggingPrefix + "climbPose", climbPose);
    // X

    DoubleSupplier omegaSupplier;
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;

    if (distCache.bothValid()) {
      passingX = (distCache.getXDistance() - climbPose.getX()) * climbParams.getXMultiplier();
      if (Math.abs(passingX) <= 0.015) {
        Logger.recordOutput(loggingPrefix + "setXFFTo0", true);
        xFF.setKs(0);
      }
      double rangeDiff = distCache.getDifference();
      Logger.recordOutput(loggingPrefix + "hitCondition", false);
      if (Math.abs(rangeDiff) <= 0.005) {
        Logger.recordOutput(loggingPrefix + "hitCondition", true);
        Logger.recordOutput(loggingPrefix + "hasHitCondition", true);
        omegaFF.setKs(0);
      }
      passingOmega = rangeDiff * climbParams.getOmegaMultiplier();
    } else {
      if (distCache.rightMeasurementsValid()) {
        passingX = (distCache.getRightFiltered() - climbPose.getX()) * climbParams.getXMultiplier();
        if (Math.abs(passingX) <= 0.0381) { // 1.5in
          xFF.setKs(0);
        }
      } else if (distCache.leftMeasurementsValid()) {
        passingX = (distCache.getLeftFiltered() - climbPose.getX()) * climbParams.getXMultiplier();
        if (Math.abs(passingX) <= 0.0381) { // 1.5in
          xFF.setKs(0);
        }
      } else {
        passingX = 0;
        Logger.recordOutput(loggingPrefix + "bothvalid", 0);
      }
      passingOmega = 0;
    }

    double deltaY = sideDistCache.getDistanceFiltered() * -directionMult; // .4064=16in to m
    if (!sideDistCache.measurementsValid()) {
      // Use localization
      Pose2d currPose = s_drive.getPose();
      deltaY = Math.abs(climbPose.getY() - currPose.getY()) * -directionMult;
    }
    passingY = -deltaY * climbParams.getStep2YMultiplier();
    Logger.recordOutput(loggingPrefix + "yDone", false);
    if (Math.abs(deltaY) <= stoppingDist) {
      Logger.recordOutput(loggingPrefix + "yDone", true);
      isDone = true;
      passingY = 0;
    }
    System.out.println("Command running");

    double pidVoltsOmega = omegaPID.calculate(passingOmega, 0) * -1;
    double ffVoltsOmega = omegaFF.calculate(passingOmega, 0);
    Logger.recordOutput(loggingPrefix + "controllers/omega/pidVolts", pidVoltsOmega);
    Logger.recordOutput(loggingPrefix + "controllers/omega/ffVolts", ffVoltsOmega);
    omegaSupplier = () -> pidVoltsOmega + ffVoltsOmega;

    double pidVoltsX = xPID.calculate(passingX, 0);
    double ffVoltsX = xFF.calculate(passingX, 0) * -1;
    Logger.recordOutput(loggingPrefix + "controllers/x/pidVolts", pidVoltsX);
    Logger.recordOutput(loggingPrefix + "controllers/x/ffVolts", ffVoltsX);
    xSupplier = () -> pidVoltsX + ffVoltsX;

    double pidVoltsY = yPID.calculate(passingY, 0);
    double ffVoltsY = yFF.calculate(passingY, 0) * -1;
    Logger.recordOutput(loggingPrefix + "controllers/y/pidVolts", 0);
    Logger.recordOutput(loggingPrefix + "controllers/y/ffVolts", 0);
    ySupplier = () -> MathUtil.clamp(pidVoltsY + ffVoltsY, -0.5, 0.5);
    // if (!yEnabled) {
    //   ySupplier = () -> 0;
    // }
    // if (!xEnabled) {
    //   xSupplier = () -> 0;
    // }
    // if (!omegaEnabled) {
    //   omegaSupplier = () -> 0;
    // }

    Logger.recordOutput(loggingPrefix + "passing/xPassing", passingX);
    Logger.recordOutput(loggingPrefix + "passing/omegaPassing", passingOmega);
    Logger.recordOutput(loggingPrefix + "passing/yPassing", passingY);

    // omegaPassed = omegaPassed && time < 100; // bad practice, but its fine :)
    Logger.recordOutput(loggingPrefix + "xSupplier", xSupplier.getAsDouble());
    joystickDriveRelativeCustom(s_drive, xSupplier, ySupplier, omegaSupplier, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
