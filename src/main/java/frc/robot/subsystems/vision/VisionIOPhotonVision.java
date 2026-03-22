// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    System.out.println("Made camera: " + name);
  }

  @Override
  public boolean updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    var allResults = camera.getAllUnreadResults();
    if (allResults.isEmpty()) {
      return false; // Nothing to do
    }

    var result = allResults.get(allResults.size() - 1);

    // Update latest target observation
    if (result.hasTargets()) {
      inputs.latestTargetObservation =
          new TargetObservation(
              Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
              Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
    } else {
      inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    }

    Logger.recordOutput("Multitag present", result.multitagResult.isPresent());

    // Add pose observation
    if (result.multitagResult.isPresent()) { // Multitag result
      var multitagResult = result.multitagResult.get();

      // Calculate robot pose
      Transform3d fieldToCamera = multitagResult.estimatedPose.best;
      Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
      Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

      // Calculate average tag distance
      double totalTagDistance = 0.0;
      for (var target : result.targets) {
        totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
      }
      // Add tag IDs
      tagIds.addAll(multitagResult.fiducialIDsUsed);
      // Save pose observation
      inputs.poseObservations =
          new PoseObservation[] {
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size(), // Average tag distance
                PoseObservationType.PHOTONVISION) // Observation type
          };

    } else if (!result.targets.isEmpty()) { // Single tag result
      var target = result.targets.get(0);
      // Calculate robot pose
      var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
      if (tagPose.isPresent()) {
        Transform3d fieldToTarget =
            new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
        Transform3d cameraToTarget = target.bestCameraToTarget;
        Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Add tag ID
        tagIds.add((short) target.fiducialId);
        // Save pose observation
        inputs.poseObservations =
            new PoseObservation[] {
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  target.poseAmbiguity, // Ambiguity
                  1, // Tag count
                  cameraToTarget.getTranslation().getNorm(), // Average tag distance
                  PoseObservationType.PHOTONVISION) // Observation type
            };
      }
    }

    // Save tag IDs to inputs object
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }

    return true; // Updated vision, return true
  }
}
