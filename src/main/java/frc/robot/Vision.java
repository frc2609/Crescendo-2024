package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AprilTag;

public class Vision {
  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  public static final Transform3d robotToCameraTransform
    = new Transform3d(-0.032, 0, 0.64, new Rotation3d(Math.toRadians(0), Math.toRadians(15), Math.toRadians(180)));

  public final PhotonCamera rearCamera = new PhotonCamera("rear");
  public final PhotonCameraSim rearCameraSim;
  public final VisionSystemSim visionSim;
  // these will not be in the correct order
  // if it don't work change coprocessor to rio
  public final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(AprilTag.fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rearCamera, robotToCameraTransform);

  public Vision() {
    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(AprilTag.fieldLayout);
 
      // properties taken from PV docs, ideally replace these with ones from a real camera
      var properties = new SimCameraProperties();
      properties.setCalibration(640, 480, Rotation2d.fromDegrees(100));
      // Approximate detection noise with average and standard deviation error in pixels.
      properties.setCalibError(0.25, 0.08);
      // Set the camera image capture framerate (Note: this is limited by robot loop rate).
      properties.setFPS(20);
      // The average and standard deviation in milliseconds of image data latency.
      properties.setAvgLatencyMs(35);
      properties.setLatencyStdDevMs(5);

      rearCameraSim = new PhotonCameraSim(rearCamera, properties);
      rearCameraSim.enableDrawWireframe(true);

      visionSim.addCamera(rearCameraSim, robotToCameraTransform);
    } else {
      visionSim = null;
      rearCameraSim = null;
    }
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public void estimatePose() {
    var estimate = poseEstimator.update();
    estimate.ifPresent(est -> {
      var pose = est.estimatedPose.toPose2d();
      var stdDevs = getEstimationStdDevs(pose);
      RobotContainer.drive.drive.addVisionMeasurement(pose, est.timestampSeconds, stdDevs);
      SmartDashboard.putNumber("PV Estimator Error", PhotonUtils.getDistanceToPose(RobotContainer.drive.drive.field.getObject("actual robot pose").getPose(), pose));
    });
    SmartDashboard.putNumber("YAGSL Pose Estimator Error", PhotonUtils.getDistanceToPose(RobotContainer.drive.drive.field.getObject("actual robot pose").getPose(), RobotContainer.drive.getPoseEfficiently()));
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = kSingleTagStdDevs;
    var targets = rearCamera.getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }
}
