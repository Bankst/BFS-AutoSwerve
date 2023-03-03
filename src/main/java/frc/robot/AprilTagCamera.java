package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class AprilTagCamera {
    public final PhotonCamera cam = new PhotonCamera("ov9281");
    // distance from robot to camera
    Transform3d robotToCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(6), 0, Units.inchesToMeters(34.75)), // camera placement on robot
            new Rotation3d(0, Units.degreesToRadians(0), 0));

    AprilTagFieldLayout aprilTagFieldLayout;
    ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    static PhotonPoseEstimator poseEstimator;

    public AprilTagCamera() {
        init();
        PhotonCamera.setVersionCheckEnabled(false);
    }

    public void updateField2d(Field2d field) {
        var poseList = new ArrayList<Pose2d>();
        for (int i = 1; i <= 8; i++) {
            poseList.add(aprilTagFieldLayout.getTagPose(i).get().toPose2d());
        }

        field.getObject("April Tags").setPoses(poseList);
    }

    private void init() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        camList.add(new Pair<PhotonCamera, Transform3d>(cam, robotToCam));
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, cam,
                robotToCam);
    }

    /**
     * 
     * @param prevEstimatedRobotPose
     * @return an EstimatedRobotPose which includes a Pose3d of the latest estimated
     *         pose (using the selected strategy) along with a double of the
     *         timestamp when the robot pose was estimated
     *         Use this to update drivetrain pose estimator
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        poseEstimator.setLastPose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }

    // unfiltered view of camera
    public void toggleDriverMode() {
        if (cam.getDriverMode()) {
            cam.setDriverMode(false);
        }

        else {
            cam.setDriverMode(true);
        }
    }

    public Optional<PhotonPipelineResult> getLatestResult() {
        var result = cam.getLatestResult();
        return result != null ? Optional.of(result) : Optional.empty();
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        var latestOpt = getLatestResult();
        if (latestOpt.isPresent()) {
            if (latestOpt.get().hasTargets()) {
                return Optional.of(latestOpt.get().getBestTarget());
            }
        }

        return Optional.empty();
    }

    public void updateReferencePose(Pose2d poseMeters) {
        poseEstimator.setReferencePose(poseMeters);
    }
}
