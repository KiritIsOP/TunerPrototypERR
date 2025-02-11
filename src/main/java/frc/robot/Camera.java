package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.nio.file.FileSystem;
import java.util.List;
import java.util.Optional;

public class Camera { ;
    private PhotonCamera camera = new PhotonCamera("3130Camera");
    private Transform3d cameraToRobot = new Transform3d(12, 4, 8, new Rotation3d(0,Math.toDegrees(-15),Math.toDegrees(5)));
    private final String fieldName = Filesystem.getDeployDirectory().getPath() + "/2025-ERRshop-field.json";
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    public Camera() {
        try{
            aprilTagFieldLayout = new AprilTagFieldLayout(fieldName);
            System.out.println(fieldName);
            Alert alert = new Alert(fieldName, Alert.AlertType.kInfo);
            alert.set(true);
        } catch(Exception e) {
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
            System.out.println("Fix ur Field ngl");
            System.out.println(fieldName);
        }
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, cameraToRobot);
    }

    public void getResult(CommandSwerveDrivetrain drivetrain) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            photonPoseEstimator.setReferencePose(drivetrain.getState().Pose);
            Optional<EstimatedRobotPose> optionalOdoState = photonPoseEstimator.update(result);
            if (optionalOdoState.isPresent()) {
                EstimatedRobotPose odoState = optionalOdoState.get();
                drivetrain.addVisionMeasurement(odoState.estimatedPose.toPose2d(), odoState.timestampSeconds);
            }
        }
    }
}
