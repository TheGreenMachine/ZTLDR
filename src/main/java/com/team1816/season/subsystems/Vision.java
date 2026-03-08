//package com.team1816.season.subsystems;
//
//import com.team1816.lib.subsystems.ITestableSubsystem;
//import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonPoseEstimator;
//
//import java.io.IOException;
//import java.util.ArrayList;
//import java.util.List;
//
//
//public class Vision extends SubsystemBase implements ITestableSubsystem {
//    private final List<PhotonCamera> cameraList = new ArrayList<>();
//    private final List<Transform3d> positionList = new ArrayList<>();
//    private final List<PhotonPoseEstimator> poseEstimatorList = new ArrayList<>();
//    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
//
//    public void setupCameras(String name, Transform3d pos){
//        var cam = new PhotonCamera(name);
//        var estimator = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.);
//    }
//
//    public Vision() throws IOException {
//        setupCameras("frontCam", new Transform3d());
//        setupCameras("backCam", new Transform3d());
//        setupCameras("rightCam", new Transform3d());
//        setupCameras("leftCam", new Transform3d());
//    }
//
//    public void periodic(){
//
//    }
//}
