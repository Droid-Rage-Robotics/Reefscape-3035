package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utility.shuffleboard.ShuffleboardValue;

// Visit Limelight Web interface at http://10.30.35.11:5801
public class Vision extends SubsystemBase {

    Pose3d visionMeasurement3d;

    Config config = new Config(1, 0,0,0,0); //The Values from LimelightHelper
    // AprilTagFieldLayout layout = new AprilTagFieldLayout(null);
    // public static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

    // AprilTagDetector;
    // AprilTagDetection detection= new AprilTagDetection("", 1, 0, 0, null, gettX(), gettA(), null)
    // AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(config);

    protected final ShuffleboardValue<Double> tAWriter = ShuffleboardValue.create
        (0.0, "Vision/tA", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> tXWriter = ShuffleboardValue.create
        (0.0, "Vision/tX", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> tYWriter = ShuffleboardValue.create
        (0.0, "Vision/tY", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Boolean> tVWriter = ShuffleboardValue.create
        (false, "Vision/tV", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<String> pose2dWriter = ShuffleboardValue.create
        ("none", "Vision/Pose2d", Vision.class.getSimpleName()).build();
    HttpCamera httpCamera = new
        HttpCamera("Limelight", "http://roborio-3035-FRC.local:5801");
    // http://roborio-2928-FRC.local:5801 - Works
    //Set Up the team number - http://limelight.local:5801/

    LimelightHelpers.LimelightResults llresult;
    // Initialize Limelight network tables
    public Vision() {
        // LimelightHelpers.setLEDMode_PipelineControl("");
        // LimelightHelpers.setLEDMode_ForceOff("");
        LimelightHelpers.setPipelineIndex("", 0);
        LimelightHelpers.setCropWindow("",-1,1,-1,1);
        CameraServer.addCamera(httpCamera);
        Shuffleboard.getTab("Misc").add(httpCamera).withSize(3, 3);

        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }

        // Optional<Alliance> ally = DriverStation.getAlliance();
        // if (ally.isPresent()) {
        //     if (ally.get() == Alliance.Red) {
        //         Constants.VisionConstants.SpeakerID = 4;
        //         Constants.VisionConstants.farSpeakerID = 1;
        //         Constants.VisionConstants.AmpID = 5;
        //         Constants.side = "red";
        //     }
        //     if (ally.get() == Alliance.Blue) {
        //         Constants.VisionConstants.SpeakerID = 7;
        //         Constants.VisionConstants.farSpeakerID = 2;
        //         Constants.VisionConstants.AmpID = 6;
        //         Constants.side = "blue";
        //     }
        // }

        // visionAlert = new Alert("Limelight is not connected! Vision will be hindered!", Alert.AlertType.WARNING);

    }

    @Override
    public void periodic() {
        tAWriter.set(LimelightHelpers.getTA(""));
        tXWriter.set(LimelightHelpers.getTX(""));
        tYWriter.set(LimelightHelpers.getTY(""));
        tVWriter.set(LimelightHelpers.getTV(""));
        pose2dWriter.set(getPose().toString());
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    //tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    public double gettX(){
        return tXWriter.get();
    }
    //ta Target Area (0% of image to 100% of image)
    public double gettA(){
        return tAWriter.get();
    }
    //ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    public double gettY(){
        return tYWriter.get();
    }
    //tv Whether the limelight has any valid targets (0 or 1)
    //isConnected
    //0 is __ and 1 is __
    public boolean gettV(){
        return tVWriter.get();
    }
    
    public Pose2d getPose(){
        // return LimelightHelpers.getBotPose2d("");
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("").pose;
    }

}
