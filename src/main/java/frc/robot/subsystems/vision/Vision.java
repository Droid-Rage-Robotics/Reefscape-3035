package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;
import frc.utility.shuffleboard.ShuffleboardValue;

// Visit Limelight Web interface at http://10.30.35.11:5801
public class Vision extends SubsystemBase {
    public enum Location{
        RIGHT_R(0,0),
        LEFT_R(0,0),
        ALGAE_R(0,0),
        RIGHT_L(0, 0),
        LEFT_L(0, 0),
        ALGAE_L(0, 0)
        
        ;

        private double distance,angle;
        private Location(double distance, double angle){
            this.distance = distance;
            this.angle = angle;
        }
        
        public double getDistance() {
            return distance;
        }

        public double getAngle() {
            return angle;
        }
    }
    // public static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();

    protected final ShuffleboardValue<Double> tARWriter = ShuffleboardValue
        .create(0.0, "R/tAR", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> tXRWriter = ShuffleboardValue
        .create(0.0, "R/tXR", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> tYRWriter = ShuffleboardValue
        .create(0.0, "R/tYR", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Boolean> tVRWriter = ShuffleboardValue
        .create(false, "R/tVR", Vision.class.getSimpleName()).build();

    protected final ShuffleboardValue<Double> tALWriter = ShuffleboardValue
        .create(0.0, "L/tAL", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> tXLWriter = ShuffleboardValue
        .create(0.0, "L/tXL", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> tYLWriter = ShuffleboardValue
        .create(0.0, "L/tYL", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Boolean> tVLWriter = ShuffleboardValue
        .create(false, "L/tVL", Vision.class.getSimpleName()).build();
    public int targetIds[];
    public PIDController rotController =new PIDController(.06,0,0);
	public PIDController xController = new PIDController(.13, 0, 0);
    // HttpCamera rightCam = new HttpCamera("limelight-right", "http://10.30.35.12:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
    // HttpCamera leftCam = new HttpCamera("limelight-left", "http://10.30.35.11:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);

    // http://10.30.35.11:5800/
    // http://roborio-2928-FRC.local:5801 - Works
    // Set Up the team number - http://limelight.local:5801/


    // Initialize Limelight network tables
    public Vision() {
        LimelightHelpers.setPipelineIndex   (DroidRageConstants.rightLimelight, 0);
        LimelightHelpers.setCropWindow      (DroidRageConstants.rightLimelight, -1, 1, -1, 1);
        // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace(DroidRageConstants.rightLimelight, 
            0.,    // Forward offset (meters)
            0.0,    // Side offset (meters)
            0.,    // Height offset (meters)
            0.0,    // Roll (degrees)
            10.0,   // Pitch (degrees)
            0.0     // Yaw (degrees)
        );
        LimelightHelpers.setPipelineIndex   (DroidRageConstants.leftLimelight, 0);
        LimelightHelpers.setCropWindow      (DroidRageConstants.leftLimelight, -1, 1, -1, 1);
        // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace(DroidRageConstants.leftLimelight, 
            0.,    // Forward offset (meters)
            0.0,    // Side offset (meters)
            0.,    // Height offset (meters)
            0.0,    // Roll (degrees)
            10.0,   // Pitch (degrees)
            0.0     // Yaw (degrees)
        );
        
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }

        
        
        // CameraServer.addCamera(rightCam);
        // Shuffleboard.getTab("Misc").add(rightCam).withSize(3, 3);
        // CameraServer.addCamera(leftCam);
        // Shuffleboard.getTab("Misc").add(leftCam).withSize(3, 3);
        

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            targetIds = new int[] { 17 };
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            targetIds = new int[] { 22 };
        }
        rotController.setTolerance(.3);
        xController.setTolerance(.2);
        // visionAlert = new Alert("Limelight is not connected! Vision will be
        // hindered!", Alert.AlertType.WARNING);

    }

    @Override
    public void periodic() {
        tARWriter.set(LimelightHelpers.getTA(DroidRageConstants.rightLimelight));
        tXRWriter.set(LimelightHelpers.getTX(DroidRageConstants.rightLimelight));
        tYRWriter.set(LimelightHelpers.getTY(DroidRageConstants.rightLimelight));
        tVRWriter.set(LimelightHelpers.getTV(DroidRageConstants.rightLimelight));

        tALWriter.set(LimelightHelpers.getTA(DroidRageConstants.leftLimelight));
        tXLWriter.set(LimelightHelpers.getTX(DroidRageConstants.leftLimelight));
        tYLWriter.set(LimelightHelpers.getTY(DroidRageConstants.leftLimelight));
        tVLWriter.set(LimelightHelpers.getTV(DroidRageConstants.leftLimelight));
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    // tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    public double gettX(String name) {  
        if(name == DroidRageConstants.leftLimelight){
            return tXLWriter.get();
        } else {
            return tXRWriter.get();
        }
    }

    // ta Target Area (0% of image to 100% of image)
    public double gettA(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return tALWriter.get();
        } else {
            return tARWriter.get();
        }
    }

    // ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    public double gettY(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return tYLWriter.get();
        } else {
            return tYRWriter.get();
        }
    }

    // tv Whether the limelight has any valid targets (0 or 1)
    // isConnected
    // 0 is __ and 1 is __
    public boolean gettV(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return tVLWriter.get();
        } else {
            return tVRWriter.get();
        }
    }

    /** The name will be which piepline to use based on which alignment direction */
    public boolean isID(String name){
        for (int element : targetIds) {
            if (element == LimelightHelpers.getFiducialID(name)) {
                return true;
            }
        }
        return false;
    }

}
