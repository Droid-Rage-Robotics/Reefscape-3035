package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.Alignment;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveDriveConstants;
import frc.utility.shuffleboard.ShuffleboardValue;

public class Vision extends SubsystemBase {
    public enum Location {
        // Limelight
        // Which Pdfsdfdfole
        // Pole Type
        // ID
        RIGHT_R_L4_17(2.14, -19.94), // Default -blue done
        RIGHT_R_L4_18(2.1, -19.8), // done
        RIGHT_R_L4_19(1.87, -20.24), // done 1.87, -20.24
        RIGHT_R_L4_20(1.2, -19.35), // done 1.2, -19.35
        RIGHT_R_L4_21(1.71, -18.83), // done
        RIGHT_R_L4_22(2.01, -19.9), // done

        RIGHT_R_L4_6(2.16, -20.57), // red//done
        RIGHT_R_L4_7(1.84, -19.13), // done
        RIGHT_R_L4_8(1.64, -22.18), // done
        RIGHT_R_L4_9(2.28, -19.06), // done
        RIGHT_R_L4_10(1.6, -16.61), // done
        RIGHT_R_L4_11(1.88, -18.38), // done

        RIGHT_R_LEFT(0, 0),

        // RIGHT_R_L3(RIGHT_R_L4),
        // RIGHT_R_L2(RIGHT_R_L4),

        // ALGAE_R(0,0),

        LEFT_L_L4_17(-0.19, 19.41), // Default -blue //done
        LEFT_L_L4_18(.05, 18.89), // done iffy
        LEFT_L_L4_19(-0.29, 22.12), // dne  -0.29, 22.12
        LEFT_L_L4_20(-0.14, 20.87), // DOne
        LEFT_L_L4_21(-0.25, 18.9), // done
        LEFT_L_L4_22(-0.1, 18.5), // done

        LEFT_L_L4_6(-0.38, 19.7), // red - done - BAD BAD
        LEFT_L_L4_7(-0.06, 19.38), // done
        LEFT_L_L4_8(-0.24, 17.81), // done
        LEFT_L_L4_9(-0.06, 17.67), // done
        LEFT_L_L4_10(-0.3, 19.4), // done
        LEFT_L_L4_11(-0.06, 20.36), // done

        LEFT_L_RIGHT(0,0),

        // LEFT_L_L3(LEFT_L_L4),
        // LEFT_L_L2(LEFT_L_L4),

        // ALGAE_L(0, 0)

        RIGHT_A(1,-3),
        LEFT_A(-.5,4.5),

        ;

        private double distance, angle;

        private Location(double distance, double angle) {
            this.distance = distance;
            this.angle = angle;
        }

        private Location(Location location) {
            this.distance = location.distance;
            this.angle = location.angle;
        }

        public double getDistance() {
            return distance;
        }

        public double getAngle() {
            return angle;
        }
    }
    public static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();

    protected final ShuffleboardValue<Double> tARWriter = ShuffleboardValue
            .create(0.0, "R/tAR", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> tXRWriter = ShuffleboardValue
            .create(0.0, "R/tXR-Rot", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> tYRWriter = ShuffleboardValue
            .create(0.0, "R/tYR-Range", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Boolean> tVRWriter = ShuffleboardValue
            .create(false, "R/tVR", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> iDRWriter = ShuffleboardValue
            .create(0., "R/iDWriter", Vision.class.getSimpleName()).build();

    protected final ShuffleboardValue<Double> tALWriter = ShuffleboardValue
            .create(0.0, "L/tAL", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> tXLWriter = ShuffleboardValue
            .create(0.0, "L/tXL-Rot", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> tYLWriter = ShuffleboardValue
            .create(0.0, "L/tYL-Range", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Boolean> tVLWriter = ShuffleboardValue
            .create(false, "L/tVL", Vision.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> iDLWriter = ShuffleboardValue
            .create(0., "L/iDWriter", Vision.class.getSimpleName()).build();

    protected final ShuffleboardValue<String> poseWriter = ShuffleboardValue
            .create("0,0,0", "PoseWriter", Vision.class.getSimpleName()).build();
    public int targetIds[];
    public PIDController rotController = new PIDController(.095, 0, 0);// .1
    public PIDController xController = new PIDController(.11, 0, 0);// .1
    private int bluePipeline = 0, redPipeline = 1, leftPipeline =2, leftFrontPipeline=3,
    rightPipeline =4, rightFrontPipeline=5;
    boolean doRejectUpdate = false;
    // private SwerveDrive drive;
    private Field2d poseTest = new Field2d();
    
    public final ShuffleboardValue<Boolean> isAlignWriter = ShuffleboardValue
            .create(false, "IsAlign", Vision.class.getSimpleName()).build();
    // Set Up the team number - http://limelight.local:5801/

    // Initialize Limelight network tables
    public Vision() {
        LimelightHelpers.setCropWindow(DroidRageConstants.rightLimelight, -1, 1, -1, 1);
        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)
        LimelightHelpers.setCameraPose_RobotSpace(DroidRageConstants.rightLimelight,
                0., // Forward offset (meters)
                0.0, // Side offset (meters)
                0., // Height offset (meters)
                0.0, // Roll (degrees)
                10.0, // Pitch (degrees)
                0.0 // Yaw (degrees)
        );
        LimelightHelpers.setCropWindow(DroidRageConstants.leftLimelight, -1, 1, -1, 1);
        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)
        LimelightHelpers.setCameraPose_RobotSpace(DroidRageConstants.leftLimelight,
                0., // Forward offset (meters)
                0.0, // Side offset (meters)
                0., // Height offset (meters)
                0.0, // Roll (degrees)
                10.0, // Pitch (degrees)
                0.0 // Yaw (degrees)
        );

        LimelightHelpers.setStreamMode_Standard(DroidRageConstants.leftLimelight);
        LimelightHelpers.setStreamMode_Standard(DroidRageConstants.rightLimelight);

        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }

        // setUpVision();
        rotController.setTolerance(.7);//.5
        xController.setTolerance(.7);//.4

        // this.drive = drive;
        SmartDashboard.putData("VisionPose", poseTest);
    }

    @Override
    public void periodic() {
        tARWriter.set(LimelightHelpers.getTA(DroidRageConstants.rightLimelight));
        tXRWriter.set(LimelightHelpers.getTX(DroidRageConstants.rightLimelight));
        tYRWriter.set(LimelightHelpers.getTY(DroidRageConstants.rightLimelight));
        tVRWriter.set(LimelightHelpers.getTV(DroidRageConstants.rightLimelight));
        iDRWriter.set(LimelightHelpers.getFiducialID(DroidRageConstants.rightLimelight));

        tALWriter.set(LimelightHelpers.getTA(DroidRageConstants.leftLimelight));
        tXLWriter.set(LimelightHelpers.getTX(DroidRageConstants.leftLimelight));
        tYLWriter.set(LimelightHelpers.getTY(DroidRageConstants.leftLimelight));
        tVLWriter.set(LimelightHelpers.getTV(DroidRageConstants.leftLimelight));
        iDLWriter.set(LimelightHelpers.getFiducialID(DroidRageConstants.leftLimelight));

        if(SwerveDriveConstants.DriveOptions.IS_POSE_UPDATED.get()){
            // LimelightHelpers.PoseEstimate leftEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(DroidRageConstants.leftLimelight);
            // LimelightHelpers.PoseEstimate rightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(DroidRageConstants.rightLimelight);
            
            // if (leftEstimate.pose.getX() != 0 && leftEstimate.pose.getY() != 0) {
            //     poseTest.setRobotPose(leftEstimate.pose);
            //     poseWriter.set(leftEstimate.pose.toString());
            //     drive.resetOdometry(leftEstimate.pose);
            // } else if (rightEstimate.pose.getX() != 0 && rightEstimate.pose.getY() != 0) {
            //     poseTest.setRobotPose(rightEstimate.pose);
            //     poseWriter.set(rightEstimate.pose.toString());
            //     drive.resetOdometry(rightEstimate.pose);
            // }
        }
         
    }

    public void setUpVision() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            targetIds = new int[] { 6, 7, 8, 9, 10, 11 };
            LimelightHelpers.setPipelineIndex(DroidRageConstants.leftLimelight, redPipeline);
            LimelightHelpers.setPipelineIndex(DroidRageConstants.rightLimelight, redPipeline);
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            targetIds = new int[] { 17, 18, 19, 20, 21, 22 };

            LimelightHelpers.setPipelineIndex(DroidRageConstants.leftLimelight, bluePipeline);
            LimelightHelpers.setPipelineIndex(DroidRageConstants.rightLimelight, bluePipeline);
        }
    }

    public void setUpLeftVision(){
        targetIds = new int[] {6,19 };

        LimelightHelpers.setPipelineIndex(DroidRageConstants.leftLimelight, leftPipeline);
        LimelightHelpers.setPipelineIndex(DroidRageConstants.rightLimelight, leftPipeline);
    }
    public void setUpLeftFrontVision(){
        targetIds = new int[] {20,11};

        LimelightHelpers.setPipelineIndex(DroidRageConstants.leftLimelight, leftFrontPipeline);
        LimelightHelpers.setPipelineIndex(DroidRageConstants.rightLimelight, leftFrontPipeline);
    }
    public void setUpRightVision(){
        targetIds = new int[] {8,17};

        LimelightHelpers.setPipelineIndex(DroidRageConstants.leftLimelight, rightPipeline);
        LimelightHelpers.setPipelineIndex(DroidRageConstants.rightLimelight, rightPipeline);
    }
    public void setUpRightFrontVision(){
        targetIds = new int[] {9,22};

        LimelightHelpers.setPipelineIndex(DroidRageConstants.leftLimelight, rightFrontPipeline);
        LimelightHelpers.setPipelineIndex(DroidRageConstants.rightLimelight, rightFrontPipeline);
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    // tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    public double gettX(String name) {
        if (name == DroidRageConstants.leftLimelight) {
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
    // public boolean isID(String name){
    // for (int element : targetIds) {
    // if (element == LimelightHelpers.getFiducialID(name)) {
    // isIDWriter.set(true);
    // return true;
    // }
    // }
    // isIDWriter.set(false);
    // return false;
    // }

    public int getID(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return iDLWriter.get().intValue();
        } else {
            return iDRWriter.get().intValue();
        }
    }

    public Pose2d getPose(String name) {
        switch (DroidRageConstants.alignmentMode) {
            case RIGHT:
                return LimelightHelpers.getBotPose2d(DroidRageConstants.rightLimelight);
            case LEFT:
                return LimelightHelpers.getBotPose2d(DroidRageConstants.leftLimelight);
            default:
                return LimelightHelpers.getBotPose2d(DroidRageConstants.leftLimelight);

        }
        // return LimelightHelpers.getBotPose2d(name);
        // if (name == DroidRageConstants.leftLimelight) {
        // return LimelightHelpers.getBotPose2d(name);
        // } else {
        // return LimelightHelpers.getBotPose2d(name);
        // }
    }

    public Location getLeftLocation(String name, int look) {
        if(DroidRageConstants.alignmentMode==Alignment.MIDDLE){
            return Vision.Location.LEFT_A;
        } else if(DroidRageConstants.alignmentMode == Alignment.RIGHT){
            return Vision.Location.LEFT_L_RIGHT;
        }
        switch (look) {
            case 17:
                return Vision.Location.LEFT_L_L4_17;
            case 18:
                return Vision.Location.LEFT_L_L4_18;
            case 19:
                return Vision.Location.LEFT_L_L4_19;
            case 20:
                return Vision.Location.LEFT_L_L4_20;
            case 21:
                return Vision.Location.LEFT_L_L4_21;
            case 22:
                return Vision.Location.LEFT_L_L4_22;

            case 6:
                return Vision.Location.LEFT_L_L4_6;
            case 7:
                return Vision.Location.LEFT_L_L4_7;
            case 8:
                return Vision.Location.LEFT_L_L4_8;
            case 9:
                return Vision.Location.LEFT_L_L4_9;
            case 10:
                return Vision.Location.LEFT_L_L4_10;
            case 11:
                return Vision.Location.LEFT_L_L4_11;

            default:
                return Vision.Location.LEFT_L_L4_17;
        }

    }

    public Location getRightLocation(String name, int look) {
        if (DroidRageConstants.alignmentMode == Alignment.MIDDLE) {
            return Vision.Location.RIGHT_A;
        } else if (DroidRageConstants.alignmentMode == Alignment.LEFT) {
            return Vision.Location.RIGHT_R_LEFT;
        }
        switch (look) {
            case 17:
                return Vision.Location.RIGHT_R_L4_17;
            case 18:
                return Vision.Location.RIGHT_R_L4_18;
            case 19:
                return Vision.Location.RIGHT_R_L4_19;
            case 20:
                return Vision.Location.RIGHT_R_L4_20;
            case 21:
                return Vision.Location.RIGHT_R_L4_21;
            case 22:
                return Vision.Location.RIGHT_R_L4_22;

            case 6:
                return Vision.Location.RIGHT_R_L4_6;
            case 7:
                return Vision.Location.RIGHT_R_L4_7;
            case 8:
                return Vision.Location.RIGHT_R_L4_8;
            case 9:
                return Vision.Location.RIGHT_R_L4_9;
            case 10:
                return Vision.Location.RIGHT_R_L4_10;
            case 11:
                return Vision.Location.RIGHT_R_L4_11;
            default:
                return Vision.Location.RIGHT_R_L4_17;
        }
    }

    public double aim() {
        double targetingAngularVelocity = 0;
        switch (DroidRageConstants.alignmentMode) {
            case LEFT:
                targetingAngularVelocity = rotController.calculate(
                        gettX(DroidRageConstants.leftLimelight),
                        getLeftLocation(DroidRageConstants.leftLimelight).getAngle());
                break;
            case RIGHT:
                targetingAngularVelocity = rotController.calculate(
                        gettX(DroidRageConstants.rightLimelight),
                        getRightLocation(DroidRageConstants.rightLimelight).getAngle());
                break;
            case MIDDLE:
                if (gettV(DroidRageConstants.leftLimelight)) {
                    targetingAngularVelocity = rotController.calculate(
                        gettX(DroidRageConstants.leftLimelight),
                        getLeftLocation(DroidRageConstants.leftLimelight).getAngle());
                } else if (gettV(DroidRageConstants.rightLimelight)) {
                    targetingAngularVelocity = rotController.calculate(
                        gettX(DroidRageConstants.rightLimelight),
                        getRightLocation(DroidRageConstants.rightLimelight).getAngle());
                }
        }
        return targetingAngularVelocity;
    }

    public double range() {
        double targetingForwardSpeed = 0;
        switch (DroidRageConstants.alignmentMode) {
            case LEFT:
                targetingForwardSpeed = xController.calculate(
                        gettY(DroidRageConstants.leftLimelight),
                        getLeftLocation(DroidRageConstants.leftLimelight).getDistance());
                break;
            case RIGHT:
                targetingForwardSpeed = xController.calculate(
                        gettY(DroidRageConstants.rightLimelight),
                        getRightLocation(DroidRageConstants.rightLimelight).getDistance());
                break;
            case MIDDLE:
                if (gettV(DroidRageConstants.leftLimelight)) {
					targetingForwardSpeed = xController.calculate(
                        gettY(DroidRageConstants.leftLimelight),
                        getLeftLocation(DroidRageConstants.leftLimelight).getDistance());
				} else if (gettV(DroidRageConstants.rightLimelight)) {
                    targetingForwardSpeed = xController.calculate(
                        gettY(DroidRageConstants.rightLimelight),
                        getRightLocation(DroidRageConstants.rightLimelight).getDistance());
                }
                
                break;
        }
        return targetingForwardSpeed;
    }

    // public double aimAuto(int look) {
    //     double targetingAngularVelocity = 0;
    //     switch (DroidRageConstants.alignmentMode) {
    //         case LEFT:
    //             targetingAngularVelocity = rotController.calculate(
    //                     gettX(DroidRageConstants.leftLimelight),
    //                     getLeftLocation(DroidRageConstants.leftLimelight, look).getAngle());
    //             break;
    //         case RIGHT:
    //             targetingAngularVelocity = rotController.calculate(
    //                     gettX(DroidRageConstants.rightLimelight),
    //                     getRightLocation(DroidRageConstants.rightLimelight, look).getAngle());
    //             break;
    //     }
    //     return targetingAngularVelocity;
    // }

    // public double rangeAuto(int look) {
    //     double targetingForwardSpeed = 0;
    //     switch (DroidRageConstants.alignmentMode) {
    //         case LEFT:
    //             targetingForwardSpeed = xController.calculate(
    //                     gettY(DroidRageConstants.leftLimelight),
    //                     getLeftLocation(DroidRageConstants.leftLimelight, look).getDistance());
    //             break;
    //         case RIGHT:
    //             targetingForwardSpeed = xController.calculate(
    //                     gettY(DroidRageConstants.rightLimelight),
    //                     getRightLocation(DroidRageConstants.rightLimelight, look).getDistance());
    //             break;
    //     }
    //     return targetingForwardSpeed;
    // }

    public Location getRightLocation(String name) {
        return getRightLocation(name, getID(name));
    }
    public Location getLeftLocation(String name) {//Teleop
        return getLeftLocation(name, getID(name));
    }
}
