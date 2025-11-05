package frc.robot.subsystems.vision;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.Alignment;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.LimelightEx;
import lombok.Getter;

public class Vision extends SubsystemBase implements Dashboard{
    public enum Location {
        // Naming convention is Limelight_Pole_Level_TagID

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

    public enum MountPose {
        R_FORWARD(0.2267388258),
        R_SIDE(0.30880893),
        R_UP(0.2228530468),
        R_ROLL(0),
        R_PITCH(20),
        R_YAW(45),

        L_FORWARD(0.2267388258),
        L_SIDE(-0.30880893),
        L_UP(0.2228530468),
        L_ROLL(0),
        L_PITCH(20),
        L_YAW(-45);

        private final double value;

        private MountPose(double value) {
            this.value=value;
        }

        public double getValue() {
            return value;
        }
    }
    
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public int targetIds[];
    public PIDController rotController = new PIDController(.095, 0, 0);// .1
    public PIDController xController = new PIDController(.11, 0, 0);// .1
    private int bluePipeline = 0, redPipeline = 1, leftPipeline =2, leftFrontPipeline=3,rightPipeline =4, rightFrontPipeline=5;
    public final AtomicBoolean isAlign = new AtomicBoolean(false);
    // Set Up the team number - http://limelight.local:5801/

    @Getter private final LimelightEx rightLimelight = LimelightEx.create(DroidRageConstants.rightLimelight)
        .withStreamMode_Standard()
        .withCropWindow(-1, 1, -1, 1);
    
    @Getter private final LimelightEx leftLimelight = LimelightEx.create(DroidRageConstants.leftLimelight)
        .withStreamMode_Standard()
        .withCropWindow(-1, 1, -1, 1);

    // Initialize Limelight network tables
    public Vision() {
        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)

        rightLimelight.setMountPose(
            MountPose.R_FORWARD.getValue(), // Forward offset (meters)
            MountPose.R_SIDE.getValue(), // Side offset (meters)
            MountPose.R_UP.getValue(), // Height offset (meters)
            MountPose.R_ROLL.getValue(), // Roll (degrees)
            MountPose.R_PITCH.getValue(), // Pitch (degrees)
            MountPose.R_YAW.getValue() // Yaw (degrees)
        );
        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)
        leftLimelight.setMountPose(
            MountPose.L_FORWARD.getValue(), // Forward offset (meters)
            MountPose.L_SIDE.getValue(), // Side offset (meters)
            MountPose.L_UP.getValue(), // Height offset (meters)
            MountPose.L_ROLL.getValue(), // Roll (degrees)
            MountPose.L_PITCH.getValue(), // Pitch (degrees)
            MountPose.L_YAW.getValue() // Yaw (degrees)
        );

        // for (int port = 5800; port <= 5809; port++) {
        //     PortForwarder.add(port, "limelight.local", port);
        // }

        // setUpVision();
        rotController.setTolerance(.7);//.5
        xController.setTolerance(.7);//.4

        DashboardUtils.register(this);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("Left Limelight", leftLimelight);
        SmartDashboard.putData("Right Limelight", rightLimelight);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    @Override
    public void periodic() {}

    public void setUpVision() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            targetIds = new int[] { 6, 7, 8, 9, 10, 11 };
            leftLimelight.setPipelineIndex(redPipeline);
            rightLimelight.setPipelineIndex(redPipeline);

        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            targetIds = new int[] { 17, 18, 19, 20, 21, 22 };
            leftLimelight.setPipelineIndex(bluePipeline);
            rightLimelight.setPipelineIndex(bluePipeline);
        }
    }

    public void setUpLeftVision(){
        targetIds = new int[] {6,19 };
        leftLimelight.setPipelineIndex(leftPipeline);
        rightLimelight.setPipelineIndex(leftPipeline);
    }
    public void setUpLeftFrontVision(){
        targetIds = new int[] {20,11};
        leftLimelight.setPipelineIndex(leftFrontPipeline);
        rightLimelight.setPipelineIndex(leftFrontPipeline);
    }
    public void setUpRightVision(){
        targetIds = new int[] {8,17};
        leftLimelight.setPipelineIndex(rightPipeline);
        rightLimelight.setPipelineIndex(rightPipeline);
    }
    public void setUpRightFrontVision(){
        targetIds = new int[] {9,22};
        leftLimelight.setPipelineIndex(rightFrontPipeline);
        rightLimelight.setPipelineIndex(rightFrontPipeline);
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    // tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    public double gettX(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return leftLimelight.getTX();
        } else {
            return rightLimelight.getTX();
        }
    }

    // ta Target Area (0% of image to 100% of image)
    public double gettA(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return leftLimelight.getTA();
        } else {
            return rightLimelight.getTA();
        }
    }

    // ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    public double gettY(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return leftLimelight.getTY();
        } else {
            return rightLimelight.getTY();
        }
    }

    // tv Whether the limelight has any valid targets (0 or 1)
    // isConnected
    // 0 is __ and 1 is __
    public boolean gettV(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return leftLimelight.getTV();
        } else {
            return rightLimelight.getTV();
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
            return (int) leftLimelight.getID();
        } else {
            return (int) rightLimelight.getID();
        }
    }

    public Pose2d getPose(String name) {
        switch (DroidRageConstants.alignmentMode) {
            case RIGHT:
                return rightLimelight.getBotPose2d();
            case LEFT:
                return leftLimelight.getBotPose2d();
            default:
                return leftLimelight.getBotPose2d();
        }
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
