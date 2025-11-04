package frc.utility;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DroidRageConstants;

public class DashboardUtils {
    /**
     * Determines whether writers placed in {@code practiceWriters()}
     * are enabled or disabled
     */
    public enum MatchValue {
        /**
         * Writers placed in {@code practiceWriters()} will be disabled
         * to prevent loop overruns at a competition.
         */
        COMPETITION,

        /**
         * Writers placed in {@code practiceWriters()} will be enabled
         * for access to testing data in Elastic.
         */
        PRACTICE
    }

    public static class Config {
        /**
         * Default is {@code MatchValue.PRACTICE}
         */
        public static MatchValue Match = MatchValue.PRACTICE;
    }

    public interface Dashboard{
        /**
         * Place all elastic configs in here to be run at robot startup
         */
        public void elasticInit();

        /**
         * Place all writers that are not neccesary during a match here 
         * to only be used during practice to prevent loop overruns
         */
        public void practiceWriters();

        
        /**
         * Place any logic for alerts here to be run periodically
         */
        public void alerts();
    }

    private static final List<Dashboard> publishers = new ArrayList<>();
    private static final Alert batteryAlert = new Alert("Battery Voltage", AlertType.kWarning);
    private static final Elastic.Notification notification = new Elastic.Notification();
    private static final PowerDistribution powerDistribution = new PowerDistribution();
    
    /**
     * Call this function to register the subsystem's {@code elasticInit()} method
     * to be run at robot startup
     * @param subsystem set to {@code this} while in a subsystem class
     */
    public static void register(Dashboard subsystem) {
        publishers.add(subsystem);
    }
    
    /**
     * Elastic configurations to run when the robot initializes. Call this once
     * in {@code Robot.robotInit()}
     */
    public static void onRobotInit() {
        for (Dashboard pub : publishers) {
            pub.elasticInit();

            if(Config.Match==MatchValue.PRACTICE) {
                pub.practiceWriters();
            }
        }

        if (DroidRageConstants.BatteryLow) {
            Elastic.sendNotification(notification
                .withLevel(Elastic.Notification.NotificationLevel.ERROR)
                .withTitle("Battery")
                .withDescription("Battery Low!")
                .withDisplaySeconds(10.0));
        }

        SmartDashboard.putData("Distribution", powerDistribution);

        // WebServer.start(1181, Filesystem.getDeployDirectory().getAbsolutePath());
        // PortForwarder.add(5800, "localhost", 1181);

    }

    /**
     * Elastic configurations to run while the robot is disabled. Call this once
     * in {@code Robot.disabledPeriodic()}
     */
    public static void onDisabledPeriodic() {
        if (DroidRageConstants.BatteryLow) {
            batteryAlert.set(true);
            batteryAlert.setText("Battery Voltage Low");
        } else {
            batteryAlert.set(false);
        }
    }
    
}