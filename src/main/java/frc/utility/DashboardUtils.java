package frc.utility;

import java.util.ArrayList;
import java.util.List;

public class DashboardUtils {
    private static final List<Dashboard> publishers = new ArrayList<>();

    /**
     * Call this function to register the subsystem's {@code elasticInit()} method
     * to be run at robot startup
     * @param subsystem set to {@code this} while in a subsystem class
     */
    public static void register(Dashboard subsystem) {
        publishers.add(subsystem);
    }
    
    /**
     * Call this once in {@code Robot.robotInit()}
     */
    public static void initAll() {
        for (Dashboard pub : publishers) {
            pub.elasticInit();
        }
    }
    
    public interface Dashboard{
        /**
         * Place all elastic configs in here to be run at robot startup
         */
        public void elasticInit();
    }
    
}
