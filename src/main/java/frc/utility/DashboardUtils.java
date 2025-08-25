package frc.utility;

import java.util.ArrayList;
import java.util.List;

public class DashboardUtils {
    private static final List<Dashboard> publishers = new ArrayList<>();

    // Called by each subsystem in its constructor
    public static void register(Dashboard subsystem) {
        publishers.add(subsystem);
    }

    // Call this once in Robot.robotInit()
    public static void initAll() {
        for (Dashboard pub : publishers) {
            pub.elasticInit();
        }
    }
    public interface Dashboard{
        public void elasticInit();
    }
    
}
