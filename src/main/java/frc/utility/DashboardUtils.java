package frc.utility;

import java.util.ArrayList;
import java.util.List;

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

            if(Config.Match==MatchValue.PRACTICE) {
                pub.practiceWriters();
            }
        }
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
    }
}
