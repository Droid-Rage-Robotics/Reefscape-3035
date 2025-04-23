package frc.robot.commands.autos;

import java.io.IOException;
import java.util.HashMap;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.old.OldSwerveDrive;

public class PathPlannerPathFollow {
    private final CommandSwerveDrivetrain drive;
    private final String pathName;
    private double maxVelocity = 0.3;
    private double acceleration = 0.5;
    private HashMap<String, Command> eventMap = new HashMap<>();

    

    private PathPlannerPathFollow(CommandSwerveDrivetrain drive, String pathName, double maxVelocity, double acceleration, HashMap<String, Command> eventMap) {
        this.drive = drive;
        this.pathName = pathName;
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.eventMap = eventMap;
    }

    private PathPlannerPathFollow(CommandSwerveDrivetrain drive, String pathName) {
        this.drive = drive;
        this.pathName = pathName;
    }

    public static PathPlannerPathFollow create(CommandSwerveDrivetrain drive, String pathName) {
        return new PathPlannerPathFollow(drive, pathName);
    }

    public PathPlannerPathFollow setMaxVelocity(double maxVelocity) {
        return new PathPlannerPathFollow(drive, pathName, maxVelocity, acceleration, eventMap);
    }

    public PathPlannerPathFollow setAcceleration(double acceleration) {
        return new PathPlannerPathFollow(drive, pathName, maxVelocity, acceleration, eventMap);
    }

    public PathPlannerPathFollow addMarker(String name, Command toRun) {
        eventMap.put(name, toRun);
        return new PathPlannerPathFollow(drive, pathName, maxVelocity, acceleration, eventMap);
    }
    public PathPlannerPathFollow addMarker(String name, ParallelCommandGroup toRun) {
        eventMap.put(name, toRun);
        return new PathPlannerPathFollow(drive, pathName, maxVelocity, acceleration, eventMap);
    }
    public Command build() throws FileVersionException, IOException, ParseException{
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);

    }
    //Translation PID constants - PID constants to correct for translation error (used to create the X and Y PID controllers)
    //Rotation PID constants - PID constants to correct for rotation error (used to create the rotation controller)
    
}
