package org.firstinspires.ftc.teamcode.Swerb506.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Swerb506.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Swerb506.core.RobotHardware;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.controllers.PPHolonomicDriveController;
import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.path.PathPlannerPath;
import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.path.PathPlannerTrajectory;
import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.util.PIDConstants;
import org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous.EventMarker;
import org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous.EventMarkerParser;
import org.firstinspires.ftc.teamcode.Swerb506.opmode.autonomous.Events;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

@Autonomous(name="Pathplanner Test")
public class Automus extends RobotHardware {

    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private PPHolonomicDriveController controller;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> speedsSupplier;
    private Consumer<ChassisSpeeds> output;

    private List<EventMarker> eventMarkers;
    private boolean hasRun = false;
    private boolean finishedDriving = false;
    private ElapsedTimer elapsedTimer = new ElapsedTimer();
    private Map<String, Runnable> eventActions;
    private List<Double> eventTriggerTimes = new ArrayList<>();
    private double lastEventTime = -1;

    private Events events = new Events(); // Initialize your Events class

    @Override
    public void init() {
        super.init();

        controller = new PPHolonomicDriveController(
                new PIDConstants(3.0, 0.0, 0.0),
                new PIDConstants(4.0, 0.0, 0.0),
                0.012,
                2.13,
                0.12
        );

        poseSupplier = swerveDrive::getPose;
        speedsSupplier = swerveDrive::getRobotVelocity;
        output = swerveDrive::drive;

        // Use the context to create EventMarkerParser
        EventMarkerParser parser = new EventMarkerParser(hardwareMap.appContext);
        eventMarkers = parser.parseEventMarkersFromJson("Auto.json"); // Adjust the path to your JSON file

        // Initialize event actions
        eventActions = events.getEventActions(); // Retrieve the actions from Events class

        // Initialize path and trajectory
        path = PathPlannerPath.fromPathFile("Auto");
        trajectory = new PathPlannerTrajectory(path, speedsSupplier.get());

        // Filter and store trigger times for target events
        for (EventMarker marker : eventMarkers) {
            if (eventActions.containsKey(marker.getName())) {
                eventTriggerTimes.add(marker.getRelativePosition());
            }
        }

        // Reset controller and timer
        controller.reset(poseSupplier.get(), speedsSupplier.get());
        elapsedTimer.reset();
    }

    @Override
    public void loop() {
        super.loop();
        swerveDrive.updateOdometry();

        if (finishedDriving) {
            return;
        }

        if (!hasRun) {
            elapsedTimer.reset();
            hasRun = true;
        }

        double currentTime = elapsedTimer.seconds();
        PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);

        // Check for and execute specific event markers
        for (Double triggerTime : eventTriggerTimes) {
            if (Math.abs(currentTime - triggerTime) < 0.1 && lastEventTime < triggerTime) {
                for (EventMarker marker : eventMarkers) {
                    if (Math.abs(marker.getRelativePosition() - triggerTime) < 0.1) {
                        Runnable action = eventActions.get(marker.getName());
                        if (action != null) {
                            action.run();
                        }
                    }
                }
                lastEventTime = triggerTime;
            }
        }

        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
        output.accept(targetSpeeds);

        if (currentTime > trajectory.getTotalTimeSeconds()) {
            finishedDriving = true;
            output.accept(new ChassisSpeeds(0, 0, 0));
        }
    }
}