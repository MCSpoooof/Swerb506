package org.firstinspires.ftc.teamcode.Swerb506.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Swerb506.core.RobotConstants;
import org.firstinspires.ftc.teamcode.Swerb506.core.RobotHardware;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.controllers.PPHolonomicDriveController;
import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.path.PathPlannerPath;
import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.path.PathPlannerTrajectory;
import org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous.EventMarker;
import org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous.EventMarkerParser;
import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.util.PIDConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

@Autonomous(name = "Automus")
public class Automus extends RobotHardware {

    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private PPHolonomicDriveController controller;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> speedsSupplier;
    private Consumer<ChassisSpeeds> output;

    private List<EventMarker> eventMarkers = new ArrayList<>();
    private boolean hasRun = false;
    private boolean finishedDriving = false;
    private ElapsedTimer elapsedTimer = new ElapsedTimer();
    private List<Double> eventTriggerTimes = new ArrayList<>();
    private double lastEventTime = -1;

    private Events events = new Events(); // Initialize your Events class
    private boolean useManualEventMarkers = false; // Variable to control manual event markers

    @Override
    public void init() {
        super.init();

        System.out.println("Initializing Pathplanner Test");

        // Initialize drive controller
        controller = new PPHolonomicDriveController(
                new PIDConstants(3.0, 0.0, 0.0),
                new PIDConstants(4.0, 0.0, 0.0),
                0.012,
                RobotConstants.maxSpeed,
                0.12
        );

        poseSupplier = swerveDrive::getPose;
        speedsSupplier = swerveDrive::getRobotVelocity;
        output = swerveDrive::drive;

        if (useManualEventMarkers) {
            // Use manual event markers
            System.out.println("Using manual event markers.");
            eventMarkers.clear(); // Clear any loaded markers
            // Add manual event markers with actions
            eventMarkers.add(new EventMarker(0.15, events.getEventActions().get("swerve1"), "swerve1"));
            eventMarkers.add(new EventMarker(1.0, events.getEventActions().get("event2"), "event2"));
        } else {
            // Load event markers from path file
            try {
                EventMarkerParser parser = new EventMarkerParser();
                eventMarkers = parser.parseEventMarkersFromPath("Auto"); // Use the correct file name without extension
                if (eventMarkers.isEmpty()) {
                    System.out.println("No event markers found in path file.");
                } else {
                    System.out.println("Event markers loaded: " + eventMarkers.size());
                }
            } catch (Exception e) {
                System.out.println("Error loading event markers from path file: " + e.getMessage());
                e.printStackTrace();
                eventMarkers.clear(); // Ensure list is empty if loading fails
            }
        }

        // Filter and store trigger times for target events
        eventTriggerTimes.clear();
        for (EventMarker marker : eventMarkers) {
            if (events.getEventActions().containsKey(marker.getName())) {
                eventTriggerTimes.add(marker.getRelativePosition());
            }
        }

        try {
            path = PathPlannerPath.fromPathFile("Auto"); // Use the correct file name without extension
            trajectory = new PathPlannerTrajectory(path, speedsSupplier.get());
            System.out.println("Path and trajectory initialized.");
        } catch (Exception e) {
            System.out.println("Error initializing path or trajectory: " + e.getMessage());
            e.printStackTrace();
        }

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

        // Handle events
        for (Double triggerTime : eventTriggerTimes) {
            if (Math.abs(currentTime - triggerTime) < 0.1 && lastEventTime < triggerTime) {
                for (EventMarker marker : eventMarkers) {
                    if (Math.abs(marker.getRelativePosition() - triggerTime) < 0.1) {
                        marker.execute(); // Use the execute method of EventMarker
                    }
                }
                lastEventTime = triggerTime;
            }
        }

        // Update movement
        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
        output.accept(targetSpeeds);

        if (currentTime > trajectory.getTotalTimeSeconds()) {
            finishedDriving = true;
            output.accept(new ChassisSpeeds(0, 0, 0));
            System.out.println("Finished driving, stopping robot.");
        }
    }
}