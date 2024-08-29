package org.firstinspires.ftc.teamcode.Swerb506.opmode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import java.util.HashMap;
import java.util.Map;

public class Events {
    private final Map<String, Runnable> eventActions = new HashMap<>();

    // Constructor to initialize event actions
    public Events() {
        initializeActions();
    }

    private void initializeActions() {
        // Initialize actions here
        eventActions.put("swerve1", this::handleEvent1);
        eventActions.put("event2", this::handleEvent2);

        // Debug initialization
     //   telemetry.addLine("Initialized Events with " + eventActions.size() + " actions.");
    }

    // Method to get the event actions map
    public Map<String, Runnable> getEventActions() {
        return eventActions;
    }

    // Define the action for "swerve1" event
    private void handleEvent1() {
        //telemetry.addLine("Handling event 1 (swerve1)");
        // Your code for Event 1
    }

    // Define the action for "event2" event
    private void handleEvent2() {
        //telemetry.addLine("Handling event 2 (event2)");
        // Your code for Event 2
    }
}
