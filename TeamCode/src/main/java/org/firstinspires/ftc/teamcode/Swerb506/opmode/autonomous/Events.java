package org.firstinspires.ftc.teamcode.Swerb506.opmode.autonomous;

import java.util.HashMap;
import java.util.Map;

public class Events {
    private final Map<String, Runnable> eventActions = new HashMap<>();

    public Events() {
        // Initialize actions here
        eventActions.put("event1", this::handleEvent1);
        eventActions.put("event2", this::handleEvent2);
        // Add more events as needed
    }

    public Map<String, Runnable> getEventActions() {
        return eventActions;
    }

    private void handleEvent1() {
        // Your code for Event 1
    }

    private void handleEvent2() {
        // Your code for Event 2
    }
}