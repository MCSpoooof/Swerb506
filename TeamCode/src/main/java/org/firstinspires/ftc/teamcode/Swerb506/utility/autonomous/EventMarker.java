package org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous;

import org.firstinspires.ftc.teamcode.Swerb506.opmode.autonomous.Events;
import org.json.simple.JSONObject;

public class EventMarker {
    private final double relativePosition;
    private final Runnable action;
    private final String name;

    // Constructor
    public EventMarker(double relativePosition, Runnable action, String name) {
        this.relativePosition = relativePosition;
        this.action = action;
        this.name = name;
    }

    // Getter methods
    public double getRelativePosition() {
        return relativePosition;
    }

    public String getName() {
        return name;
    }

    // Execute the action if it's not null
    public void execute() {
        if (action != null) {
            System.out.println("Executing action for event: " + name);
            action.run();
        } else {
            System.out.println("No action assigned for event: " + name);
        }
    }

    // Factory method to create an EventMarker from JSON
    public static EventMarker fromJson(JSONObject json, Events eventRegistry) {
        try {
            double relativePosition = ((Number) json.get("relativePosition")).doubleValue();
            String name = (String) json.get("name");
            Runnable action = eventRegistry.getEventActions().get(name);

            if (action == null) {
                System.err.println("No action found for event: " + name);
            }

            return new EventMarker(relativePosition, action, name);
        } catch (Exception e) {
            System.err.println("Error creating EventMarker from JSON: " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }
}
