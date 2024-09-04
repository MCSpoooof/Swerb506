package org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous;

public class EventMarker {
    private final double relativePosition;
    private final Runnable action;
    private final String name;

    // constructor
    public EventMarker(double relativePosition, Runnable action, String name) {
        this.relativePosition = relativePosition;
        this.action = action;
        this.name = name;
    }

    // getter methods
    public double getRelativePosition() {
        return relativePosition;
    }
    public String getName() {
        return name;
    }
    public void execute() { // execute if action exists
        if (action != null) {
            System.out.println("Executing action for event: " + name);
            action.run();
        } else {
            System.out.println("No action assigned for event: " + name);
        }
    }

}