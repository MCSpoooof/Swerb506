package org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous;

public class EventMarker {
    private final double relativePosition;
    private final Runnable action;
    private final String name;

    public EventMarker(double relativePosition, Runnable action, String name) {
        this.relativePosition = relativePosition;
        this.action = action;
        this.name = name;
    }

    public double getRelativePosition() {
        return relativePosition;
    }

    public String getName() {
        return name;
    }

    public void execute() {
        if (action != null) {
            action.run();
        }
    }
}