package org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous;

import org.firstinspires.ftc.teamcode.Swerb506.utility.math.ElapsedTimer;

public class PauseAction {
    private final double pauseDuration;
    private ElapsedTimer timer;
    private boolean isPaused;

    public PauseAction(double duration) {
        this.pauseDuration = duration;
        this.timer = new ElapsedTimer();
        this.isPaused = false;
    }

    public void start() {
        timer.reset();
        isPaused = true;
    }

    public boolean isPaused(double currentTime) {
        return isPaused && timer.seconds() < pauseDuration;
    }

    public void stop() {
        isPaused = false;
    }

    public double getPauseDuration() {
        return pauseDuration;
    }
}