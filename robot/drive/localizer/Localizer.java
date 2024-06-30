package org.firstinspires.ftc.teamcode.Swerb506.robot.drive.localizer;

import org.firstinspires.ftc.teamcode.Swerb506.robot.drive.geometry.Pose;

public interface Localizer {

    void periodic();

    Pose getPos();

    void setPos(Pose pose);
}
