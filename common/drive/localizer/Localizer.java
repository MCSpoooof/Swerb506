package org.firstinspires.ftc.teamcode.Swerb506.common.drive.localizer;

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;

public interface Localizer {

    void periodic();

    Pose getPos();

    void setPos(Pose pose);
}
