package org.firstinspires.ftc.teamcode.Swerb506.opmode.autonomous;

import static org.firstinspires.ftc.teamcode.Swerb506.core.RobotConfiguration.TEST_SERVO;

import org.firstinspires.ftc.teamcode.Swerb506.core.RobotHardware;

public class Events extends RobotHardware {

    public static void swerve1() {
        //TODO[code for Event 1]
        TEST_SERVO.getAsServo().setPosition(1);
    }

    public static void event2(){
        System.out.println("Handling event 2 (event2)");
        //TODO[code for Event 2]
    }
}