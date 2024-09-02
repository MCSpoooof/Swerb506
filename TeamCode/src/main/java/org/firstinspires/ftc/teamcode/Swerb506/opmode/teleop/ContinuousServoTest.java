package org.firstinspires.ftc.teamcode.Swerb506.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Swerb506.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.ContinuousServo;

@TeleOp(name = "Continuous Servo Test", group = "Test")
public class ContinuousServoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Retrieve the continuous servo from the configuration
        ContinuousServo testServo = RobotConfiguration.ANGLE_FRONT_LEFT.getAsContinuousServo();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Test the continuous servo by setting power to -1, 0, and 1
            telemetry.addData("Testing Servo", "Turning Left");
            telemetry.update();
            testServo.setPower(-1.0); // Turn left (continuous servo should spin)
            sleep(2000);

            telemetry.addData("Testing Servo", "Stopping");
            telemetry.update();
            testServo.setPower(0.0); // Stop the servo
            sleep(2000);

            telemetry.addData("Testing Servo", "Turning Right");
            telemetry.update();
            testServo.setPower(1.0); // Turn right (continuous servo should spin)
            sleep(2000);

            telemetry.addData("Test Complete", "Check if servo kept spinning");
            telemetry.update();
        }
    }
}