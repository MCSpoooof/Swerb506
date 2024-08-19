package org.firstinspires.ftc.teamcode.Swerb506.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Swerb506.core.RobotHardware;
import org.firstinspires.ftc.teamcode.Swerb506.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Translation2d;

@Config
@TeleOp(name = "TelePOP")
public class TelePOP extends RobotHardware {

    public static boolean fieldRelative = true;
    public static boolean headingCorrection = false;
    public boolean slowMode = false;

    public double speed;
    private final Executive.StateMachine<TelePOP> stateMachine;

    public TelePOP() {
        stateMachine = new Executive.StateMachine<>(this);
    }

    @Override
    public void init() {
        super.init();
        stateMachine.init();
        // Initialize additional components if needed
    }

    @Override
    public void init_loop() {
        super.init_loop();
        stateMachine.update();
        // Additional initialization if needed
    }

    @Override
    public void start() {
        super.start();
        stateMachine.changeState(Executive.StateMachine.StateType.DRIVE, new Drive_Manual());

        swerveDrive.zeroGyro();
        swerveDrive.resetOdometry(new Pose2d());
    }

    @Override
    public void loop() {
        super.loop();
        stateMachine.update();
    }

    class Drive_Manual extends Executive.StateBase<TelePOP> {
        @Override
        public void update() {
            super.update();

            // Control logic for resetting gyro and odometry
            if (primary.YOnce()) {
                swerveDrive.zeroGyro();
                swerveDrive.resetOdometry(new Pose2d());
            }

            // Toggle field-relative mode
            if (primary.BOnce()) {
                fieldRelative = !fieldRelative;
            }

            // Toggle heading correction
            if (primary.XOnce()) {
                headingCorrection = !headingCorrection;
            }

            // Set fixed heading for the robot
            if (primary.rightStickButtonOnce()) {
                SwerveDrive.lastHeadingRadians = (3.0 * Math.PI) / 2.0;
                SwerveDrive.updatedHeading = true;
            }

            // Toggle slow mode
            if (primary.leftBumperOnce()) {
                slowMode = !slowMode;
            }

            // Adjust speed based on mode
            speed = slowMode ? 0.3 : 1.0; // Assuming 0.3 is the slow mode multiplier

            // Calculate velocities with cubic scaling for smoother control
            double xV = (Math.pow(-primary.left_stick_y, 3) * swerveControllerConfiguration.maxSpeed) / speed;
            double yV = (Math.pow(-primary.left_stick_x, 3) * swerveControllerConfiguration.maxSpeed) / speed;
            double thetaV = (Math.pow(-primary.right_stick_x, 3) * swerveControllerConfiguration.maxAngularVelocity) / speed;

            // Drive the robot
            swerveDrive.drive(new Translation2d(xV, yV), thetaV, fieldRelative, true, headingCorrection);
            swerveDrive.updateOdometry();

            // Telemetry updates
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Field Oriented", fieldRelative);
            telemetry.addData("Robot Heading", swerveDrive.getYaw().getDegrees());
            telemetry.addData("Driver Pose", swerveDrive.getPose());
        }
    }
}
