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

    // Acceleration parameters
    private static final double MAX_ACCELERATION = 2.0; // m/s^2
    private double xVelocity = 0.0;
    private double yVelocity = 0.0;
    private double thetaVelocity = 0.0;

    private long lastUpdateTime = 0; // Last update time in nanoseconds

    public TelePOP() {
        stateMachine = new Executive.StateMachine<>(this);
    }

    @Override
    public void init() {
        super.init();
        stateMachine.init();
        lastUpdateTime = System.nanoTime(); // Initialize the last update time
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

            // Calculate the time difference
            double deltaTime = getDeltaTime(); // Time since last update in seconds

            // Desired velocities based on joystick input
            double targetXVelocity = (Math.pow(-primary.left_stick_y, 3) * swerveControllerConfiguration.maxSpeed) / speed;
            double targetYVelocity = (Math.pow(-primary.left_stick_x, 3) * swerveControllerConfiguration.maxSpeed) / speed;
            double targetThetaVelocity = (Math.pow(-primary.right_stick_x, 3) * swerveControllerConfiguration.maxAngularVelocity) / speed;

            // Apply acceleration limits
            xVelocity = applyAcceleration(xVelocity, targetXVelocity, MAX_ACCELERATION, deltaTime);
            yVelocity = applyAcceleration(yVelocity, targetYVelocity, MAX_ACCELERATION, deltaTime);
            thetaVelocity = applyAcceleration(thetaVelocity, targetThetaVelocity, MAX_ACCELERATION, deltaTime);

            // Drive the robot
            swerveDrive.drive(new Translation2d(xVelocity, yVelocity), thetaVelocity, fieldRelative, true, headingCorrection);
            swerveDrive.updateOdometry();

            // Telemetry updates
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Field Oriented", fieldRelative);
            telemetry.addData("Robot Heading", swerveDrive.getYaw().getDegrees());
            telemetry.addData("Driver Pose", swerveDrive.getPose());
            telemetry.addData("X Velocity", xVelocity);
            telemetry.addData("Y Velocity", yVelocity);
            telemetry.addData("Theta Velocity", thetaVelocity);
        }

        // Method to apply acceleration limits
        private double applyAcceleration(double currentVelocity, double targetVelocity, double maxAcceleration, double deltaTime) {
            double velocityDifference = targetVelocity - currentVelocity;
            double maxChange = maxAcceleration * deltaTime;
            if (Math.abs(velocityDifference) > maxChange) {
                return currentVelocity + Math.signum(velocityDifference) * maxChange;
            } else {
                return targetVelocity;
            }
        }

        // Method to calculate the time difference between updates
        private double getDeltaTime() {
            long currentTime = System.nanoTime(); // Get current time in nanoseconds
            double deltaTime = (currentTime - lastUpdateTime) / 1_000_000_000.0; // Convert to seconds
            lastUpdateTime = currentTime; // Update last update time
            return deltaTime;
        }
    }
}
