package org.firstinspires.ftc.teamcode.Swerb506.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Swerb506.core.RobotHardware;
import org.firstinspires.ftc.teamcode.Swerb506.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.controller.PIDController;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Translation2d;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

@Config
@TeleOp(name = "TelePOP")
public class TelePOP extends RobotHardware {
    //public SwerveDrive swerveDrive;
    public static boolean fieldRelative = true;
    public boolean slowMode = false;
    public double speed;
    public final PIDController headingController = new PIDController (0.5,0,0.1);
    public boolean lockHeading = false;
    public double targetHeading;

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

    // Inner class for manual drive state
    class Drive_Manual extends Executive.StateBase<TelePOP> {
        @Override
        public void update() {
            super.update();

            if (gamepad1.right_stick_button) {
                swerveDrive.zeroGyro();
                swerveDrive.resetOdometry(new Pose2d());
            }

            if (gamepad1.right_stick_y > 0.25) {
                lockHeading = true;
                targetHeading = Math.PI + swerveDrive.imuOffset;
            }
            if (gamepad1.right_stick_y < -0.25) {
                lockHeading = true;
                targetHeading = 0 + swerveDrive.imuOffset;
            }

            double turn = gamepad1.left_trigger - gamepad1.right_trigger;

            if (Math.abs(turn) > 0.002) {
                lockHeading = false;
            }

            double error = normalizeRadians(normalizeRadians(targetHeading)-normalizeRadians(swerveDrive.getYaw().getDegrees()));
            double headingCorrection = -headingController.calculate(0, error) * 12.4 / swerveDrive.getVoltage();

            if (Math.abs(headingCorrection) < 0.01) {
                headingCorrection = 0;
            }

            swerveDrive.maintainHeading = (Math.abs(gamepad1.left_stick_x)) < 0.002
                    && Math.abs(gamepad1.left_stick_y) < 0.002
                    && Math.abs(turn) < 0.002
                    && Math.abs(headingCorrection) < 0.02;

            double rotationAmount = swerveDrive.getYaw().getDegrees() - swerveDrive.imuOffset;
            Pose2d drive = new Pose2d(
                    new Translation2d(joyScalar(gamepad1.left_stick_y, 0.001),
                            joystickScalar(gamepad1.left_stick_x, 0.001)).rotate(rotationAmount),
                    lockHeading ? headingCorrection :
                            joystickScalar(turn, 0.01)
            );

            // Toggle field-relative mode
            if (primary.BOnce()) {
                fieldRelative = !fieldRelative;
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
            if (!slowMode) {
                speed = 1;
            }
            else {
                speed = 0.3;
            }

            // Calculate velocities with cubic scaling for smoother control
            double xV = (Math.pow(-primary.left_stick_y, 3) * swerveControllerConfiguration.maxSpeed) * speed;
            double yV = (Math.pow(-primary.left_stick_x, 3) * swerveControllerConfiguration.maxSpeed) * speed;
            double thetaV = (Math.pow(-primary.right_stick_x, 3) * swerveControllerConfiguration.maxAngularVelocity) * speed;

            // Drive the robot
            swerveDrive.drive(new Translation2d(xV, yV), thetaV, fieldRelative, true);
            swerveDrive.updateOdometry();

            // Telemetry updates
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Field Oriented", fieldRelative);
            telemetry.addData("Robot Heading", swerveDrive.getYaw().getDegrees());
            telemetry.addData("Driver Pose", swerveDrive.getPose());
            telemetry.update();
        }
    }
}
