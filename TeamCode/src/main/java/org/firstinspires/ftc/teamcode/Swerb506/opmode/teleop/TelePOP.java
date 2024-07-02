package org.firstinspires.ftc.teamcode.Swerb506.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Swerb506.core.RobotHardware;
import org.firstinspires.ftc.teamcode.Swerb506.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Translation2d;

@Config
@TeleOp(name= "TelePOP")
public class TelePOP extends RobotHardware {
    public static double precisionMode = 1.0;
    private final double precisionPercentage = 0.4;
    public static boolean fieldRelative = true;
    public static boolean headingCorrection = true;
    private final Executive.StateMachine<TelePOP> stateMachine;

    public TelePOP() {
        stateMachine = new Executive.StateMachine<>(this);
        stateMachine.update();
    }

    @Override
    public void init() {
        super.init();
        stateMachine.init();
//Init things here
    }

    @Override
    public void init_loop() {
        super.init_loop();
        stateMachine.update();
//Init more things here?
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

            if (primary.AOnce()) {
//                swerveDrive.setMaximumSpeed(precisionMode ? SWERVE_MAX_SPEED : SWERVE_PRECISION_SPEED);
                precisionMode = precisionMode == 1.0 ? precisionPercentage : 1.0;
            }

            if (primary.YOnce()) {
                swerveDrive.zeroGyro();
                swerveDrive.resetOdometry(new Pose2d());
            }

            if (primary.BOnce()) {
                fieldRelative = !fieldRelative;
            }

            if (primary.XOnce())
                headingCorrection = !headingCorrection;

            if (primary.rightStickButtonOnce()) {
                SwerveDrive.lastHeadingRadians = (3.0 * Math.PI) / 2.0;
                SwerveDrive.updatedHeading = true;
            }

            double xV = -primary.left_stick_y * swerveControllerConfiguration.maxSpeed * precisionMode;
            double yV = -primary.left_stick_x * swerveControllerConfiguration.maxSpeed * precisionMode;
            double thetaV = -primary.right_stick_x * swerveControllerConfiguration.maxAngularVelocity * precisionMode;
            swerveDrive.drive(new Translation2d(xV, yV), thetaV, fieldRelative, true, headingCorrection);
            swerveDrive.updateOdometry();
            telemetry.addData("Robot Oriantation", swerveDrive.getYaw().getDegrees());
        }
    }
}