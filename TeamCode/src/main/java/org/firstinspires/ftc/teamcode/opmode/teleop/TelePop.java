package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;

@Config
@TeleOp(name="Manual", group="A")
public class TelePop extends RobotHardware {
    public static double precisionMode = 1.0;
    private final double precisionPercentage = 0.4;
    public static boolean fieldRelative = true;
    public static boolean headingCorrection = true;
    private final Executive.StateMachine<TelePop> stateMachine;

    public static double slideDownSpeed = 0.6, slideSpeed = 1.0, liftSpeed = 1.0;

    public TelePop() {
        stateMachine = new Executive.StateMachine<>(this);
        stateMachine.update();
    }

    @Override
    public void init() {
    }

    @Override
    public void init_loop() {
        super.init_loop();
        stateMachine.update();
    }

    @Override
    public void start() {
        super.start();
        stateMachine.changeState(Executive.StateMachine.StateType.DRIVE, new Drive_Manual());

        if (fieldRelative)
            primary.setLedColor(0.0, 0.0, 1.0, -1);
        else
            primary.setLedColor(1.0, 1.0, 0.0, -1);

        swerveDrive.zeroGyro();
        swerveDrive.resetOdometry(new Pose2d());
    }

    @Override
    public void loop() {
        super.loop();
        stateMachine.update();
    }

    class Drive_Manual extends Executive.StateBase<TelePop> {
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
                if (fieldRelative)
                    primary.setLedColor(0.0, 0.0, 1.0, -1);
                else
                    primary.setLedColor(1.0, 1.0, 0.0, -1);
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
        }
    }
}