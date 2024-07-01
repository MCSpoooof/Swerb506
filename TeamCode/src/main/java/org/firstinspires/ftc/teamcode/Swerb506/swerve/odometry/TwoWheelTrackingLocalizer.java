package org.firstinspires.ftc.teamcode.Swerb506.swerve.odometry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Swerb506.hardware.Encoder;
import org.firstinspires.ftc.teamcode.Swerb506.core.RobotConfiguration;

import java.util.Arrays;
import java.util.List;

public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {

    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 48/25.4; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -5.19; // X is the up and down direction
    public static double PARALLEL_Y = -0.77; // Y is the strafe direction

    public static double PERPENDICULAR_X = 3.39;
    public static double PERPENDICULAR_Y = -0.05;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    public TwoWheelTrackingLocalizer() {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        parallelEncoder = RobotConfiguration.ODOMETRY_PARALLEL.getAsEncoder();
        perpendicularEncoder = RobotConfiguration.ODOMETRY_PERPENDICULAR.getAsEncoder();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getRawVelocity()),
                encoderTicksToInches(perpendicularEncoder.getRawVelocity())
        );
    }
}
