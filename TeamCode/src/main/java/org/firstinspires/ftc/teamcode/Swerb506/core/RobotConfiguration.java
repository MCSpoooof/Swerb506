package org.firstinspires.ftc.teamcode.Swerb506.core;

import static org.firstinspires.ftc.teamcode.Swerb506.core.RobotConstants.AXON_CONTINUOUS_PWM;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Swerb506.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.Encoder;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.ExpansionHub;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.IMU;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.Motor;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.MotorTypes;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.Servo;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.Webcam;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.meta.HardwareDevice;

/**
 * Enum to configure and retrieve hardware components for the robot.
 */
public enum RobotConfiguration {

    // IMU sensor configuration
    IMU(
            new IMU("imu")
    ),

    // Control Hub configuration
    CONTROL_HUB(
            new ExpansionHub("Control Hub")
                    .configureBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
    ),

    // Uncomment when using an additional Expansion Hub
    /*
    EXPANSION_HUB(
        new ExpansionHub("Expansion Hub 2")
            .configureBulkCachingMode(LynxModule.BulkCachingMode.OFF)
    ),
    */

    // Drive motors configuration with PIDF
    DRIVE_FRONT_LEFT(
            new Motor("cm2")
                    .configureDirection(DcMotorSimple.Direction.REVERSE)
                    .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                    .configureRunMode(DcMotor.RunMode.RUN_USING_ENCODER)
                    .setType(MotorTypes.DRIVE)
                    .configurePIDF(1.0, 0.0, 0.0, 0.0)
    ),
    DRIVE_FRONT_RIGHT(
            new Motor("cm0")
                    .configureDirection(DcMotorSimple.Direction.FORWARD)
                    .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                    .configureRunMode(DcMotor.RunMode.RUN_USING_ENCODER)
                    .setType(MotorTypes.DRIVE)
                    .configurePIDF(1.0, 0.0, 0.0, 0.0)
    ),
    DRIVE_BACK_LEFT(
            new Motor("cm3")
                    .configureDirection(DcMotorSimple.Direction.REVERSE)
                    .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                    .configureRunMode(DcMotor.RunMode.RUN_USING_ENCODER)
                    .setType(MotorTypes.DRIVE)
                    .configurePIDF(1.0, 0.0, 0.0, 0.0)
    ),
    DRIVE_BACK_RIGHT(
            new Motor("cm1")
                    .configureDirection(DcMotorSimple.Direction.FORWARD)
                    .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                    .configureRunMode(DcMotor.RunMode.RUN_USING_ENCODER)
                    .setType(MotorTypes.DRIVE)
                    .configurePIDF(1.0, 0.0, 0.0, 0.0)
    ),

    // Absolute encoders configuration
    ABSOLUTE_FRONT_LEFT(
            new AbsoluteEncoder("ca1")
                    .zero(231.27)
                    .setInverted(false)
    ),
    ABSOLUTE_FRONT_RIGHT(
            new AbsoluteEncoder("ca2")
                    .zero(359.23)
                    .setInverted(false)
    ),
    ABSOLUTE_BACK_LEFT(
            new AbsoluteEncoder("ca0")
                    .zero(100.8)
                    .setInverted(false)
    ),
    ABSOLUTE_BACK_RIGHT(
            new AbsoluteEncoder("ca3")
                    .zero(165.7)
                    .setInverted(false)
    ),

    // Continuous servos configuration
    ANGLE_FRONT_LEFT(
            new ContinuousServo("cs2")
                    .configureDirection(DcMotorSimple.Direction.FORWARD)
                    .configurePIDWrapping()
                    .configurePWMRange(AXON_CONTINUOUS_PWM)
                    .configurePIDF(0.01, 0.0, 0.0)
                    .configureFF(0.04)
                    .configureEncoder(ABSOLUTE_FRONT_LEFT.getAsAbsoluteEncoder())
    ),
    ANGLE_FRONT_RIGHT(
            new ContinuousServo("cs0")
                    .configureDirection(DcMotorSimple.Direction.FORWARD)
                    .configurePIDWrapping()
                    .configurePWMRange(AXON_CONTINUOUS_PWM)
                    .configurePIDF(0.01, 0.0, 0.0)
                    .configureFF(0.03)
                    .configureEncoder(ABSOLUTE_FRONT_RIGHT.getAsAbsoluteEncoder())
    ),
    ANGLE_BACK_LEFT(
            new ContinuousServo("cs3")
                    .configureDirection(DcMotorSimple.Direction.FORWARD)
                    .configurePIDWrapping()
                    .configurePWMRange(AXON_CONTINUOUS_PWM)
                    .configurePIDF(0.01, 0.0, 0.0)
                    .configureFF(0.04)
                    .configureEncoder(ABSOLUTE_BACK_LEFT.getAsAbsoluteEncoder())
    ),
    ANGLE_BACK_RIGHT(
            new ContinuousServo("cs1")
                    .configureDirection(DcMotorSimple.Direction.FORWARD)
                    .configurePIDWrapping()
                    .configurePWMRange(AXON_CONTINUOUS_PWM)
                    .configurePIDF(0.01, 0.0, 0.0)
                    .configureFF(0.05)
                    .configureEncoder(ABSOLUTE_BACK_RIGHT.getAsAbsoluteEncoder())
    ),

    // Encoders for odometry configuration
    ODOMETRY_PARALLEL(
            new Encoder("cm3")
                    .setDirection(Encoder.Direction.REVERSE)
    ),
    ODOMETRY_PERPENDICULAR(
            new Encoder("cm0")
                    .setDirection(Encoder.Direction.FORWARD)
    );

    // Uncomment when using a webcam
    /*
    WEBCAM(
        new Webcam("cu0")
            .configureCameraResolution(640, 480)
            .configureVisionProcessor(new SpikeDetectionProcessor())
    );
    */

    private final HardwareDevice device;

    RobotConfiguration(HardwareDevice device) {
        this.device = device;
    }

    public HardwareDevice getAsHardwareDevice() {
        return device;
    }

    public IMU getAsIMU() {
        ensureType(IMU.class);
        return (IMU) device;
    }

    public Motor getAsMotor() {
        ensureType(Motor.class);
        return (Motor) device;
    }

    public Servo getAsServo() {
        ensureType(Servo.class);
        return (Servo) device;
    }

    public ContinuousServo getAsContinuousServo() {
        ensureType(ContinuousServo.class);
        return (ContinuousServo) device;
    }

    public AbsoluteEncoder getAsAbsoluteEncoder() {
        ensureType(AbsoluteEncoder.class);
        return (AbsoluteEncoder) device;
    }

    public ExpansionHub getAsExpansionHub() {
        ensureType(ExpansionHub.class);
        return (ExpansionHub) device;
    }

    public Encoder getAsEncoder() {
        ensureType(Encoder.class);
        return (Encoder) device;
    }

    public Webcam getAsWebcam() {
        ensureType(Webcam.class);
        return (Webcam) device;
    }

    private void ensureType(Class<?> type) {
        if (!type.isInstance(device)) {
            throw new IllegalStateException("Device is not of type " + type.getSimpleName());
        }
    }
}