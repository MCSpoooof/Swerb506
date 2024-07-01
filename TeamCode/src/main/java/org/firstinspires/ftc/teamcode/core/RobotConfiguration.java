package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.AXON_CONTINUOUS_PWM;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.AXON_PWM;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.ExpansionHub;
import org.firstinspires.ftc.teamcode.hardware.IMU;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorTypes;
import org.firstinspires.ftc.teamcode.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionProcessor;

public enum RobotConfiguration {
    IMU(
            new IMU("imu")
    ),
    CONTROL_HUB(
            new ExpansionHub("Control Hub")
            .configureBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
    ),
    /*EXPANSION_HUB(
            new ExpansionHub("Expansion Hub 2")
            .configureBulkCachingMode(LynxModule.BulkCachingMode.OFF)
    ),*/
    DRIVE_FRONT_LEFT(
            new Motor("cm2")
            .configureDirection(DcMotorSimple.Direction.REVERSE)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.DRIVE)
    ),
    DRIVE_FRONT_RIGHT(
            new Motor("cm0")
            .configureDirection(DcMotorSimple.Direction.FORWARD)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.DRIVE)
    ),
    DRIVE_BACK_LEFT(
            new Motor("cm3")
            .configureDirection(DcMotorSimple.Direction.REVERSE)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.DRIVE)
    ),
    DRIVE_BACK_RIGHT(
            new Motor("cm1")
            .configureDirection(DcMotorSimple.Direction.FORWARD)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.DRIVE)
    ),
    ///////////////////////
    ABSOLUTE_FRONT_LEFT(
            new AbsoluteEncoder("ca1")
            .zero(230.9)
            .setInverted(false)
    ),
    ABSOLUTE_FRONT_RIGHT(
            new AbsoluteEncoder("ca2")
            .zero(2.39)
            .setInverted(false)
    ),
    ABSOLUTE_BACK_LEFT(
            new AbsoluteEncoder("ca0")
            .zero(103.41)
            .setInverted(false)
    ),
    ABSOLUTE_BACK_RIGHT(
            new AbsoluteEncoder("ca3")
            .zero(164.94)
            .setInverted(false)
    ),
    //////////////////////
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
    ODOMETRY_PARALLEL(
            new Encoder("cm0")
            .setDirection(Encoder.Direction.FORWARD)
    ),
    ODOMETRY_PERPENDICULAR(
            new Encoder("cm3")
            .setDirection(Encoder.Direction.REVERSE)
    );
    /*WEBCAM(
            new Webcam("cu0")
            .configureCameraResolution(640, 480)
            .configureVisionProcessor(new SpikeDetectionProcessor())
    );*/

    private final HardwareDevice device;

    RobotConfiguration(HardwareDevice device) {
        this.device = device;
    }

    public HardwareDevice getAsHardwareDevice() {
        return device;
    }

    public IMU getAsIMU() {
        if(!(device instanceof IMU))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (IMU) device;
    }

    public Motor getAsMotor() {
        if(!(device instanceof Motor))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (Motor) device;
    }

    public Servo getAsServo() {
        if(!(device instanceof Servo))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (Servo) device;
    }

    public ContinuousServo getAsContinuousServo() {
        if(!(device instanceof ContinuousServo))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (ContinuousServo) device;
    }

    public AbsoluteEncoder getAsAbsoluteEncoder() {
        if(!(device instanceof AbsoluteEncoder))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (AbsoluteEncoder) device;
    }

    public ExpansionHub getAsExpansionHub() {
        if(!(device instanceof ExpansionHub))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (ExpansionHub) device;
    }

    public Encoder getAsEncoder() {
        if(!(device instanceof Encoder))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (Encoder) device;
    }
    
    public Webcam getAsWebcam() {
        if(!(device instanceof Webcam))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (Webcam) device;
    }
}