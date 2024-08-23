package org.firstinspires.ftc.teamcode.Swerb506.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Swerb506.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Swerb506.core.RobotHardware;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.Motor;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Translation2d;



@Config
@TeleOp(name = "Swerve Tuning")
public class SwerveTuning extends RobotHardware {
    public static double
            fLOffset = 0, fROffset = 0, bLOffset = 0, bROffset = 0;

    public static TuneState state = TuneState.DRIVE;
    public static WheelPosition angleWheel = WheelPosition.FRONT_LEFT;

    // Angle PID parameters
    public static double aP = 0.08, aI = 0.0, aD = 0.0, aFF = 0.0, angleSetpointStart = 0.0, angleSetpointEnd = 30.0, kStatic = 0.0;
    private double prevAP = aP, prevAI = aI, prevAD = aD, prevAFF = aFF;

    // Drive PID parameters
    public static double dP = 0.1, dI = 0.0, dD = 0.0, dFF = 0.0;
    private double prevDP = dP, prevDI = dI, prevDD = dD, prevDFF = dFF;

    private boolean reverse = false;
    private ElapsedTimer changeDirection;
    public static double changeDirectionTime = 1.0;
    private int index = 0;
    public static boolean precision = false;
    private double precisionMode = 0.35;

    enum WheelPosition {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    enum TuneState {
        DRIVE,
        OFFSET,
        ANGLE_PID,
        DRIVE_PID,
        SINGLE_ANGLE_PID,
        SINGLE_MOTOR,
        kStatic
    }

    private AbsoluteEncoder frontLeft, frontRight, backLeft, backRight;
    private ContinuousServo fL, fR, bL, bR;

    @Override
    public void init() {
        super.init();
        frontLeft = RobotConfiguration.ABSOLUTE_FRONT_LEFT.getAsAbsoluteEncoder();
        frontRight = RobotConfiguration.ABSOLUTE_FRONT_RIGHT.getAsAbsoluteEncoder();
        backLeft = RobotConfiguration.ABSOLUTE_BACK_LEFT.getAsAbsoluteEncoder();
        backRight = RobotConfiguration.ABSOLUTE_BACK_RIGHT.getAsAbsoluteEncoder();

        fL = RobotConfiguration.ANGLE_FRONT_LEFT.getAsContinuousServo();
        fR = RobotConfiguration.ANGLE_FRONT_RIGHT.getAsContinuousServo();
        bL = RobotConfiguration.ANGLE_BACK_LEFT.getAsContinuousServo();
        bR = RobotConfiguration.ANGLE_BACK_RIGHT.getAsContinuousServo();

        changeDirection = new ElapsedTimer();
        changeDirection.reset();
    }

    @Override
    public void loop() {
        super.loop();
        switch (state) {
            case OFFSET:
                frontLeft.zero(fLOffset);
                frontRight.zero(fROffset);
                backLeft.zero(bLOffset);
                backRight.zero(bROffset);

                for (RobotConfiguration configuration : RobotConfiguration.values()) {
                    HardwareDevice device = configuration.getAsHardwareDevice();
                    if (device instanceof AbsoluteEncoder) {
                        telemetry.addData(configuration.name() + " Abs", ((AbsoluteEncoder) device).getCurrentPosition());
                    }
                }
                break;

            case DRIVE:
                double xVelocity;
                double yVelocity;
                double angVelocity;
                double prec = precision ? precisionMode : 1.0;
                if (primary.dpadUp()) {
                    xVelocity = swerveControllerConfiguration.maxSpeed * prec;
                    yVelocity = 0.0;
                    angVelocity = 0.0;
                } else if (primary.dpadRight()) {
                    xVelocity = 0.0;
                    yVelocity = swerveControllerConfiguration.maxSpeed * prec;
                    angVelocity = 0.0;
                } else {
                    xVelocity = -primary.left_stick_y * swerveControllerConfiguration.maxSpeed * prec;
                    yVelocity = -primary.left_stick_x * swerveControllerConfiguration.maxSpeed * prec;
                    angVelocity = -primary.right_stick_x * swerveControllerConfiguration.maxAngularVelocity * prec;
                }

                swerveDrive.drive(new Translation2d(xVelocity, yVelocity), angVelocity, false, true);
                break;

            case SINGLE_MOTOR:
                Motor motor;
                switch (angleWheel) {
                    case FRONT_LEFT:
                        motor = RobotConfiguration.DRIVE_FRONT_LEFT.getAsMotor();
                        break;
                    case FRONT_RIGHT:
                        motor = RobotConfiguration.DRIVE_FRONT_RIGHT.getAsMotor();
                        break;
                    case BACK_LEFT:
                        motor = RobotConfiguration.DRIVE_BACK_LEFT.getAsMotor();
                        break;
                    default:
                        motor = RobotConfiguration.DRIVE_BACK_RIGHT.getAsMotor();
                }

                motor.setPower(-primary.left_stick_y * swerveControllerConfiguration.maxSpeed);
                break;

            case ANGLE_PID:
                if (prevAP != aP || prevAI != aI || prevAD != aD || prevAFF != aFF) {
                    fL.configurePIDF(aP, aI, aD, aFF);
                    fR.configurePIDF(aP, aI, aD, aFF);
                    bL.configurePIDF(aP, aI, aD, aFF);
                    bR.configurePIDF(aP, aI, aD, aFF);
                    prevAP = aP;
                    prevAI = aI;
                    prevAD = aD;
                    prevAFF = aFF;
                    index++;
                }

                double angleXVelocity = -primary.left_stick_y * swerveControllerConfiguration.maxSpeed;
                double angleYVelocity = -primary.left_stick_x * swerveControllerConfiguration.maxSpeed;
                double angleAngVelocity = -primary.right_stick_x * swerveControllerConfiguration.maxAngularVelocity;

                swerveDrive.drive(new Translation2d(angleXVelocity, angleYVelocity), angleAngVelocity, false, true);
                telemetry.addData("Index", index);
                telemetry.addData("Front Left Servo Position", fL.getPosition());  // Ensure this method exists
                telemetry.addData("Front Right Servo Position", fR.getPosition());  // Ensure this method exists
                telemetry.addData("Back Left Servo Position", bL.getPosition());  // Ensure this method exists
                telemetry.addData("Back Right Servo Position", bR.getPosition());  // Ensure this method exists
                break;

            case DRIVE_PID:
                if (prevDP != dP || prevDI != dI || prevDD != dD || prevDFF != dFF) {
                    RobotConfiguration.DRIVE_FRONT_LEFT.getAsMotor().configurePIDF(dP, dI, dD, dFF);
                    RobotConfiguration.DRIVE_FRONT_RIGHT.getAsMotor().configurePIDF(dP, dI, dD, dFF);
                    RobotConfiguration.DRIVE_BACK_LEFT.getAsMotor().configurePIDF(dP, dI, dD, dFF);
                    RobotConfiguration.DRIVE_BACK_RIGHT.getAsMotor().configurePIDF(dP, dI, dD, dFF);
                    prevDP = dP;
                    prevDI = dI;
                    prevDD = dD;
                    prevDFF = dFF;
                    index++;
                }

                double driveX = -primary.left_stick_y * swerveControllerConfiguration.maxSpeed;
                double driveY = -primary.left_stick_x * swerveControllerConfiguration.maxSpeed;
                double driveAng = -primary.right_stick_x * swerveControllerConfiguration.maxAngularVelocity;

                swerveDrive.drive(new Translation2d(driveX, driveY), driveAng, false, true);
                telemetry.addData("Index", index);
                telemetry.addData("Front Left Motor Power", RobotConfiguration.DRIVE_FRONT_LEFT.getAsMotor().getPower());
                telemetry.addData("Front Right Motor Power", RobotConfiguration.DRIVE_FRONT_RIGHT.getAsMotor().getPower());
                telemetry.addData("Back Left Motor Power", RobotConfiguration.DRIVE_BACK_LEFT.getAsMotor().getPower());
                telemetry.addData("Back Right Motor Power", RobotConfiguration.DRIVE_BACK_RIGHT.getAsMotor().getPower());
                telemetry.addData("Front Left Motor Velocity", RobotConfiguration.DRIVE_FRONT_LEFT.getAsMotor().getVelocity());
                telemetry.addData("Front Right Motor Velocity", RobotConfiguration.DRIVE_FRONT_RIGHT.getAsMotor().getVelocity());
                telemetry.addData("Back Left Motor Velocity", RobotConfiguration.DRIVE_BACK_LEFT.getAsMotor().getVelocity());
                telemetry.addData("Back Right Motor Velocity", RobotConfiguration.DRIVE_BACK_RIGHT.getAsMotor().getVelocity());
                break;

            case SINGLE_ANGLE_PID:
                ContinuousServo servo;
                switch (angleWheel) {
                    case FRONT_LEFT:
                        servo = RobotConfiguration.ANGLE_FRONT_LEFT.getAsContinuousServo();
                        break;
                    case FRONT_RIGHT:
                        servo = RobotConfiguration.ANGLE_FRONT_RIGHT.getAsContinuousServo();
                        break;
                    case BACK_LEFT:
                        servo = RobotConfiguration.ANGLE_BACK_LEFT.getAsContinuousServo();
                        break;
                    default:
                        servo = RobotConfiguration.ANGLE_BACK_RIGHT.getAsContinuousServo();
                }
                servo.configurePIDF(aP, aI, aD, aFF);

                if (reverse) {
                    if (changeDirection.seconds() > changeDirectionTime) {
                        changeDirection.reset();
                        reverse = false;
                    }
                    servo.setReference(angleSetpointStart, aFF);
                } else {
                    if (changeDirection.seconds() > changeDirectionTime) {
                        changeDirection.reset();
                        reverse = true;
                    }
                    servo.setReference(angleSetpointEnd, aFF);
                }
                break;

            case kStatic:
                ContinuousServo s;
                switch (angleWheel) {
                    case FRONT_LEFT:
                        s = RobotConfiguration.ANGLE_FRONT_LEFT.getAsContinuousServo();
                        break;
                    case FRONT_RIGHT:
                        s = RobotConfiguration.ANGLE_FRONT_RIGHT.getAsContinuousServo();
                        break;
                    case BACK_LEFT:
                        s = RobotConfiguration.ANGLE_BACK_LEFT.getAsContinuousServo();
                        break;
                    default:
                        s = RobotConfiguration.ANGLE_BACK_RIGHT.getAsContinuousServo();
                }
                s.setPower(kStatic);
                telemetry.addData("Static Power", kStatic);
                break;
        }

        telemetry.addData("Mode", state);
        telemetry.addData("Angle Wheel", angleWheel);
        telemetry.addData("Current Battery Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage()); // Adjust as needed
        telemetry.update();
    }
}