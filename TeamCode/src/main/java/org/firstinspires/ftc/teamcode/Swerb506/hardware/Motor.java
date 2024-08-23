package org.firstinspires.ftc.teamcode.Swerb506.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.Swerb506.hardware.meta.HardwareStatus;
import org.firstinspires.ftc.teamcode.Swerb506.core.RobotConstants;
import org.firstinspires.ftc.teamcode.Swerb506.swerve.configuration.PIDFConfig;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.controller.PIDController;

public class Motor extends HardwareDevice {
    private DcMotorEx device;
    private DcMotorSimple.Direction direction = Direction.FORWARD;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    private MotorTypes type = MotorTypes.OTHER;
    private final PIDController controller = new PIDController(0.0, 0.0, 0.0, 0.01);
    private double lastPower;
    private int offset;

    public Motor(String configName) {
        super(configName, DcMotorEx.class);
    }

    public Motor(String configName, PIDFConfig pidfConfig) {
        this(configName);
        configurePIDF(pidfConfig.p, pidfConfig.i, pidfConfig.d, pidfConfig.f);
        setPIDTolerance(1.0);
    }

    @Override
    public void initialize(Object device) {
        if (!getDeviceClass().isInstance(device)) {
            setStatus(HardwareStatus.MISSING);
            return;
        }
        this.device = (DcMotorEx) device;
        this.device.setZeroPowerBehavior(zeroPowerBehavior);
        this.device.setDirection(direction);
        this.device.setMode(runMode);
        setStatus(HardwareStatus.SUCCESS);
    }

    public Motor configurePIDF(double p, double i, double d, double f) {
        if (device != null) {
            device.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
        }
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
        controller.reset();
        return this;
    }

    public Motor setPIDTolerance(double positionTolerance) {
        controller.setTolerance(positionTolerance);
        return this;
    }

    public Motor configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
        if (device != null) {
            device.setZeroPowerBehavior(zeroPowerBehavior);
        }
        return this;
    }

    public Motor configureDirection(DcMotor.Direction direction) {
        this.direction = direction;
        if (device != null) {
            device.setDirection(direction);
        }
        return this;
    }

    public Motor configureRunMode(DcMotor.RunMode runMode) {
        this.runMode = runMode;
        if (device != null) {
            device.setMode(runMode);
        }
        return this;
    }

    public DcMotor.Direction getDirection() {
        return direction;
    }

    public DcMotor.RunMode getRunMode() {
        return runMode;
    }

    public MotorTypes getType() {
        return type;
    }

    public Motor setType(MotorTypes type) {
        this.type = type;
        return this;
    }

    public double getPower() {
        return lastPower;
    }

    public void setPower(double power) {
        if (getStatus().equals(HardwareStatus.MISSING) || Math.abs(power - lastPower) < RobotConstants.MOTOR_CACHE_TOLERANCE) return;
        lastPower = power;
        if (device != null) {
            device.setPower(power);
        }
    }

    public void setReference(double setpoint) {
        setReference(setpoint, getEncoderValue(), 0.0);
    }

    public void setReference(double setpoint, double measurement) {
        setReference(setpoint, measurement, 0.0);
    }

    public void setReference(double setpoint, double measurement, double feedforward) {
        if (getStatus().equals(HardwareStatus.MISSING)) return;
        double pidOutput = controller.calculate(measurement, setpoint);
        setPower(pidOutput + feedforward);
    }

    public void setEncoderPositionOffset(int offset) {
        this.offset += offset;
    }

    public int getRawEncoderValue() {
        return device != null ? device.getCurrentPosition() : 0;
    }

    public int getEncoderValue() {
        return device != null ? device.getCurrentPosition() - offset : 0;
    }

    public double getVelocity() {
        return device != null ? device.getVelocity() : 0.0;
    }

    public double getCurrent() {
        return device != null ? device.getCurrent(CurrentUnit.AMPS) : 0.0;
    }
}