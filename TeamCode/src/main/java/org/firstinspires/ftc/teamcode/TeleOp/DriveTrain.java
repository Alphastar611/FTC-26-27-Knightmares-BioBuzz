package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {

    public static final double STICK_DEADBAND = 0.05;

    private final RobotHardware hw;
    private final double[] lastDrive = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    public DriveTrain(RobotHardware hw) {
        this.hw = hw;
    }

    public void update(double rawY, double rawX, double rawRX) {
        double y = deadband(rawY);
        double x = deadband(rawX);
        double rx = deadband(rawRX);

        double fl = (y - x) - rx;
        double bl = (y + x) - rx;
        double fr = (y + x) + rx;
        double br = (y - x) + rx;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))));

        setMotor(hw.FL, 0, fl / max);
        setMotor(hw.BL, 1, bl / max);
        setMotor(hw.FR, 2, fr / max);
        setMotor(hw.BR, 3, br / max);
    }

    private void setMotor(DcMotor motor, int idx, double power) {
        if (hw.needsWrite(power, lastDrive[idx], 0.005, idx)) {
            motor.setPower(power);
            lastDrive[idx] = power;
        }
    }

    private double deadband(double val) {
        return Math.abs(val) < STICK_DEADBAND ? 0 : val;
    }
}