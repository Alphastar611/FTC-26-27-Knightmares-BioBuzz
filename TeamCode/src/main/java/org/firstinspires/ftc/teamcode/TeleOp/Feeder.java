package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Feeder {

    public static final double INTAKE_VEL = 2400.0;
    public static final double TRANSFER_FEED_VEL = 2800.0;
    public static final double TRANSFER_IDLE_POWER = -0.15;

    private final RobotHardware Hardware;
    public boolean flowerOn = false;

    private boolean isFeeding = false;
    private boolean lastTransferWasVel = true;
    private double lastIntakeVel = Double.NaN;
    private double lastTransferCmd = Double.NaN;
    private double lastFlowerPos = Double.NaN;

    public Feeder(RobotHardware hw) {
        this.Hardware = hw;
    }

    public void toggleFlower() {
        flowerOn = !flowerOn;
    }

    public void update(double rightTrigger, double leftTrigger, boolean rightBumper, boolean turretInRange) {
        // Trigger hysteresis with safety check: prevents feeding if turret is out of range or wrapping
        if (!isFeeding && rightTrigger > 0.15 && turretInRange) {
            isFeeding = true;
        } else if (isFeeding && (rightTrigger < 0.05 || !turretInRange)) {
            isFeeding = false;
        }

        // Transfer (slot 4)
        if (isFeeding) {
            if (!lastTransferWasVel) {
                Hardware.Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastTransferWasVel = true;
            }
            if (Hardware.needsWrite(TRANSFER_FEED_VEL, lastTransferCmd, 5.0, 4)) {
                Hardware.Transfer.setVelocity(TRANSFER_FEED_VEL);
                lastTransferCmd = TRANSFER_FEED_VEL;
            }
        } else {
            if (lastTransferWasVel) {
                Hardware.Transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lastTransferWasVel = false;
            }
            if (Hardware.needsWrite(TRANSFER_IDLE_POWER, lastTransferCmd, 0.001, 4)) {
                Hardware.Transfer.setPower(TRANSFER_IDLE_POWER);
                lastTransferCmd = TRANSFER_IDLE_POWER;
            }
        }

        // Intake (slot 5)
        double intakeTrigger = Math.abs(leftTrigger) < DriveTrain.STICK_DEADBAND ? 0 : leftTrigger;
        double intakeCmd = isFeeding ? INTAKE_VEL
                : rightBumper ? -INTAKE_VEL
                : intakeTrigger * INTAKE_VEL;
        if (Hardware.needsWrite(intakeCmd, lastIntakeVel, 5.0, 5)) {
            Hardware.Intake.setVelocity(intakeCmd);
            lastIntakeVel = intakeCmd;
        }

        // Flower servo (slot 7)
        double flowerCmd = flowerOn ? 0.31 : 0.025;
        if (Hardware.needsWrite(flowerCmd, lastFlowerPos, 0.001, 7)) {
            Hardware.FlowerMechanism.setPosition(flowerCmd);
            lastFlowerPos = flowerCmd;
        }
    }
}