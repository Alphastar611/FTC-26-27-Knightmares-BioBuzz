package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Feeder {

    public static final double INTAKE_VEL = 2400.0;
    public static final double TRANSFER_FEED_VEL = 2800.0;
    public static final double TRANSFER_IDLE_POWER = -0.15;

    private final RobotHardware hw;
    public boolean flowerOn = false;

    private boolean isFeeding = false;
    private boolean lastTransferWasVel = true;
    private double lastIntakeVel = Double.NaN;
    private double lastTransferCmd = Double.NaN;
    private double lastFlowerPos = Double.NaN;

    public Feeder(RobotHardware hw) {
        this.hw = hw;
    }

    public void toggleFlower() {
        flowerOn = !flowerOn;
    }

    public void update(double rightTrigger, double leftTrigger, boolean rightBumper) {
        // Trigger hysteresis prevents mode-switch chatter near 0.1
        if (!isFeeding && rightTrigger > 0.15) isFeeding = true;
        else if (isFeeding && rightTrigger < 0.05) isFeeding = false;

        // Transfer (slot 4)
        if (isFeeding) {
            if (!lastTransferWasVel) {
                hw.Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastTransferWasVel = true;
            }
            if (hw.needsWrite(TRANSFER_FEED_VEL, lastTransferCmd, 5.0, 4)) {
                hw.Transfer.setVelocity(TRANSFER_FEED_VEL);
                lastTransferCmd = TRANSFER_FEED_VEL;
            }
        } else {
            if (lastTransferWasVel) {
                hw.Transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lastTransferWasVel = false;
            }
            if (hw.needsWrite(TRANSFER_IDLE_POWER, lastTransferCmd, 0.001, 4)) {
                hw.Transfer.setPower(TRANSFER_IDLE_POWER);
                lastTransferCmd = TRANSFER_IDLE_POWER;
            }
        }

        // Intake (slot 5)
        double intakeTrigger = Math.abs(leftTrigger) < DriveTrain.STICK_DEADBAND ? 0 : leftTrigger;
        double intakeCmd = isFeeding ? INTAKE_VEL
                : rightBumper ? -INTAKE_VEL
                : intakeTrigger * INTAKE_VEL;
        if (hw.needsWrite(intakeCmd, lastIntakeVel, 5.0, 5)) {
            hw.Intake.setVelocity(intakeCmd);
            lastIntakeVel = intakeCmd;
        }

        // Flower servo (slot 7)
        double flowerCmd = flowerOn ? 0.425 : 0.0;
        if (hw.needsWrite(flowerCmd, lastFlowerPos, 0.001, 7)) {
            hw.FlowerMechanism.setPosition(flowerCmd);
            lastFlowerPos = flowerCmd;
        }
    }
}