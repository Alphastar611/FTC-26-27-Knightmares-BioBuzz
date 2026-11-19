package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware Hardware = new RobotHardware();
        Hardware.init(hardwareMap);

        Localization localization = new Localization(Hardware);
        DriveTrain drivetrain = new DriveTrain(Hardware);
        Feeder feeder = new Feeder(Hardware);
        TurretShooter turretShooter = new TurretShooter(Hardware);

        ElapsedTime telemetryTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        double loopMs;

        telemetry.setMsTransmissionInterval(100);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Ready");
            telemetry.addData("Start Pose", "(%.1f, %.1f) @ %.1f°", localization.x, localization.y, localization.yaw);
            telemetry.update();
            sleep(50);
        }
        if (isStopRequested()) return;

        telemetryTimer.reset();
        loopTimer.reset();

        while (opModeIsActive()) {
            loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            Hardware.beginLoop();

            // Driver 1 Controls
            if (gamepad1.backWasPressed()) {
                if (localization.isStationary()) localization.resetPose();
                else gamepad1.rumble(150);
            }
            if (gamepad1.aWasPressed()) turretShooter.toggleTurret();
            if (gamepad1.bWasPressed()) turretShooter.toggleShooter();
            if (gamepad1.leftBumperWasPressed()) feeder.toggleFlower();

            // Driver 2 Live Trims
            if (gamepad2.dpadRightWasPressed()) turretShooter.adjustHeading(-5);
            if (gamepad2.dpadLeftWasPressed()) turretShooter.adjustHeading(5);
            if (gamepad2.dpadDownWasPressed()) turretShooter.adjustVelocity(-20);
            if (gamepad2.dpadUpWasPressed()) turretShooter.adjustVelocity(20);

            // Update Subsystems
            localization.update();
            drivetrain.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            feeder.update(gamepad1.right_trigger, gamepad1.left_trigger, gamepad1.right_bumper,turretShooter.turretInRange);

            if (turretShooter.update(localization)) {
                gamepad1.rumble(150);
            }

            // Telemetry
            if (telemetryTimer.milliseconds() >= 250) {
                telemetry.addData("Loop Time", "%.1f ms (%.0f Hz)", loopMs, 1000.0 / Math.max(0.1, loopMs));
                telemetry.addData("Pose (X, Y, Yaw)", "%.1f, %.1f, %.1f°", localization.x, localization.y, localization.yaw);
                telemetry.addData("Goal (X, Y) | Dist", "%.1f, %.1f | %.1f in",
                        turretShooter.goalX, turretShooter.goalY, turretShooter.distance);
                telemetry.addData("Shooter Vel (Act / Cmd)", "%.0f / %.0f",
                        turretShooter.getActualVelocity(), turretShooter.getCommandedVelocity());
                telemetry.addData("Trims (Vel / Head)", "%d / %d",
                        turretShooter.velocityOffset, turretShooter.headingOffset);
                telemetry.addData("Pinpoint OK", localization.pinpointOK);
                telemetry.update();
                telemetryTimer.reset();
            }
        }
    }
}