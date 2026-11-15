package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static com.pedropathing.ivy.commands.Commands.*;   // instant, waitMs, waitUntil, infinite

import org.firstinspires.ftc.teamcode.pedro.Constants;



@Autonomous(name = "OneTipUp", group = "Goal Up")
public class OneTipUp extends LinearOpMode {

    final int shooterVelocity = 1440;
    final int intakeVelocity = 2400;
    final int transferVelocity = 2800;
    private Follower follower;
    private DcMotor Shooter;
    private DcMotor Intake;
    private DcMotor Transfer;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.9267, 8.1789, 90);
    private final Pose scoreToFlower1 = poseFactory.of(12.909, 90.5752, 90);
    private final Pose scoreToFlower1Control1 = poseFactory.of(22.8636, 35.9542, 0);
    public Path park() {
        return Paths.curve(start, scoreToFlower1Control1, scoreToFlower1).linear(start, scoreToFlower1);
    }
    private Command stopShooting() {
        return parallel(
                instant(() -> Intake.setPower(0)),
                instant(() -> Transfer.setPower(0)),
                instant(() -> Shooter.setPower(0))
        );
    }
    private Command startShooting(){
        return parallel(
                instant(() -> ((DcMotorEx) Intake).setVelocity(intakeVelocity)),
                instant(this::transferFeed)
        );
    }
    private Command shooterOn() {
        return instant(() -> ((DcMotorEx) Shooter).setVelocity(shooterVelocity));
    }
    private void transferFeed() {
        // Feeding the shooter: encoder mode, velocity control
        if (Transfer.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        ((DcMotorEx) Transfer).setVelocity(transferVelocity);
    }

// ---------- Commands ----------


    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                // Shoot Pre-Load //
                shooterOn(),
                waitMs(1000),
                startShooting(),
                waitMs(1500),
                stopShooting(),
                // Shoot Pre-Load //
                
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");

        // Shooter //
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) Shooter).setVelocityPIDFCoefficients(100, 0, 0, 15);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setDirection(DcMotor.Direction.REVERSE);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Shooter //


        // Intake //
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) Intake).setVelocityPIDFCoefficients(30, 0, 0, 16);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Intake //


        // Transfer //
        Transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) Transfer).setVelocityPIDFCoefficients(30, 0, 0, 14);
        Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Transfer //

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("Shooter Velocity", ((DcMotorEx) Shooter).getVelocity());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}