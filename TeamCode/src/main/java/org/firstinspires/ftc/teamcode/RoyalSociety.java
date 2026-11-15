package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;


@Autonomous(name = "RoyalSociety", group = "Royal Society")
public class RoyalSociety extends LinearOpMode {

    final int shooterVelocity = 1520;
    final int intakeVelocity = 2000;
    final int transferVelocity = 2800;
    private Follower follower;
    private DcMotor Shooter;
    private DcMotor Intake;
    private DcMotor Transfer;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.2933, 134.6523, 90);
    private final Pose idle = poseFactory.of(34.1757, 58.6448, 270);
    private final Pose idleControl1 = poseFactory.of(65.0732, 112.7666, 0);
    private final Pose idleControl2 = poseFactory.of(24.0601, 138.9629, 0);
    private final Pose shootStart = poseFactory.of(34.1757, 58.6448, 270);
    private final Pose shoot = poseFactory.of(33.9488, 12.1605, 60);
    private final Pose park = poseFactory.of(12.5409, 92.8462, 90);

    public Path halt() {
        return Paths.curve(start, idleControl1, idleControl2, idle).constant(idle);
    }

    public Path shoot() {
        return Paths.line(shootStart, shoot).linear(shootStart, shoot);
    }

    public Path park() {
        return Paths.line(shoot, park).constant(park);
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

            // Get out of way //
                follow(follower, halt()),
                waitMs(7500),
                shooterOn(),
            // Get out of way //

            // Shoot Pre-load //
                follow(follower, shoot()),
                waitMs(5000),
                startShooting(),
                waitMs(1500),
                stopShooting(),
            // Shoot Pre-load //

            // Park //
                follow(follower, park())
            // Park //

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
        Servo flowerMechanism = hardwareMap.get(Servo.class, "Flower Mechanism");
        Servo turret = hardwareMap.get(Servo.class, "Turret");

        // Shooter //
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) Shooter).setVelocityPIDFCoefficients(50, 0, 0, 15);
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

        turret.setPosition(0.507);

        flowerMechanism.setPosition(0);

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