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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.ivy.commands.Commands.*;   // instant, waitMs, waitUntil, infinite

import org.firstinspires.ftc.teamcode.pedro.Constants;



@Autonomous(name = "AutoPath", group = "Autonomous")
public class GoalUp extends LinearOpMode {

    private final ElapsedTime pathTimer = new ElapsedTime();
    final int shooterVelocity = 1500;
    final int intakeVelocity = 2400;
    final int transferVelocity = 2800;
    final double flowerUp = 0.025;
    final double flowerDown = 0.3875;
    private Follower follower;
    private DcMotor Shooter;
    private DcMotor Intake;
    private DcMotor Transfer;
    private Servo FlowerMechanism;
    private Servo Turret;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.9267, 8.1789, 90);
    private final Pose scoreToFlower1 = poseFactory.of(22.7478, 46.7478, 180);
    private final Pose scoreToFlower1Control1 = poseFactory.of(46.5365, 37.608, 0);
    private final Pose flowerGrab1Start = poseFactory.of(22.7478, 46.7478, 180);
    private final Pose flowerGrab1 = poseFactory.of(12, 46.75, 180);
    private final Pose score2Start = poseFactory.of(12, 46.75, 180);
    private final Pose score2 = poseFactory.of(60, 130, 270);
    private final Pose score2Control1 = poseFactory.of(50.0206, 73.8511, 0);
    private final Pose score2Control2 = poseFactory.of(0, 95.5562, 0);
    private final Pose score2Control3 = poseFactory.of(54.18, 109.1717, 0);
    private final Pose scoreToFlower2HesiStart = poseFactory.of(60, 130, 270);
    private final Pose scoreToFlower2Hesi = poseFactory.of(54.9788, 112.129, 90);
    private final Pose scoreToFlower2 = poseFactory.of(46.75, 120, 90);
    private final Pose flowerGrab2 = poseFactory.of(46.75, 128, 90);
    private final Pose score3HesiStart = poseFactory.of(46.75, 128, 90);
    private final Pose score3Hesi = poseFactory.of(53.2023, 113.7212, 270);
    private final Pose score3 = poseFactory.of(60, 130, 270);
    private final Pose parkStart = poseFactory.of(60, 130, 270);
    private final Pose park = poseFactory.of(11.5422, 123.7484, 180);
    private final Pose parkControl1 = poseFactory.of(48.7288, 107.878, 0);
    public Path scoreToFlower1() {
        return Paths.curve(start, scoreToFlower1Control1, scoreToFlower1).linear(start, scoreToFlower1);
    }

    public Path flowerGrab1() {
        return Paths.line(flowerGrab1Start, flowerGrab1).linear(flowerGrab1Start, flowerGrab1);
    }

    public Path score2() {
        return Paths.curve(score2Start, score2Control1, score2Control2, score2Control3, score2).linear(score2Start, score2);
    }

    public Path scoreToFlower2Hesi() {
        return Paths.line(scoreToFlower2HesiStart, scoreToFlower2Hesi).linear(scoreToFlower2HesiStart, scoreToFlower2Hesi);
    }

    public Path scoreToFlower2() {
        return Paths.line(scoreToFlower2Hesi, scoreToFlower2).constant(scoreToFlower2);
    }

    public Path flowerGrab2() {
        return Paths.line(scoreToFlower2, flowerGrab2).constant(flowerGrab2);
    }

    public Path score3Hesi() {
        return Paths.line(score3HesiStart, score3Hesi).linear(score3HesiStart, score3Hesi);
    }

    public Path score3() {
        return Paths.line(score3Hesi, score3).constant(score3);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
    private Command followWithTimeout(Path path, double timeoutSec) {
        return sequential(
                instant(pathTimer::reset),
                follow(follower, path).until(() -> pathTimer.seconds() > timeoutSec)
        );
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

    private void transferHoldBack() {
        Transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Transfer.setPower(-0.2);
    }

    private void transferFeed() {
        // Feeding the shooter: encoder mode, velocity control
        if (Transfer.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        ((DcMotorEx) Transfer).setVelocity(transferVelocity);
    }

    private void transferStop() {
        Transfer.setPower(0);
    }

// ---------- Commands ----------

    private Command intakeOn() {
        return instant(() -> {
            ((DcMotorEx) Intake).setVelocity(intakeVelocity);
            transferHoldBack();
        });
    }

    private Command intakeOff() {
        return instant(() -> {
            Intake.setPower(0);
            transferStop();
        });
    }
    private Command flowerDown(){
        return instant(() -> FlowerMechanism.setPosition(flowerDown));
    }
    private Command flowerUp(){
        return instant(() -> FlowerMechanism.setPosition(flowerUp));
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
            // Shoot Pre-Load //
                shooterOn(),
                waitMs(500),
                startShooting(),
                waitMs(1500),
                stopShooting(),
            // Shoot Pre-Load //

            // Drive to the first flower //
                follow(follower, scoreToFlower1()),
                flowerDown(),
                intakeOn(),
                waitMs(500),
            // Drive to the first flower //

            // Grab out of the flower //
                followWithTimeout(flowerGrab1(),2),
                waitMs(1000),
                intakeOff(),
                shooterOn(),
            // Grab out of the flower //

            // Shoot for the second time //
                follow(follower, score2()),
                flowerUp(),
                waitMs(500),
                startShooting(),
                waitMs(1500),
                stopShooting(),
            // Shoot for the second time //

                follow(follower, scoreToFlower2Hesi()),

            // Drive to the second flower //
                follow(follower, scoreToFlower2()),
                flowerDown(),
                intakeOn(),
            // Drive to the second flower //

            // Grab out of the flower //
                followWithTimeout(flowerGrab2(), 2),
                waitMs(1000),
                shooterOn(),
                intakeOff(),
            // Grab out of the flower //


                follow(follower, score3Hesi()),
                flowerUp(),

            // Score for the third time //
                follow(follower, score3()),
                waitMs(500),
                startShooting(),
                waitMs(1500),
                stopShooting(),
            // Score for the third time //

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
        FlowerMechanism = hardwareMap.get(Servo.class, "FlowerMechanism");
        Turret = hardwareMap.get(Servo.class, "Turret");

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

        Turret.setPosition(0.5);

        FlowerMechanism.setPosition(0.025);

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