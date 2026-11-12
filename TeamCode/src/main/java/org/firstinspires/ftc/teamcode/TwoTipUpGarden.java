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
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;


@Autonomous(name = "TwoTipUpGarden", group = "Goal Up")
public class TwoTipUpGarden extends LinearOpMode {

    private final ElapsedTime pathTimer = new ElapsedTime();
    int shooterVelocity = 1520;
    final int intakeVelocity = 2000;
    final int transferVelocity = 2800;
    final double flowerUp = 0;
    final double flowerDown = 0.425;
    private Follower follower;
    private DcMotor Shooter;
    private DcMotor Intake;
    private DcMotor Transfer;
    private Servo FlowerMechanism;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(53.8534, 8.3578, 90);
    private final Pose shootPreload = poseFactory.of(34, 14, 70);
    private final Pose gardenBufferStart = poseFactory.of(34, 12, 70);
    private final Pose gardenBuffer = poseFactory.of(16, 20, 270);
    private final Pose gardenCollect = poseFactory.of(6, 6, 270);
    private final Pose score2 = poseFactory.of(34, 130, -55);
    private final Pose score2Control1 = poseFactory.of(20.2246, 29.0205, 0);
    private final Pose score2Segment1Heading = poseFactory.of(34, 130, 270);
    private final Pose score2Segment2Start = poseFactory.of(34, 130, -90);
    private final Pose score2Segment2End = poseFactory.of(34, 130, 305);
    private final Pose flower1BufferStart = poseFactory.of(34, 130, 305);
    private final Pose flower1Buffer = poseFactory.of(24, 50, 180);
    private final Pose flower1Collect = poseFactory.of(5, 50, 180);
    private final Pose flower1Shuffle = poseFactory.of(24, 50, 180);
    private final Pose flower1Collect2 = poseFactory.of(5, 50, 180);
    private final Pose score3 = poseFactory.of(34, 130, -55);
    private final Pose score3Control1 = poseFactory.of(38.093, 69.429, 0);
    private final Pose score3Segment1Start = poseFactory.of(34, 130, 180);
    private final Pose score3Segment1End = poseFactory.of(34, 130, 210);
    private final Pose score3Segment3Start = poseFactory.of(34, 130, -93.4927);
    private final Pose score3Segment3End = poseFactory.of(34, 130, 305);
    private final Pose flower2BufferStart = poseFactory.of(34, 130, 305);
    private final Pose flower2Buffer = poseFactory.of(46.75, 120, 90);
    private final Pose flower2Collect = poseFactory.of(46.75, 134, 90);
    private final Pose flower2Shuffle = poseFactory.of(46.75, 120, 90);
    private final Pose flower2Collect2 = poseFactory.of(46.75, 134, 90);
    private final Pose shoot4Start = poseFactory.of(46.75, 134, 90);
    private final Pose shoot4 = poseFactory.of(34, 130, 305);
    private final Pose shoot4Control1 = poseFactory.of(39.4855, 117.5357, 0);
    private final Pose parkStart = poseFactory.of(34, 130, 305);
    private final Pose park = poseFactory.of(12, 120, 270);

    public Path shootPreload() {
        return Paths.line(start, shootPreload).linear(start, shootPreload);
    }

    public Path gardenBuffer() {
        return Paths.line(gardenBufferStart, gardenBuffer).linear(gardenBufferStart, gardenBuffer);
    }

    public Path gardenCollect() {
        return Paths.line(gardenBuffer, gardenCollect).constant(gardenCollect);
    }

    public Path score2() {
        return Paths.curve(gardenCollect, score2Control1, score2).heading(Interpolator.piecewise().until(0.5, Interpolator.constant(score2Segment1Heading)).until(1, Interpolator.linear(score2Segment2Start, score2Segment2End)));
    }

    public Path flower1Buffer() {
        return Paths.line(flower1BufferStart, flower1Buffer).linear(flower1BufferStart, flower1Buffer);
    }

    public Path flower1Collect() {
        return Paths.line(flower1Buffer, flower1Collect).constant(flower1Collect);
    }

    public Path flower1Shuffle() {
        return Paths.line(flower1Collect, flower1Shuffle).constant(flower1Shuffle);
    }

    public Path flower1Collect2() {
        return Paths.line(flower1Shuffle, flower1Collect2).constant(flower1Collect2);
    }

    public Path score3() {
        return Paths.curve(flower1Collect2, score3Control1, score3).heading(Interpolator.piecewise().until(0.125, Interpolator.linear(score3Segment1Start, score3Segment1End)).until(0.75, Interpolator.tangent.reverse()).until(1, Interpolator.linear(score3Segment3Start, score3Segment3End)));
    }

    public Path flower2Buffer() {
        return Paths.line(flower2BufferStart, flower2Buffer).linear(flower2BufferStart, flower2Buffer);
    }

    public Path flower2Collect() {
        return Paths.line(flower2Buffer, flower2Collect).constant(flower2Collect);
    }

    public Path flower2Shuffle() {
        return Paths.line(flower2Collect, flower2Shuffle).constant(flower2Shuffle);
    }

    public Path flower2Collect2() {
        return Paths.line(flower2Shuffle, flower2Collect2).constant(flower2Collect2);
    }

    public Path shoot4() {
        return Paths.curve(shoot4Start, shoot4Control1, shoot4).linear(shoot4Start, shoot4);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
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

// ---------- Commands ----------

    private Command intakeOn() {
        return instant(() -> {
            ((DcMotorEx) Intake).setVelocity(intakeVelocity);
            transferHoldBack();
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
                follow(follower, shootPreload()),
                waitMs(750),
                startShooting(),
                waitMs(1250),
                stopShooting(),
            // Shoot Pre-Load //

            // Drive to garden //
                follow(follower, gardenBuffer()),
                intakeOn(),
            // Drive to garden //

            // Grab from garden //
                followWithTimeout(gardenCollect(), 1),
                waitMs(250),
                instant(() -> shooterVelocity = 1520),
                shooterOn(),
            // Grab from garden //

            // Shoot for the second time //
                follow(follower, score2()),
                waitMs(250),
                startShooting(),
                waitMs(1000),
                stopShooting(),
            // Shoot for the second time //

            // Drive to the first flower //
                follow(follower, flower1Buffer()),
                flowerDown(),
                intakeOn(),
                waitMs(250),
            // Drive to the first flower //

            // Grab out of the flower //
                followWithTimeout(flower1Collect(),0.5),
                waitMs(250),
                follow(follower,flower1Shuffle()),
                followWithTimeout(flower1Collect2(),0.5),
                waitMs(250),
                shooterOn(),
            // Grab out of the flower //

            // Score for the third time //
                follow(follower,score3()),
                flowerUp(),
                waitMs(250),
                startShooting(),
                waitMs(1000),
                stopShooting(),
            // Score for the third time //

            // Drive to the second flower //
                follow(follower, flower2Buffer()),
                flowerDown(),
                waitMs(250),
                intakeOn(),
            // Drive to the second flower //

            // Grab out of the second flower //
                followWithTimeout(flower2Collect(),0.5),
                waitMs(250),
                follow(follower, flower2Shuffle()),
                followWithTimeout(flower2Collect2(),0.5),
                waitMs(250),
                shooterOn(),
            // Grab out of the second flower //

            // Score for the fourth time //
                follow(follower, shoot4()),
                waitMs(250),
                flowerUp(),
                startShooting(),
                waitMs(1000),
                stopShooting(),
            // Score for the fourth time //

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
        FlowerMechanism = hardwareMap.get(Servo.class, "Flower Mechanism");
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

        FlowerMechanism.setPosition(0);

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