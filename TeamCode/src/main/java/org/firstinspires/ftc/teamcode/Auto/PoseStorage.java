package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PoseStorage {
    public static double teleOpX = -59.0;
    public static double teleOpY = 70.0;
    public static double teleOpHeadingDeg = -90.0;
    public static boolean hasAutoPose = false;

    public static void saveFromPedro(Pose pedroPose) {
        teleOpX = pedroPose.x() - 72.0;
        teleOpY = pedroPose.y() - 72.0;
        teleOpHeadingDeg = AngleUnit.DEGREES.normalize(
                Math.toDegrees(pedroPose.heading())
        );
        hasAutoPose = true;
    }
}