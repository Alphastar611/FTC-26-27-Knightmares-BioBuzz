package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PoseStorage {
    // Default fallback to standard starting pose if TeleOp is run without Auto
    public static double teleOpX = -59.0;
    public static double teleOpY = 70.0;
    public static double teleOpHeadingDeg = -90.0;
    public static boolean hasAutoPose = false;
    
    private static final double HEADING_OFFSET_DEG = -90.0;
    public static void saveFromPedro(Pose pedroPose) {
        teleOpX = pedroPose.x() - 72.0;
        teleOpY = pedroPose.y() - 72.0;

        double rawDegrees = Math.toDegrees(pedroPose.heading());
        teleOpHeadingDeg = AngleUnit.DEGREES.normalize(rawDegrees + HEADING_OFFSET_DEG);

        hasAutoPose = true;
    }
    public static void clear() {
        hasAutoPose = false;
    }
}