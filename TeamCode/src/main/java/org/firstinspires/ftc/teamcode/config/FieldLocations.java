package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import java.util.Arrays;
import java.util.List;

/**
 * Single source of truth for field coordinate Poses across autonomous routines,
 * teleop driver assist, and path containers, constructed cleanly using PoseFactory.degrees().
 *
 * Supports C1 (tipped up hive side) and C6 (tipped down hive side) starting positions,
 * hive scoring targets, garden pickup, and left flower (A2/3) locations.
 *
 * <p><b>Field Diagram Reference:</b></p>
 * <p>
 * <img src="field_layout.png" alt="FTC Field Layout and Coordinates" width="500"/>
 * </p>
 * <p><i>Tip: Use Android Studio's "Rendered Documentation" view (hover over this comment
 * or press ctrl+Q) to view the rendered field image directly in the code editor!</i></p>
 */
public class FieldLocations {

    private static final PoseFactory pf = PoseFactory.degrees();

    // =========================================================================
    // HIVE FIELD LOCATIONS
    // =========================================================================
    public static final Pose RED_HIVE_C3 = pf.of(58.0,  53.0,  90.0);
    public static final Pose RED_HIVE_C4 = pf.of(58.0,  87.0, 270.0);

    public static final Pose BLUE_HIVE_D3 = pf.of(87.0,  14.0,  90.0);
    public static final Pose BLUE_HIVE_D4 = pf.of(87.0, 130.0, 270.0);

    public static final List<Pose> RED_HIVES  = Arrays.asList(RED_HIVE_C3,   RED_HIVE_C4);
    public static final List<Pose> BLUE_HIVES = Arrays.asList(BLUE_HIVE_D3, BLUE_HIVE_D4);

    // =========================================================================
    // AUTONOMOUS ROUTINE POSES (AutoPath1 / PP_C1_HiveUp_AutoPath)
    // =========================================================================
    public static final Pose START_C1              = pf.of(58.0,  8.5,  90.0);
    public static final Pose SCORE_C1              = pf.of(58.0, 14.0,  90.0);
    public static final Pose RED_GARDEN_APPROACH   = pf.of(24.0,  9.5, 180.0);
    public static final Pose RED_GARDEN_PICKUP     = pf.of( 9.0,  9.0, 180.0);
    public static final Pose FLOWER_A2_3           = pf.of(12.0, 42.0, 180.0);
    public static final Pose RED_PARK_A4           = pf.of(10.0, 90.0,  90.0);

    // =========================================================================
    // BEZIER PATH CONTROL POINTS
    // =========================================================================
    public static final Pose SCORE_TO_RED_GARDEN_CONTROL_1  = pf.of(58.0, 21.0, 0.0);
    public static final Pose SCORE_TO_RED_GARDEN_CONTROL_2  = pf.of(37.0,  9.5, 0.0);
    public static final Pose RED_GARDEN_TO_SCORE_CONTROL_1  = pf.of(20.0, 15.5, 0.0);
    public static final Pose SCORE_TO_FLOWER_A23_CONTROL_1  = pf.of(57.0, 30.0, 0.0);
    public static final Pose SCORE_TO_FLOWER_A23_CONTROL_2  = pf.of(25.0, 42.0, 0.0);
    public static final Pose FLOWER_A23_TO_SCORE_CONTROL_1  = pf.of(25.5, 42.0, 0.0);
    public static final Pose FLOWER_A23_TO_SCORE_CONTROL_2  = pf.of(58.0, 30.0, 0.0);
    public static final Pose SCORE_C1_TO_RED_PARK_CONTROL_1 = pf.of(58.0, 28.0, 0.0);
    public static final Pose SCORE_C1_TO_RED_PARK_CONTROL_2 = pf.of(10.0, 40.0, 0.0);

    /**
     * Gets list of Hive target poses for the specified alliance.
     *
     * @param alliance RED or BLUE alliance.
     * @return List of {@link Pose} objects representing Hive locations.
     */
    public static List<Pose> getHivesForAlliance(GameState.Alliance alliance) {
        return (alliance == GameState.Alliance.RED) ? RED_HIVES : BLUE_HIVES;
    }

    /**
     * Finds the nearest Hive target pose for the given alliance relative to a robot pose.
     *
     * @param robotPose Current pose of the robot.
     * @param alliance  Active alliance.
     * @return Closest {@link Pose} representing a Hive.
     */
    public static Pose getNearestHive(Pose robotPose, GameState.Alliance alliance) {
        List<Pose> candidateHives = getHivesForAlliance(alliance);
        Pose nearest = null;
        double minDistance = Double.MAX_VALUE;

        for (Pose hive : candidateHives) {
            double dx = hive.x() - robotPose.x();
            double dy = hive.y() - robotPose.y();
            double distance = Math.hypot(dx, dy);

            if (distance < minDistance) {
                minDistance = distance;
                nearest = hive;
            }
        }

        return nearest != null ? nearest : candidateHives.get(0);
    }
}
