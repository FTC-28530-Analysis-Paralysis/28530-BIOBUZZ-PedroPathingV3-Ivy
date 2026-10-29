package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.math.Pose;

/**
 * Global game state configuration including Alliance selection and target element preferences.
 */
public class GameState {
    public static Pose autonomousEndPose = new Pose(0, 0, 0);
    public enum Alliance {
        RED("Red Alliance"),
        BLUE("Blue Alliance");

        private final String name;
        Alliance(String name) { this.name = name; }
        public String getName() { return name; }
    }

    public enum TargetElement {
        YELLOW_POLLEN("Yellow Pollen (Neutral)"),
        ALLIANCE_NECTAR("Alliance Nectar (Match Color)"),
        RED_NECTAR("Red Nectar"),
        BLUE_NECTAR("Blue Nectar");

        private final String description;
        TargetElement(String description) { this.description = description; }
        public String getDescription() { return description; }
    }

    public static Alliance alliance = Alliance.BLUE;
    public static TargetElement targetElement = TargetElement.YELLOW_POLLEN;

    public static TargetElement getEffectiveTargetElement() {
        if (targetElement == TargetElement.ALLIANCE_NECTAR) {
            return alliance == Alliance.RED ? TargetElement.RED_NECTAR : TargetElement.BLUE_NECTAR;
        }
        return targetElement;
    }
}
