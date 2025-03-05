package frc.robot.Subsystems;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    // THESE ARE DUMMY VALUES!!!!! TODO: #9 Update climber arm position values once determined.
    public enum Position {
        STARTING(0.0),
        LIFTED(0.0);

        private static final Map<Double, Position> lookup = new HashMap<Double, Position>();

        static {
            for (Position p : EnumSet.allOf(Position.class))
                lookup.put(p.getPosition(), p);
        }
        private double position;

        private Position(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }

        public static Position get(double position) {
            return lookup.get(position);
        }
    }

    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */
    public void setTargetPosition(double goal) {
    }

    /**
     * Set the goal of the pivot
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command setGoal(Position goal) {
        return run(() -> setTargetPosition(goal.getPosition()));
    }

}
