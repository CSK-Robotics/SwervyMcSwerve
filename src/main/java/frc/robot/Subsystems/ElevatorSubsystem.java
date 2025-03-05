package frc.robot.Subsystems;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    // THESE ARE DUMMY VALUES!!!!! TODO: #11 Update elevator position values once determined.
    public enum Position {
        ZERO(0.0),
        PROCESSOR(0.0),
        HPSTATION(0.0),
        L1(20.0),
        L2(40.0),
        L3(60.0),
        L4(80.0),
        BARGE(100.0);

        private static final Map<Double, Position> lookup = new HashMap<>();

        static {
            for (Position p : EnumSet.allOf(Position.class)) {
                lookup.put(p.getPosition(), p);
            }
        }
        private final double position;

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
     * Set the goal of the elevator
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command setGoal(Position goal) {
        return run(() -> setTargetPosition(goal.getPosition()));
    }
}
