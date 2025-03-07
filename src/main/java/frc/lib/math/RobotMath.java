package frc.lib.math;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ElevatorConstants;

public class RobotMath {

    public static class Elevator {

        /**
         * Convert {@link Distance} into {@link Angle}
         *
         * @param distance Distance, usually Meters.
         * @return {@link Angle} equivalent to rotations of the motor.
         */
        public static Angle convertDistanceToRotations(Distance distance) {
            return Rotations.of(distance.in(Meters) /
                    (ElevatorConstants.kElevatorConfig.kDrumRadius * 2 * Math.PI) *
                    ElevatorConstants.kElevatorConfig.kMotorConfig.kSimulation.kGearboxRatio);
        }

        /**
         * Convert {@link Angle} into {@link Distance}
         *
         * @param rotations Rotations of the motor
         * @return {@link Distance} of the elevator.
         */
        public static Distance convertRotationsToDistance(Angle rotations) {
            return Meters.of((rotations.in(Rotations) / ElevatorConstants.kElevatorConfig.kMotorConfig.kSimulation.kGearboxRatio) *
                    (ElevatorConstants.kElevatorConfig.kDrumRadius * 2 * Math.PI));
        }

    }
}
