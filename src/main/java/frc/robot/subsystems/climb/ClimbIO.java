package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    class ClimbInputs {
        /** Whether the climb hardware communication is healthy in this cycle. */
        boolean hardwareConnected = false;
        /** Supply current of climb motor in amps. */
        double climbCurrentAmps = 0.0;
        /** Position feedback of climb motor (in rotations). */
        double climbAbsolutePosition = 0.0;
    }

    void updateInputs(ClimbInputs inputs);

    /** Sets climb motor output voltage. Positive/negative direction depends on mechanism wiring. */
    default void setMotorOutput(double volts) {}
}
