package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    private static final double MANUAL_UP_VOLTAGE = 10.5;
    private static final double MANUAL_DOWN_VOLTAGE = -10.5;

    private final ClimbIO io;
    private final ClimbInputsAutoLogged inputs;

    public Climb(ClimbIO io) {
        this.io = io;
        io.setMotorOutput(0.0);
        inputs = new ClimbInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    /** Stops the climb motor output immediately. */
    public void stop() {
        io.setMotorOutput(0.0);
    }

    // Pov Up
    public Command manualUpCommand() {
        return run(() -> {
                    if (inputs.hardwareConnected) {
                        io.setMotorOutput(MANUAL_UP_VOLTAGE);
                    } else {
                        io.setMotorOutput(0.0);
                    }
                })
                .finallyDo(this::stop);
    }

    // Pov Down
    public Command manualDownCommand() {
        return run(() -> {
                    if (inputs.hardwareConnected) {
                        io.setMotorOutput(MANUAL_DOWN_VOLTAGE);
                    } else {
                        io.setMotorOutput(0.0);
                    }
                })
                .finallyDo(this::stop);
    }
}
