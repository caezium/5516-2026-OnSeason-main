package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public final class ClimbIOReal implements ClimbIO {
    private final TalonFX climbMotor;
    private final StatusSignal<Angle> climbPosition;
    private final StatusSignal<Current> climbCurrent;

    public ClimbIOReal() {
        this.climbMotor = new TalonFX(13);

        climbMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        climbMotor
                .getConfigurator()
                .apply(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40)
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(60));
        // This mechanism has no external CANcoder, so use TalonFX integrated position signal.
        climbPosition = climbMotor.getPosition();
        climbCurrent = climbMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, climbPosition, climbCurrent);
    }

    private final VoltageOut voltageOut = new VoltageOut(0);

    @Override
    public void updateInputs(ClimbInputs inputs) {
        StatusCode statusCode = BaseStatusSignal.refreshAll(climbPosition, climbCurrent);
        inputs.hardwareConnected = statusCode.isOK();

        inputs.climbAbsolutePosition = climbPosition.getValueAsDouble();
        inputs.climbCurrentAmps = climbCurrent.getValueAsDouble();
    }

    @Override
    public void setMotorOutput(double volts) {
        climbMotor.setControl(voltageOut.withOutput(volts));
    }
}
