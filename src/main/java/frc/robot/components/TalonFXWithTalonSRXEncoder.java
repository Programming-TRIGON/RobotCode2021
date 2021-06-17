package frc.robot.components;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class TalonFXWithTalonSRXEncoder extends TrigonTalonFX {
    private TrigonTalonSRX srx;

    public TalonFXWithTalonSRXEncoder(int id, TrigonTalonSRX encoderSrx, MotorConfig config) {
        super(id, config);
        srx = encoderSrx;
        srx.setSensorPhase(config.isSensorInverted());
    }

    public TalonFXWithTalonSRXEncoder(int id, int encoderId, MotorConfig config) {
        this(id, new TrigonTalonSRX(encoderId), config);
    }

    @Override
    public double getSelectedSensorPosition() {
        return srx.getSelectedSensorPosition();
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice) {
        return srx.configSelectedFeedbackSensor(feedbackDevice);
    }

    @Override
    public ErrorCode configFeedbackNotContinuous(boolean feedbackNotContinuous, int timeoutMs) {
        return srx.configFeedbackNotContinuous(feedbackNotContinuous, timeoutMs);
    }
}
