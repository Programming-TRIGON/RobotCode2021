package frc.robot.components;

public class TalonFXWithTalonSRXEncoder extends TrigonTalonFX {
    private TrigonTalonSRX srx;

    public TalonFXWithTalonSRXEncoder(int id, int encoderId) {
        super(id);
        srx = new TrigonTalonSRX(encoderId);
    }

    @Override
    public double getSelectedSensorPosition() {
        return srx.getSelectedSensorPosition();
    }
}
