package frc.lib.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Implementation for Pigeon2 IMU IO */
public class Pigeon2IMUIO implements IMUIO {
    private final Pigeon2 pigeon;
    private final StatusSignal<Double> yaw;
    private final StatusSignal<Double> yawVelocity;

    /**
     * Constructs a new Pigeon2IMUIO.
     * @param id The CAN ID of the Pigeon2.
     */
    public Pigeon2IMUIO(int id) {
        pigeon = new Pigeon2(id);
        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(100.0);
        yawVelocity.setUpdateFrequency(100.0);
        pigeon.optimizeBusUtilization();
    }

    /**
     * Updates the system with new inputs from the IMU.
     * @param inputs The IMU inputs.
     */
    @Override
    public void updateInputs(IMUInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocity = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    }
}