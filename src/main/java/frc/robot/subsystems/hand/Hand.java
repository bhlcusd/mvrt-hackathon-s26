package frc.robot.subsystems.hand;

import static frc.robot.subsystems.hand.HandConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.util.Util;

public class Hand extends SubsystemBase<Hand.Command> {
	public enum Command {
		DISABLE,
		IDLE,
		MANUAL,
		TRAVEL
	}

	private enum Travel {
		MOVING,
		HOLDING
	}

	private static Hand instance;

	private Motor motor;

	private final Hand2d hand2d = new Hand2d("Hand", new Color8Bit(255, 0, 0));

	private double targetAngle_deg;
	private double targetVolts_v;

	public static Hand getInstance() {
		if (instance == null) {
			instance = new Hand();
		}

		return instance;
	}

	private Hand() {
		super("Hand");

		MotorConfig config = new MotorConfig(MOTOR_ID)
			.withCanbus(CANBUS)
			.withBrake(BRAKE)
			.withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO)
			.withFFGains(kS, kV, kA, kG)
			.withPIDGains(kP, kI, kD, GRAVITY_TYPE)
			.withMotionMagic(CRUISE_VELOCITY_rps, ACCELERATION_rps2, JERK_rps3)
			.withSim(MOTOR, SENSOR_TO_MECHANISM_RATIO, SIM_MOI_kgm2);

		this.motor = new Motor("Hand/Motor", config);

		setCommand(Command.IDLE);
	}

	@Override
	protected void inputPeriodic() {
		motor.readInputs();
	}

	@Override
	protected void handle() {
		switch (getCommand()) {
			case DISABLE:
				motor.stop();
				break;
			case IDLE:
				motor.setVoltage(0);
				break;
			case MANUAL:
				motor.setVoltage(targetVolts_v);
				break;
			case TRAVEL:
				if (firstLoop()) {
					setSubstate(Travel.MOVING);
				}

				motor.setMotionMagic(Units.degreesToRotations(targetAngle_deg));

				switch ((Travel) getSubstate()) {
					case MOVING:
						if (atAngleTarget()) {
							setSubstate(Travel.HOLDING);
						}
						break;
					case HOLDING:
						if (!atAngleTarget()) {
							setSubstate(Travel.MOVING);
						}
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
	}

	@Override
	protected void outputPeriodic() {
		hand2d.update(getAngle());
		hand2d.periodic();

		Logger.recordOutput("Hand/Angle_deg", getAngle());
		Logger.recordOutput("Hand/Velocity_dps", getVelocity());
		Logger.recordOutput("Hand/TargetAngle_deg", targetAngle_deg); 
	}

	public void disable() {
		setCommand(Command.DISABLE);
	}

	public void idle() {
		setCommand(Command.IDLE);
	}

	public void manual(double volts) {
		this.targetVolts_v = MathUtil.clamp(volts, -12, 12);
		setCommand(Command.MANUAL);
	}

	public void intake(double delta_deg) {
		this.targetAngle_deg = getAngle() + delta_deg;
		setCommand(Command.TRAVEL);
		setSubstate(Travel.MOVING); 
	}

	public void expel(double delta_deg) {
		this.targetAngle_deg = getAngle() - delta_deg;
		setCommand(Command.TRAVEL);
		setSubstate(Travel.MOVING);
	}

	public void zero() {
		this.targetAngle_deg = ZERO_deg;
		setCommand(Command.TRAVEL);
	}

	public double getAngle() {
		return Units.rotationsToDegrees(motor.getPosition());
	}

	public double getVelocity() {
		return Units.rotationsToDegrees(motor.getVelocity());
	}

	public boolean atAngleTarget() {
		return Util.inRange(targetAngle_deg - getAngle(), TOLERANCE_deg);
	}
}
