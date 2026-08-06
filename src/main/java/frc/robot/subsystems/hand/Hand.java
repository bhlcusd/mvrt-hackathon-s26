package frc.robot.subsystems.hand;

import static frc.robot.subsystems.hand.HandConstants.*;

import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.subsystems.SubsystemBase;

public class Hand extends SubsystemBase<Hand.Command> {
	public enum Command {
		DISABLED,
		IDLE
	}

	private static Hand instance;

	private Motor motor;

	public static Hand getInstance() {
		if (instance == null) {
			instance = new Hand();
		}

		return instance;
	}

	private Hand() {
		super("Hand");

		// TODO: Set motor config
		MotorConfig config = new MotorConfig(MOTOR_ID);

		this.motor = new Motor("Hand/Motor", config);
	}

	@Override
	protected void inputPeriodic() {
		motor.readInputs();
	}

	@Override
	protected void handle() {
		switch (getCommand()) {
			case DISABLED:
				motor.stop();
				break;
			case IDLE:
				motor.setVoltage(0);
				break;
			default:
				break;
		}
	}

	@Override
	protected void outputPeriodic() {
		// TODO: Update 2d with new inputs from motor, call their periodic, and log info
	}
}
