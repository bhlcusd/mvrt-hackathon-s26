package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.subsystems.SubsystemBase;

public class Arm extends SubsystemBase<Arm.Command> {
	
	private static Arm instance;

	// Motor direction is based on looking at the bottom (logo) side
	// Two motors are used to rotate the entire Arm and Hand contraption
	private final Motor rotateLeftMotor;
	private final Motor rotateRightMotor;

	// One motor pulls the wide chain to extend/contract the Arm
	private final Motor extendMotor;

	public enum Command {
		DISABLED,
		IDLE,
		TRAVEL,
		MANUAL
	}

	// Represents the intended level the arm should extend/contract to
	private enum Travel {
		LEVEL_1,
		LEVEL_2,
		LEVEL_3
	}

	public static Arm getInstance() {
		if (instance == null) {
			instance = new Arm();
		}

		return instance;
	}

	private Arm() {
		super("Arm");

		// TODO: Add motor configs (refer to Elevator)
		MotorConfig leftConfig = new MotorConfig(LEFT_MOTOR_ID);
		MotorConfig rightConfig = new MotorConfig(RIGHT_MOTOR_ID);

		this.rotateLeftMotor = new Motor("Arm/rotateLeftMotor", leftConfig);
		this.rotateRightMotor = new Motor("Arm/rotateRightMotor", rightConfig);

		MotorConfig extendConfig = new MotorConfig(EXTEND_MOTOR_ID);

		this.extendMotor = new Motor("Arm/ExtendMotor", extendConfig);

		setCommand(Command.IDLE);
	}

	@Override
	protected void inputPeriodic() {
		rotateLeftMotor.readInputs();
		rotateRightMotor.readInputs();
		extendMotor.readInputs();
	}

	@Override
	protected void handle() {
		switch (getCommand()) {
			case DISABLED:
				rotateLeftMotor.stop();
				rotateRightMotor.stop();
				break;
			case IDLE:
				rotateLeftMotor.setVoltage(0);
				rotateRightMotor.setVoltage(0);
			case TRAVEL:
				break;
			case MANUAL:
				break;
			default:
				break;
		}
	}

	@Override
	protected void outputPeriodic() {
		// TODO: update 2d instances and call their periodics and log outputs
	}

	public void idle() {
		setCommand(Command.IDLE);
	}

	public void disable() {
		setCommand(Command.DISABLED);
	}

	public void manual(double volts) {
		// TODO: Implement later
	}

	public void atRotationTarget() {

	}

	public void atExtendTarget() {

	}
}
