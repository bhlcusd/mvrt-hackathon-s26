package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color8Bit;
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

	private Arm2d arm2d = new Arm2d("Arm", new Color8Bit(0, 0, 255));

	private double targetLength_m;
	private double targetAngle_deg;
	private double targetExtendVolts_v;
	private double targetRotateVolts_v;

	public enum Command {
		DISABLED,
		IDLE,
		TRAVEL,
		MANUAL
	}

	private enum Travel {
		MOVING,
		HOLDING
	}

	public static Arm getInstance() {
		if (instance == null) {
			instance = new Arm();
		}

		return instance;
	}

	private Arm() {
		super("Arm");

		this.targetAngle_deg = ArmConstants.STOW_ANGLE_deg;
		this.targetLength_m = ArmConstants.MIN_LENGTH_m;
		this.targetRotateVolts_v = 0;
		this.targetExtendVolts_v = 0;

		MotorConfig leftConfig = new MotorConfig(LEFT_MOTOR_ID)
			.withCanbus(CANBUS)
			.withBrake(BRAKE)
			.withSensorToMechanismRatio(ROTATE_GEAR_RATIO)
			.withFFGains(kRotateS, kRotateV, kRotateA, kRotateG)
			.withPIDGains(kRotateP, kRotateI, kRotateD, ROTATE_GRAVITY_TYPE)
			.withMotionMagic(ROTATE_CRUISE_VELOCITY_rps, ROTATE_ACCELERATION_rps2, ROTATE_JERK_rps3)
			.withSim(ROTATE_SIM_MOTORS, ROTATE_GEAR_RATIO, SIM_MOI_kgm2);
		
		MotorConfig rightConfig = new MotorConfig(RIGHT_MOTOR_ID)
			.withCanbus(CANBUS)
			.withBrake(BRAKE)
			.withSensorToMechanismRatio(ROTATE_GEAR_RATIO)
			.withFFGains(kRotateS, kRotateV, kRotateA, kRotateG)
			.withPIDGains(kRotateP, kRotateI, kRotateD, ROTATE_GRAVITY_TYPE)
			.withMotionMagic(ROTATE_CRUISE_VELOCITY_rps, ROTATE_ACCELERATION_rps2, ROTATE_JERK_rps3)
			.withSim(ROTATE_SIM_MOTORS, ROTATE_GEAR_RATIO, SIM_MOI_kgm2);

		this.rotateLeftMotor = new Motor("Arm/rotateLeftMotor", leftConfig);
		this.rotateRightMotor = new Motor("Arm/rotateRightMotor", rightConfig);

		MotorConfig extendConfig = new MotorConfig(EXTEND_MOTOR_ID)
			.withCanbus(CANBUS)
			.withBrake(BRAKE)
			.withSensorToMechanismRatio(EXTEND_METERS_TO_ROTATIONS)
			.withFFGains(kExtendS, kExtendV, kExtendA, kExtendG)
			.withPIDGains(kExtendP, kExtendI, kExtendD, EXTEND_GRAVITY_TYPE)
			.withMotionMagic(EXTEND_CRUISE_VELOCITY_mps, EXTEND_ACCELERATION_mps2, EXTEND_JERK_mps3)
			.withSim(EXTEND_SIM_MOTOR, EXTEND_METERS_TO_ROTATIONS, SIM_MOI_kgm2);

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
				extendMotor.stop();
				break;
			case IDLE:
				rotateLeftMotor.setVoltage(0);
				rotateRightMotor.setVoltage(0);
				extendMotor.setVoltage(0);
				break;
			case TRAVEL:
				if (firstLoop()) {
					setSubstate(Travel.MOVING);
				}

				// This could be improved upon to be more granular in the substate (especially to see what MIGHT be holding up MOVING)
				// but that would mean that we would:
				//	1. Have multiple substate enums to make it rotate and extend simultaniously (but how would we know what substate it's in without logic in Arm?)
				//	2. Add a ROTATE and EXTEND substates (but that would make it rotate or extend, not both)
				rotateLeftMotor.setMotionMagic(targetAngle_deg);
				rotateRightMotor.setMotionMagic(targetAngle_deg);
				// extendMotor.setMotionMagic(targetLength_m);
				extendMotor.setMotionMagic(1);

				switch ((Travel) getSubstate()) {
					case MOVING:
						if (atRotationTarget() && atExtendTarget()) {
							setSubstate(Travel.HOLDING);
						}
						break;
					case HOLDING:
						if (!atRotationTarget() || !atExtendTarget()) {
							setSubstate(Travel.MOVING);
						}
						break;
					default:
						break;
				}
				break;
			case MANUAL:
				rotateLeftMotor.setVoltage(targetRotateVolts_v);
				rotateRightMotor.setVoltage(targetRotateVolts_v);
				extendMotor.setVoltage(targetExtendVolts_v);
				break;
			default:
				break;
		}
	}

	@Override
	protected void outputPeriodic() {
		arm2d.setLength(getLength());
		arm2d.setAngle(getAngle());

		arm2d.periodic();

		Logger.recordOutput("Arm/Length_m", getLength());
		Logger.recordOutput("Arm/LengthVelocity_mps", extendMotor.getVelocity());
		Logger.recordOutput("Arm/TargetLength_m", targetLength_m);
		Logger.recordOutput("Arm/Angle_deg", getAngle());
		Logger.recordOutput("Arm/AngleVelocity_degps", getAngleVelocity());
		Logger.recordOutput("Arm/TargetAngle_deg", targetAngle_deg);
	}

	public void disable() {
		setCommand(Command.DISABLED);
	}

	public void idle() {
		setCommand(Command.IDLE);
	}

	public void stow() {
		moveTo(ArmConstants.MIN_LENGTH_m, ArmConstants.STOW_ANGLE_deg);
	}

	public void moveTo(double length, double angle) {
		this.targetLength_m = length;
		this.targetAngle_deg = angle;
		setCommand(Command.TRAVEL);
	}

	public void moveToLength(double length) {
		moveTo(length, this.targetAngle_deg);
	}

	public void moveToAngle(double angle) {
		moveTo(this.targetLength_m, angle);
	}

	public void manual(double extendVolts, double rotateVolts) {
		this.targetExtendVolts_v = extendVolts;
		this.targetRotateVolts_v = rotateVolts;
		setCommand(Command.MANUAL);
	}

	public void manualExtend(double extendVolts) {
		manual(extendVolts, this.targetRotateVolts_v);
	}

	public void manualRotate(double rotateVolts) {
		manual(this.targetExtendVolts_v, rotateVolts);
	}

	public double getLength() {
		return extendMotor.getPosition();
	}

	/*
	 * The angles from the two motors SHOULD be the same, but take average 
	 * to average out errors between motors.
	 */
	public double getAngle() {
		return (rotateLeftMotor.getPosition() + rotateRightMotor.getPosition()) / 2;
	}

	/*
	 * The angle velocities from the two motors SHOULD be the same, but take average 
	 * to average out errors between motors.
	 */
	public double getAngleVelocity() {
		return (rotateLeftMotor.getVelocity() + rotateRightMotor.getVelocity()) / 2;
	}

	public boolean atRotationTarget() {
		return Math.abs(targetAngle_deg - getAngle()) < ANGLE_TOLERANCE_deg;
	}

	public boolean atExtendTarget() {
		return Math.abs(targetLength_m - getLength()) < LENGTH_TOLERANCE_m;
	}

	public boolean atTarget() {
		return atRotationTarget() && atExtendTarget();
	}
}
