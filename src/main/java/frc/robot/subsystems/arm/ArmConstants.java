package frc.robot.subsystems.arm;

import java.util.Arrays;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.generated.TunerConstants;

public class ArmConstants {
	private ArmConstants() {}

	// Motor configs
	public static final int LEFT_MOTOR_ID = 0;
	public static final int RIGHT_MOTOR_ID = 1;
	public static final int EXTEND_MOTOR_ID = 2;
	
	public static final String CANBUS = TunerConstants.kCANBus.getName();
	public static final boolean BRAKE = true;
	public static final double[] SEGMENT_MOI_kgm2 = {0.146, 0.47, 1.373};
	// We could change this to instead return a predicted MOI value at a given length instead of pre-calculated MOI
	// MOI should be similar to a rod (ie. equation should be quadratic - rod: 1/12 * m * l^2), but IDK about the SIM MOI provided since that must be a constant

	// PID
	public static final double kRotateP = 0;
	public static final double kRotateI = 0;
	public static final double kRotateD = 0;
	public static final double kRotateA = 0;
	public static final double kRotateG = 0;
	public static final double kRotateV = 0;
	public static final double kRotateS = 0;

	public static final double kExtendP = 0;
	public static final double kExtendI = 0;
	public static final double kExtendD = 0;
	public static final double kExtendA = 0;
	public static final double kExtendG = 0;
	public static final double kExtendV = 0;
	public static final double kExtendS = 0;

	public static final GravityType ROTATE_GRAVITY_TYPE = GravityType.ARM;
	public static final GravityType EXTEND_GRAVITY_TYPE = GravityType.ELEVATOR;

	// Gear ratios
	public static final double ROTATE_GEAR_RATIO = 7.5;
	public static final double EXTEND_GEAR_RATIO = 2.4;

	// Motion magic
	public static final double ROTATE_CRUISE_VELOCITY_rps = 11.34;
	public static final double ROTATE_ACCELERATION_rps2 = 2.26;
	public static final double ROTATE_JERK_rps3 = 34.02;

	public static final double EXTEND_CRUISE_VELOCITY_mps = 0;
	public static final double EXTEND_ACCELERATION_mps2 = 0;
	public static final double EXTEND_JERK_mps3 = 0;

	// Tolerances
	public static final double LENGTH_TOLERANCE_m = 0.05;
	public static final double ANGLE_TOLERANCE_deg = 5;

	// Physical values
	public static final int SEGMENTS = 3;
	public static final double ARM_LENGTH_m = 0.62;
	public static final double MIN_LENGTH_m = 0.8;
	public static final double MAX_LENGTH_m = 1.77;
	
	// These angles are read as if the arm is 90 degress when pointing upwards.
	// Technically the values can be smaller due to changes in arm length, but these
	// are measured with the shortest arm length.
	public static final int MIN_ANGLE_deg = 20;
	public static final int MAX_ANGLE_deg = 160;

	// Sim
	public static final DCMotor ROTATE_SIM_MOTORS = DCMotor.getFalcon500Foc(2);
	public static final DCMotor EXTEND_SIM_MOTOR = DCMotor.getFalcon500Foc(1);
	public static final double SIM_MOI_kgm2 = Arrays.stream(SEGMENT_MOI_kgm2).average().orElse(0.146);
}
