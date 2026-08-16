package frc.robot.subsystems.hand;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.generated.TunerConstants;

public class HandConstants {
	public HandConstants() {}

	// Motor configs
	public static final int MOTOR_ID = 5;

	public static final String CANBUS = TunerConstants.kCANBus.getName();
	public static final boolean BRAKE = true;

	// PID
	public static final double kP = 1.0;
	public static final double kI = 0;
	public static final double kD = 0.2;
	public static final double kA = 0;
	public static final double kG = 0;
	public static final double kV = 0.11;
	public static final double kS = 0.06;

	public static final GravityType GRAVITY_TYPE = GravityType.NONE;

	// Gear Ratio
	public static final double GEAR_RATIO = 1;
	public static final double SENSOR_TO_MECHANISM_RATIO = GEAR_RATIO;

	// Motion Magic
	public static final double CRUISE_VELOCITY_rps = 1.0;
	public static final double ACCELERATION_rps2 = 4.0;
	public static final double JERK_rps3 = 20.0;

	// Tolerances
	public static final double TOLERANCE_deg = 1.5;

	// Physical values
	public static final double INTAKE_deg = 1440;
	public static final double EXPEL_deg = 1440;
	public static final double ZERO_deg = 0;

	// Sim
	public static final DCMotor MOTOR = DCMotor.getFalcon500Foc(1);
	public static final double SIM_MOI_kgm2 = 0.005;

	// Manual
	public static final double MANUAL_VOLTS_v = 2;
}
