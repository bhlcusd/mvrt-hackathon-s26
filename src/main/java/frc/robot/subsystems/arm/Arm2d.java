package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Arm2d {
	private LoggedMechanism2d mech;
	private LoggedMechanismRoot2d root;
	private LoggedMechanismLigament2d arm;
	
	private String name;

	public Arm2d(String name, Color8Bit color) {
		this.name = name;

		// TODO: Init logged mechanisms
		// First init mech, then get the root from mech, then add ligaments onto root
	}
	
	public void periodic() {
		SmartDashboard.putData(name, mech);
		Logger.recordOutput(name, mech);
	}
}
