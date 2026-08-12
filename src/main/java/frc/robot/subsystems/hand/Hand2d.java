package frc.robot.subsystems.hand;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Hand2d {
	private LoggedMechanism2d mech;

	// Fixed root points
	private LoggedMechanismRoot2d intakeLeftRoot;
	private LoggedMechanismRoot2d intakeRightRoot;
	private LoggedMechanismRoot2d largeLeftRoot;
	private LoggedMechanismRoot2d largeRightRoot;
	
	// Lines to represent rollers (shows rotation)
	private LoggedMechanismLigament2d intakeLeft;
	private LoggedMechanismLigament2d intakeRight;
	private LoggedMechanismLigament2d largeLeft;
	private LoggedMechanismLigament2d largeRight;

	private String name;

	public Hand2d(String name, Color8Bit color) {
		this.name = name;

		this.mech = new LoggedMechanism2d(0.5, 0.5);
		
		this.intakeLeftRoot = mech.getRoot("IntakeLeftRoot", 0.13, 0.318);
		this.intakeLeft = intakeLeftRoot.append(new LoggedMechanismLigament2d("IntakeLeft", 0.2, 0, 2, color));
		
		this.intakeRightRoot = mech.getRoot("IntakeRightRoot", 0.37, 0.318);
		this.intakeRight = intakeRightRoot.append(new LoggedMechanismLigament2d("IntakeRight", 0.2, 0, 2, color));
		
		this.largeLeftRoot = mech.getRoot("LargeLeftRoot", 0.19, 0.203);
		this.largeLeft = largeLeftRoot.append(new LoggedMechanismLigament2d("LargeRight", 0.25, 0, 3, color));
		
		this.largeRightRoot = mech.getRoot("LargeRightRoot", 0.31, 0.203);
		this.largeRight = largeRightRoot.append(new LoggedMechanismLigament2d("LargeRight", 0.25, 0, 3, color));
	}

	/**
	 * Updates the rotation of the intakes and large rollers.
	 * 
	 * @param angle			angle in degrees
	 */
	public void update(double angle) {
		intakeLeft.setAngle(angle);
		intakeRight.setAngle(angle);
		largeLeft.setAngle(angle);
		largeRight.setAngle(angle);
	}
	
	public void periodic() {
		SmartDashboard.putData(name, mech);
		Logger.recordOutput(name, mech);
	}
}
