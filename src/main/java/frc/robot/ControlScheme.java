package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.subsystems.drive.PathingOverride;
import frc.robot.subsystems.drive.SwerveInput;
import frc.robot.superstructure.SS;
import frc.robot.superstructure.SS.Flag;
import frc.robot.superstructure.SS.Manual;
import frc.robot.superstructure.SS.Score;
import frc.robot.util.Util;

public class ControlScheme {

    private final SS ss;
    private final Drive drive;

    private int scoring;
    private static final Score[] SCORE_MODES = {Score.LOW, Score.MED, Score.HIGH};

    private int manual;
    private static final Manual[] MANUAL_MODES = {Manual.ARM_ROTATE, Manual.ARM_EXTEND, Manual.HAND_INTAKE, Manual.HAND_EXPEL};

    public ControlScheme(SS ss, Drive drive) {
        this.ss = ss;
        this.drive = drive;
        
        this.scoring = 0;
        this.manual = 0;
    }

    public void init() {
        drive.queueState(PathingMode.FIELD_RELATIVE);
        drive.setPathingOverride(PathingOverride.NONE);
        System.out.println("Controls initialized");
    }

    public void update() {
        double x_ = OI.deadband(-OI.DR.getLeftY());
        double y_ = OI.deadband(-OI.DR.getLeftX());
        double w_ = 0.5 * -Util.sqInput(OI.deadband(OI.DR.getRightX()));
        double throttle = Util.sqInput(
                1.0 - OI.deadband(Math.max(OI.DR.getLeftTriggerAxis(), OI.DR.getRightTriggerAxis())));

        if (OI.DR.getPOV() == 180) {
            drive.zeroGyro();
        }
        drive.setInput(new SwerveInput(x_, y_, w_, throttle));

        if (OI.DR.getRightTriggerAxis() > 0.5) {
            drive.setPathingOverride(PathingOverride.TRACKING);
        } else {
            drive.setPathingOverride(PathingOverride.NONE);
        }

        // Input mapping (for my broken controller)
        // Left bumper - 5
        // Right bumper - 6
        // A - 1
        // B - 2
        // X - 3
        // Y - 4
        // Hamburger Menu - 8
        // Mirror - 7
        // Right joystick button - 10

        // Check left bumper to increase Manual value
        if (OI.DR.getLeftBumperButtonPressed()) {
            manual = Math.abs(manual + 1) % MANUAL_MODES.length;
            System.out.println("Manual is now: " + manual);
        }

        // Check right bumper to increase the scoring value
        if (OI.DR.getRightBumperButtonPressed()) {
            scoring = Math.abs(scoring + 1) % SCORE_MODES.length;
            System.out.println("Score is now: " + scoring);
        }

        ss.set(Flag.DISABLE, OI.DR.getRawButton(7)); // Mirror button
        ss.set(Flag.IDLE, OI.DR.getAButton());
        ss.setManual(MANUAL_MODES[manual], OI.DR.getBButton());
        ss.set(Flag.INTAKE, OI.DR.getXButton());
        ss.setScore(SCORE_MODES[scoring], OI.DR.getYButton());
        ss.set(Flag.STOW, OI.DR.getRawButton(10)); // Right joystick button
    }
}
