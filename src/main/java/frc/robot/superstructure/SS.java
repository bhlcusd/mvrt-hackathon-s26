package frc.robot.superstructure;

import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.hand.Hand;
import frc.robot.subsystems.hand.HandConstants;
import frc.robot.subsystems.tracking.Tracking;
import frc.robot.subsystems.vision.Vision;

import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;

public class SS extends SubsystemBase<SS.Command> {

    public enum Flag {
        DISABLE,
        IDLE,
        STOW,
        MANUAL,
        INTAKE,
        SCORE
    }

    public enum Command {
        DISABLE,
        IDLE,
        MANUAL,
        INTAKE,
        SCORE,
        STOW,
    }

    // Value Enums (could be used as substate or hold enumerated values)
    public enum Manual {
        ARM_ROTATE,
        ARM_EXTEND,
        HAND_INTAKE,
        HAND_EXPEL
    }

    public enum Score {
        LOW(0),
        MED(1),
        HIGH(2);

        private final int index;

        private Score(int index) {
            this.index = index;
        }

        public int index() {
            return index;
        }
    }

    // Substate enums (only used to differentiate between phases for an action)
    private enum Intake {
        LOWERING,
        INTAKE,
        READY
    }

    private enum Scoring {
        TRAVELING,
        SETTLING,
        READY
    }

    private static final double SETTLE_TIME_s = 0.2;

    private final EnumSet<Flag> flags = EnumSet.noneOf(Flag.class);

    private static SS instance;
    
    private final Arm arm;
    private final Hand hand;
    private final Vision vision;
    private final Tracking tracking;

    private Manual manualMode;
    private Score scoreMode;

    private int manualDirection;

    private double armAngleTarget_deg;
    private double armLengthTarget_m;
    private double handAngle_deg;

    public static SS getInstance() {
        if (instance == null) {
            instance = new SS();
        }
        return instance;
    }

    private SS() {
        super("Superstructure");

        arm = Arm.getInstance();
        hand = Hand.getInstance();
        vision = Vision.getInstance();
        tracking = Tracking.getInstance();

        this.manualDirection = 1;

        setCommand(Command.IDLE);
    }

    @Override
    protected void inputPeriodic() {}

    @Override
    protected void handle() {
        handleFlags();

        switch (getCommand()) {
            case DISABLE:
                arm.disable();
                hand.disable();
                vision.disable();
                tracking.disable();
                break;
            case IDLE:
                arm.idle();
                hand.idle();
                break;
            case MANUAL:
                handleManual();
                break;
            case INTAKE:
                handleIntake();
                break;
            case SCORE:
                handleScoring();
                break;
            case STOW:
                arm.stow();
                hand.zero();
                break;
            default:
                System.out.println("Unimplemented command: " + getCommand().name());
                break;
        }
    }

    private void handleFlags() {
        if (has(Flag.DISABLE)) {
            setCommand(Command.DISABLE);
        } else if (has(Flag.IDLE)) {
            setCommand(Command.IDLE);
        } else if (has(Flag.MANUAL)) {
            setCommand(Command.MANUAL);
        } else if (has(Flag.STOW)) {
            setCommand(Command.STOW);
        } else if (has(Flag.INTAKE)) {
            setCommand(Command.INTAKE);
        } else if (has(Flag.SCORE)) {
            setCommand(Command.SCORE);
        }
    }

    private void handleManual() {
        if (!(getSubstate() instanceof Manual)) {
            if (manualMode != null) {
                setSubstate(manualMode);
            } else {
                System.out.println("Command.Manual does not have corresponing Manual substate!");
                
                arm.idle();
                hand.idle();

                return;
            }
        }

        switch ((Manual) getSubstate()) {
            case ARM_EXTEND:
                arm.manualExtend(manualDirection * ArmConstants.MANUAL_EXTEND_VOLTS_v);
                break;
            case ARM_ROTATE:
                arm.manualRotate(manualDirection * ArmConstants.MANUAL_ROTATE_VOLTS_v);
                break;
            case HAND_EXPEL:
                if (manualDirection < 0) {
                    toggleDirection();
                }

                hand.manual(manualDirection * HandConstants.MANUAL_VOLTS_v);
                break;
            case HAND_INTAKE:
                if (manualDirection > 0) {
                    toggleDirection();
                }

                hand.manual(manualDirection * HandConstants.MANUAL_VOLTS_v);
                break;
            default:
                // A ClassCastException would be thrown if the substate was not Manual or null, so no need to intervene here
                break;
        }
    }

    private void toggleDirection() {
        this.manualDirection = -this.manualDirection;
    }

    private void handleIntake() {
        if (firstLoop()) {
            setSubstate(Intake.LOWERING);
        }

        switch ((Intake) getSubstate()) {
            case LOWERING:
                arm.moveTo(ArmConstants.INTAKE_LENGTH_m, ArmConstants.INTAKE_ANGLE_deg);

                if (arm.atTarget()) {
                    setSubstate(Intake.INTAKE);
                }
                break;
            case INTAKE:
                hand.intake(HandConstants.INTAKE_deg);

                if (!arm.atTarget()) {
                    setSubstate(Intake.LOWERING);
                } else if (hand.atAngleTarget()) {
                    setSubstate(Intake.READY);
                }
                break;
            case READY:
                // Do nothing here, just a signal that intake is complete or "Ready" for teleoperation
                break;
            default:
                break;
        }
    }

    private void handleScoring() {
        if (firstLoop()) {
            setSubstate(Scoring.TRAVELING);
        }

        this.armAngleTarget_deg = ArmConstants.SCORING_ANGLES[scoreMode.index()];
        this.armLengthTarget_m = ArmConstants.SCORING_LENGTHS[scoreMode.index()];
        this.handAngle_deg = HandConstants.EXPEL_deg;

        switch ((Scoring) getSubstate()) {
            case TRAVELING:
                arm.moveTo(armLengthTarget_m, armAngleTarget_deg);

                if (arm.atTarget()) {
                    setSubstate(Scoring.SETTLING);   
                }
                break;
            case SETTLING:
                arm.moveTo(armLengthTarget_m, armAngleTarget_deg);
                
                if (!arm.atTarget()) {
                    setSubstate(Scoring.TRAVELING);    
                } else if (substateElapsed(SETTLE_TIME_s)) {
                    setSubstate(Scoring.READY);  
                }
                break;
            case READY:
                arm.moveTo(armLengthTarget_m, armAngleTarget_deg);
                hand.expel(handAngle_deg);

                if (!arm.atTarget()) {
                    setSubstate(Scoring.TRAVELING);   
                }
                break;
            default:
                break;
        }
    }

    @Override
    protected void outputPeriodic() {
        String[] active = flags.stream().map(Enum::name).toArray(String[]::new);
        Logger.recordOutput("Superstructure/Flags", active);
        Logger.recordOutput("Superstructure/ArmAngleTarget_deg", armAngleTarget_deg);
        Logger.recordOutput("Superstructure/ArmLengthTarget_m", armLengthTarget_m);
        Logger.recordOutput("Superstructure/HandAngle_deg", handAngle_deg);
    }

    /**
     * Sets the mode to manual using the Manual substate. Setting substate the normal way won't work since
     * the Superstructure changes the command, wiping the substate. This keeps the substate in a dedicated
     * field variable and gets reassigned to the substate when needed in handleManual()
     * 
     * @param manual            the manual mode
     * @param active            true if active, false otherwise
     */
    public void setManual(Manual manual, boolean active) {
        if (active) {
            if (getCommand() == Command.MANUAL) {
                setSubstate(manual);
            }

            this.manualMode = manual;
        } else {
            this.manualMode = null;
        }

        set(Flag.MANUAL, active);
    }

    /**
     * Assigns the current score to the provided one if active, null otherwise. Then flags to score.
     * 
     * @param score             the score mode
     * @param active            true if active, false otherwise
     */
    public void setScore(Score score, boolean active) {
        this.scoreMode = active ? score : null;
        set(Flag.SCORE, active);
    }

    // INFO: The following are methods to handle flags. Do not modify!

    public void enable(Flag flag) {
        flags.add(flag);
    }

    public void disable(Flag flag) {
        flags.remove(flag);
    }

    public void set(Flag flag, boolean active) {
        if (active) {
            flags.add(flag);
        } else {
            flags.remove(flag);
        }
    }

    public void toggle(Flag flag) {
        set(flag, !has(flag));
    }

    public boolean has(Flag flag) {
        return flags.contains(flag);
    }
}
