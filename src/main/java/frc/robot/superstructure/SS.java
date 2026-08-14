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

// TODO: Implement intake for intaking items
public class SS extends SubsystemBase<SS.Command> {

    public enum Flag {
        DISABLE,
        IDLE,
        STOW,
        MANUAL,
        INTAKE,
        SCORE_LOW,
        SCORE_MED,
        SCORE_HIGH,
    }

    public enum Command {
        DISABLE,
        IDLE,
        MANUAL,
        INTAKE,
        SCORE,
        STOW,
    }

    public enum Manual {
        ARM_ROTATE,
        ARM_EXTEND,
        HAND_INTAKE,
        HAND_EXPEL
    }

    private enum Intake {
        LOWERING,
        INTAKE,
        SETTLING,
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
            case IDLE:
                arm.idle();
                hand.idle();
                vision.enable();
                tracking.enable();
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
                arm.moveTo(ArmConstants.MIN_LENGTH_m, ArmConstants.STOW_ANGLE_deg);
                hand.zero();
                break;
            default:
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
        } else if (has(Flag.INTAKE)) {
            setCommand(Command.INTAKE);
        } else if (has(Flag.SCORE_LOW) || has(Flag.SCORE_MED) || has(Flag.SCORE_HIGH)) {
            setCommand(Command.SCORE);
        }
    }

    private void handleManual() {
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
                break;
            case INTAKE:
                break;
            case SETTLING:
                break;
            case READY:
                break;
            default:
                break;
        }
    }

    private void handleScoring() {
        if (firstLoop()) {
            setSubstate(Scoring.TRAVELING);
        }

        handleTargets();

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

    public void setManual(Manual manual, boolean active) {
        setSubstate(manual);
        set(Flag.MANUAL, active);
    }

    /**
     * Updates the current targets based on the current flag, accessing values from subsystem constant class's arrays.
     * 
     * Dev note: This isn't the best system, but without modifying how flags work (no support for mutually exclusive flags), 
     * it's the best for what the current handle() system needs. Besides, this project is pretty minor in scope, with only 
     * the Arm and Hand subsystems needing their own custom implementations.
     */
    private void handleTargets() {
        int index = -1;

        if (has(Flag.SCORE_LOW)) {
            index = 0;
        } else if (has(Flag.SCORE_MED)) {
            index = 1;
        } else if (has(Flag.SCORE_HIGH)) {
            index = 2;
        }

        this.armAngleTarget_deg = ArmConstants.SCORING_ANGLES[index];
        this.armLengthTarget_m = ArmConstants.SCORING_LENGTHS[index];
        this.handAngle_deg = HandConstants.EXPEL_deg;
    }

    @Override
    protected void outputPeriodic() {
        String[] active = flags.stream().map(Enum::name).toArray(String[]::new);
        Logger.recordOutput("Superstructure/Flags", active);
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
