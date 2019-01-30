package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;

import org.firstinspires.ftc.teamcode.Utilities.Control.HoldingPIDMotor;

@Config
public class Extender {
    public static int MAX_EXTENDER_POS = 1100;
    public static int MIN_EXTENDER_POS = 0;
    public static int COLLECT_POS = 0;

    public static int MAXED_THRESHOLD = 800;
    public static int MIN_THRESHOLD = 0;
    public static int RESET_TRUE_POSITION = 970;

    private DigitalChannel slideSwitch;
    private HoldingPIDMotor extender;
    private boolean prevState;
    private int offset;

    public Extender(DigitalChannel slideSwitch, DcMotorEx extender) {
        this.extender = new HoldingPIDMotor(extender, 1.0);
        this.slideSwitch = slideSwitch;
    }

    public void setPower(double p) {
        extender.setPower(p);
        boolean currState = slideSwitch.getState();
        if (prevState && !currState) {
            // Reset encoder
            offset = extender.getCurrentPosition() - RESET_TRUE_POSITION;
            prevState = false;
        } else if (currState && !prevState) {
            prevState = true;
        }
        if (p == 0 && !extender.isBusy()) {
            extender.stop();
        }
    }

    public void goToPosition(int position) {
        extender.setTargetPos(position + offset);
    }

    public int getPosition() {
        return extender.getCurrentPosition() - offset;
    }

    public boolean maxExtend() { return getPosition() > MAXED_THRESHOLD; }
    public boolean minExtend() { return getPosition() < MIN_THRESHOLD; }
    public void goToMax() { goToPosition(MAX_EXTENDER_POS); }
    public void goToMin() { goToPosition(MIN_EXTENDER_POS); }
    public void goToCollect() { goToPosition(COLLECT_POS); }
}
