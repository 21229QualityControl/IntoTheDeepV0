package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.util.MotorWithPID;
import org.firstinspires.ftc.teamcode.pedroPathing.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.pedroPathing.util.control.PIDCoefficients;

@Config
public class Outtake {
    public static PIDCoefficients outtakePID = new PIDCoefficients(0.007, 0.002, 0.0002);
    public static int OUTTAKE_TELEOP = 100; // Changes throughout teleop

    public static double CLAW_OPEN = 0.6;
    public static double CLAW_CLOSED = 0.27;

    final MotorWithPID slide;
    public boolean slidePIDEnabled = true;
    final Servo claw;
    final Servo wrist;

    public double outtakeFF(double target, double measured, double vel) {
        if ((target + slide.internalOffset) == 0 && (measured + slide.internalOffset) > 5 && Math.abs(this.slide.getVelocity()) < 5) {
            return -0.5;
        }
        return 0;
    }

    public Outtake(HardwareMap hardwareMap) {
        this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "slide"),
                outtakePID,
                this::outtakeFF);
        this.slide.setMaxPower(1.0);
        this.claw = HardwareCreator.createServo(hardwareMap, "outtakeClaw");
        this.wrist = HardwareCreator.createServo(hardwareMap, "outtakeWrist");
    }

    public void initialize(boolean teleop) {
        this.slide.resetIntegralGain();
        this.claw.setPosition(CLAW_OPEN);
        Log.d("initialization", "just started teleop");
        this.slide.setTargetPosition(0);
    }

    public void prepInitializeSlides() {
        this.slide.getMotor().setPower(-0.1);
    }

    public boolean initializeSlides() {
        if ((int)this.slide.getVelocity() == 0) {
            this.slide.getMotor().setPower(0);
            this.slide.setCurrentPosition(0);
            this.slide.setTargetPosition(0);
            return false;
        }
        return true;
    }

    public void update() {
        Log.d("POW", "update");
        if (slidePIDEnabled) {
            slide.update();
        }
    }

    public void setSlidePower(double power) {
        slide.getMotor().setPower(power);
    }
    public void resetSlideOffset() {
        this.slide.setCurrentPosition(0);
        this.slide.setTargetPosition(0);
    }

    public void raiseSlides() {
        slide.setTargetPosition(OUTTAKE_TELEOP);
    }
}