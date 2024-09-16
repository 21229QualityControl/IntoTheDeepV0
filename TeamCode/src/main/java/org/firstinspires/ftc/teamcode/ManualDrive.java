package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.FeedForwardConstant;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

@TeleOp(name = "Manual Drive", group = "Test")
@Config
public class ManualDrive extends OpMode {
    public static double ARM_F = -0.1;
    public static double TICK_PER_RAD = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * (1+(46.0/17.0)) * 28) / 2*Math.PI / 3.2;
    public static double ARM_OFF = -2.01;
    public static CustomPIDFCoefficients PID = new CustomPIDFCoefficients(1, 0.02, 0.02, new ArmPIDF());
    public static double ARM_TARGET = 0.0;
    public static double ARM_SPEED = 0.05;

    public static double WRIST_OUT = 0.31;
    public static double WRIST_IN = 0.67;

    public static double ARM_MIN = -2.25;
    public static double ARM_MAX = 1.9;
    public static double ARM_LOWBASKET = -0.2;

    static class ArmPIDF implements FeedForwardConstant {

        @Override
        public double getConstant(double input) {
            return Math.sin(input) * ARM_F;
        }
    }

    //private Follower follower;
    private DcMotorEx armMotor;
    private CRServo intakeServo;
    private Servo wristServo;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private double armAngle() {
        return (armMotor.getCurrentPosition() -  armStart)/TICK_PER_RAD - ARM_OFF;
    }

    private PIDFController armPID = new PIDFController(PID);
    private double armStart = 0.0;

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        /*follower = new Follower(hardwareMap);
        follower.startTeleopDrive();*/

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);


        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armStart = armMotor.getCurrentPosition();

        intakeServo = hardwareMap.get(CRServo.class, "intake");
        intakeServo.setPower(0.0);

        wristServo = hardwareMap.get(Servo.class, "wrist");
        wristServo.setPosition(WRIST_IN);

        ARM_TARGET = armAngle();

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    boolean aPressed = false;
    boolean bPressed = false;
    boolean xPressed = false;

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        /*follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();*/

        // Drivetrain
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.2;

        leftFront.setPower(y + x + rx);
        leftRear.setPower(y - x + rx);
        rightFront.setPower(y - x - rx);
        rightRear.setPower(y + x - rx);

        // Arm control
        armPID.updateFeedForwardInput(armAngle());
        armPID.setTargetPosition(ARM_TARGET);
        armPID.updatePosition(armAngle());
        armMotor.setPower(armPID.runPIDF());
        ARM_TARGET += gamepad1.right_stick_y * ARM_SPEED;

        // Check that arm target is in right position
        if (ARM_TARGET < ARM_MIN) {
            ARM_TARGET = ARM_MIN;
        } else if (ARM_TARGET > ARM_MAX) {
            ARM_TARGET = ARM_MAX;
        }

        if (gamepad1.y) {
            ARM_TARGET = ARM_LOWBASKET;
        }


        // Intake servo control
        if (gamepad1.a && !aPressed) {
            aPressed = true;
            intakeServo.setPower(intakeServo.getPower() == 0.0 ? -1.0 : 0.0);
        } else if (!gamepad1.a) {
            aPressed = false;
        }
        if (gamepad1.b && !bPressed) {
            bPressed = true;
            intakeServo.setPower(intakeServo.getPower() == 0.0 ? 1.0 : 0.0);
        } else if (!gamepad1.b) {
            bPressed = false;
        }

        // Wrist control
        if (gamepad1.x && !xPressed) {
            xPressed = true;
            wristServo.setPosition(wristServo.getPosition() == WRIST_IN ? WRIST_OUT : WRIST_IN);
        } else if (!gamepad1.x) {
            xPressed = false;
        }

        telemetry.addData("Arm Angle (deg)", Math.toDegrees(armAngle()));
        telemetry.addData("Arm Power", armPID.runPIDF());
        telemetry.update();
    }
}
