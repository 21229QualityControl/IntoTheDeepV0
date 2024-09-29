package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Outtake;

/*
Change all the "CHANGE THIS" things for your robot!

Controls
DRIVE:
Left joystick: forward/backward/strafe
Triggers: turn, Bumpers: slow turn (21229 drivers like this)

ARM:
Right joystick up/down: move arm up and down
Y: Move the arm to the low basket preset position

INTAKE:
A: toggle intake
B:  toggle reverse/score

Wrist:
X: toggle wrist between in and out
 */

@TeleOp(name = "Manual Drive", group = "Test")
@Config
public class ManualDriveSlide extends OpMode {
    public static double TICK_PER_RAD = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * (1+(46.0/17.0)) * 28) / 2*Math.PI / 3.2;
    // CHANGE THIS: The "/ 3.2" at the end of the previous line is the gear ratio, since we were using a 32 tooth sprocket on the arm and a 10 tooth sprocket on the motor
    // Make sure to initialize the robot with the arm resting inside the robot

    // CHANGE THIS: The wrist out position should be how you intake and wrist in should be how it is initialized
    public static double WRIST_OUT = 0.31;
    public static double WRIST_IN = 0.67;

    //private Follower follower;
    private CRServo intakeServo;
    private Outtake outtake;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

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

        intakeServo = hardwareMap.get(CRServo.class, "intake");
        intakeServo.setPower(0.0);

        outtake = new Outtake(hardwareMap);
        outtake.prepInitializeSlides();
        outtake.initialize(true);

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    boolean aPressed = false;
    boolean bPressed = false;
    boolean xPressed = false;

    private boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) < 0.00001;
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        outtake.update();
        /*follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();*/

        // Drivetrain
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.4;
        if (gamepad1.left_bumper) {
            rx += 0.15;
        }
        if (gamepad1.right_bumper) {
            rx -= 0.15;
        }

        leftFront.setPower(y + x + rx);
        leftRear.setPower(y - x + rx);
        rightFront.setPower(y - x - rx);
        rightRear.setPower(y + x - rx);

        // Intake servo control
        if (gamepad1.a && !aPressed) {
            aPressed = true;
            intakeServo.setPower(epsilonEquals(intakeServo.getPower(), 0.0) ? -1.0 : 0.0);
        } else if (!gamepad1.a) {
            aPressed = false;
        }
        if (gamepad1.b && !bPressed) {
            bPressed = true;
            intakeServo.setPower(epsilonEquals(intakeServo.getPower(), 0.0) ? 1.0 : 0.0);
        } else if (!gamepad1.b) {
            bPressed = false;
        }

        // Wrist control
        if (gamepad1.x && !xPressed) {
            xPressed = true;
        } else if (!gamepad1.x) {
            xPressed = false;
        }

        // slide control
        if (gamepad1.y && !xPressed) {
            outtake.raiseSlides();
        }

        telemetry.update();
    }
}
