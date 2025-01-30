package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="MecanumTeleOp Rev1", group="TeleOp")
public class MecanumTeleOpRevOne extends LinearOpMode {
    // Arm pos control variables--possibly redundant as this snippet is authored by GoBilda.
    //Its a possible dependency for the rest of their position holding code so womp womp
    final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0; //what it looks like
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    double armPosition = ARM_COLLAPSED_INTO_ROBOT;  // Starting position is collapsed into the robot

    public DcMotor leftDrive = null; // the left drivetrain motor
    public DcMotor rightDrive = null; // the right drivetrain motor
    public DcMotor ViperSlideMotor = null; // the 1 Viper slide motor
    public DcMotor armMotor = null; // the arm motor
    public CRServo intake = null; // the active intake servo
    public CRServo wrist = null; // the wrist servo

    @Override

    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("drive0");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("drive1");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("drive2");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("drive3");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm0");
        DcMotor ViperSlideMotor = hardwareMap.dcMotor.get("VSM0");
        wrist = hardwareMap.get(CRServo.class, "serv0");
        intake = hardwareMap.get(CRServo.class, "serv1");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ViperSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // setting all the motors to break mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //most important, this makes pos holding eco-friendly
        ViperSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0); //resetting manually too for redundancy i guess...
        //if have time, remove and observe results
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use encoder-based control, necessary for continuous pos holding in teleop
        //Carson, if this breaks code we can always revert to RUN_USING_ENCODER :)

        telemetry.addLine("Sigmus Prime Rollout!");
        telemetry.update();

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            int armPosition = armMotor.getCurrentPosition();

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Regulate motor speeds
            double rx = gamepad1.right_stick_x;
            //double dy = -gamepad2.right_stick_y;
            //commented out to be replaced with arm hold
        
            // Denominator is the largest motor power (1)
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Control arm motor
            //if (dy > 0.2) {
            //    armMotor.setPower(-0.4);
            //} else if (dy < -0.2) {
            //    armMotor.setPower(0.4);
            //} else {
            //    armMotor.setPower(-0.01);
            //}
            //see line 72

            if (armPosition < -1970) {
                armMotor.setPower(0.2);
            }
            //Leaving this because it's needed to pass inspection, if this breaks code, do not remove,
            //save file and revert to previous version of this class.

            //begin arm hold

            if (Math.abs(-gamepad2.right_stick_y) > 0.1) {//inverted to match original controls
                armPosition += gamepad1.left_stick_y * 10; // Adjust speed (number) for arm movement
            }//arm hold should work with this, this is Revision1 i guess


            armPosition = Math.max(ARM_COLLAPSED_INTO_ROBOT, armPosition); // Don't allow the arm to go too far in
            // You can also add an upper limit here if needed (max)

            // Set the arm's target position and run it to that position
            armMotor.setTargetPosition((int) armPosition);

            // Run the arm motor to the target position
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);  // ajust power number (speed value) based on needs

            // Telemetry feedback for arm hold
            telemetry.addData("Arm Target Position", armMotor.getTargetPosition());
            telemetry.addData("Arm Current Position", armMotor.getCurrentPosition());
            telemetry.update();
            //end arm hold

            // Control intake servo
            if (gamepad2.y) {
                intake.setPower(1.0);
            } else if (gamepad2.x) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }

            if (gamepad2.right_trigger > 0) {
                ViperSlideMotor.setPower(-0.6);
            } else if (gamepad2.left_trigger > 0) {
                ViperSlideMotor.setPower(0.6);
            } else {
                ViperSlideMotor.setPower(0.03);
            }
        }
    }

}