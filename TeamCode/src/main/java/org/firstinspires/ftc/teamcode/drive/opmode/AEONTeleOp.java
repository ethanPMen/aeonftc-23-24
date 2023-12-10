package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class AEONTeleOp extends LinearOpMode {
    class SlewRateLimiter {
        private double slewRateMax;
        private double lastSetpoint = 0;

        SlewRateLimiter(double slewRateMax) {
            this.slewRateMax = slewRateMax;
        }

        double update(double setpoint) {
            double change = setpoint - lastSetpoint;
            if (Math.abs(change) > slewRateMax) {
                return slewRateMax * Math.signum(change);
            }
            return setpoint;
        }
    }

    final double kElevatorPower = .7;
    double kElevatorOffset = 0;
    final double kElevatorScale = 1.0 / 100.0;

    private double getElevatorPosition() {
        return kElevatorScale * (elevatorMotor.getCurrentPosition() - kElevatorOffset);
    }

    private void reZero() {
        kElevatorOffset = elevatorMotor.getCurrentPosition();
    }

    DcMotor elevatorMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftRear = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightRear = hardwareMap.dcMotor.get("rightRear");
        elevatorMotor = hardwareMap.dcMotor.get("elevatorMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo droneServo = hardwareMap.servo.get("droneServo");
        Servo trapdoorServo = hardwareMap.servo.get("trapdoorServo");
        final double kP = 1.0 / 3.0;
        final double kD = 0;
        final double kI = 0;
        boolean intakeOn = false;
        boolean trapdoorBoom = false;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;
        double setpoint = 0;
        double lastError = 0;
        boolean prevRightBumper = gamepad1.right_bumper;
        boolean prevLeftBumper = gamepad1.left_bumper;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean buttonY = gamepad1.y;
            boolean buttonB = gamepad1.b;
            boolean buttonA = gamepad1.a;
            boolean buttonX = gamepad1.x;
            // for intake toggle
            boolean rightBumperPressed = !prevRightBumper && gamepad1.right_bumper;
            prevRightBumper = gamepad1.right_bumper;
            // trapdoor toggle
            boolean leftBumperPressed = !prevLeftBumper && gamepad1.left_bumper;
            prevLeftBumper = gamepad1.left_bumper;

            //operator buttons
            boolean opY = gamepad2.y;
            boolean opB = gamepad2.b;
            boolean opA = gamepad2.a;
            boolean opX = gamepad2.x;
            boolean opLB = gamepad2.left_bumper;
            boolean opRB = gamepad2.right_bumper;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            //intake command
            if (rightBumperPressed) {
                if (intakeOn) {
                    intakeOn = false;
                } else {
                    intakeOn = true;
                }
            }
            if (intakeOn) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }

            if (opRB) {
                intakeMotor.setPower(-1);
            }

            //trapdoor
            if (leftBumperPressed) {
                if (trapdoorBoom) {
                    trapdoorBoom = false;
                } else {
                    trapdoorBoom = true;
                }
                if (trapdoorBoom) {
                    trapdoorServo.setPosition(0);
                } else {
                    trapdoorServo.setPosition(1);
                }
            }

            //elevator code
            setpoint = elevatorMotor.getTargetPosition() * kElevatorScale;
            double elevatorPosition = getElevatorPosition();

            double error = setpoint - elevatorPosition;
            double changeInError = error - lastError;
            double elevatorPower = kP * error + kD * changeInError;
            lastError = error;

            // TRAPDOOR AUTO CLOSE
            if (elevatorPosition < 13.0) {
                trapdoorServo.setPosition(1);
            }

            //elevator
            if (opA) {
                reZero(); //sets whatever position it's at to zero
            }
            if (buttonA && (elevatorPosition > 0)) { //stow
                elevatorMotor.setTargetPosition(0);
            }
            if (buttonX) {
                elevatorMotor.setTargetPosition((int) (14.0 / kElevatorScale)); // low
            }
            if (buttonB) {
                elevatorMotor.setTargetPosition((int) (27.0 / kElevatorScale)); // mid
            }
            if (buttonY && elevatorPosition < 3900 * kElevatorScale) { //high
                elevatorMotor.setTargetPosition((int) (39.0 / kElevatorScale));
            }
            elevatorMotor.setPower(elevatorPower);

            //manual movement
            if (opX) {
                elevatorMotor.setPower(.5); //up
                elevatorMotor.setTargetPosition(elevatorMotor.getCurrentPosition());
            }
            if (opB) {
                elevatorMotor.setPower(-.5);
                elevatorMotor.setTargetPosition(elevatorMotor.getCurrentPosition());//down
            }
            if (opY) {
                elevatorPower = 0;
                elevatorMotor.setTargetPosition(elevatorMotor.getCurrentPosition()); //stop
            }


            //drone code
            if (opLB) {
                droneServo.setDirection(Servo.Direction.REVERSE);
                droneServo.setPosition(0); //to release
            }
            //end of drone code

            if (elevatorPosition < 11.0) {
                leftFront.setPower(frontLeftPower);
                leftRear.setPower(backLeftPower);
                rightFront.setPower(frontRightPower);
                rightRear.setPower(backRightPower);
            } else {
                leftFront.setPower(frontLeftPower / 4);
                leftRear.setPower(backLeftPower / 4);
                rightFront.setPower(frontRightPower / 4);
                rightRear.setPower(backRightPower / 4);
            }

            telemetry.addLine("Elevator:");
            telemetry.addData("Elevator Setpoint", setpoint);
            telemetry.addData("Elevator Raw Position", elevatorMotor.getCurrentPosition());
            telemetry.addData("Elevator Position", getElevatorPosition());
            telemetry.addData("Elevator Target Position", elevatorMotor.getTargetPosition());
            telemetry.addData("Error", error);
            telemetry.addData("Elevator Power", elevatorPower);
            telemetry.addLine("\nfield centric stuff idk:");
            telemetry.addData("YawPitchRoll Angles:", imu.getRobotYawPitchRollAngles());
            telemetry.addData("Orientation as Quaternion:", imu.getRobotOrientationAsQuaternion());
            telemetry.addData("Drone Servo position", droneServo.getPosition());
            telemetry.addData("trapdoor position", trapdoorServo.getPosition());
            telemetry.addData("trapdoor pressed", trapdoorBoom);
            telemetry.addData("ticks", hardwareMap.dcMotor.get("leftFront").getCurrentPosition());
            telemetry.addData("elevator ticks", hardwareMap.dcMotor.get("elevatorMotor").getCurrentPosition());
            telemetry.update();
        }
    }
}