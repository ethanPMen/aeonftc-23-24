package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ARCTO extends LinearOpMode {
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
        final double kP = 1.0 / 4.0;
        final double kD = 0;
        final double kI = 0;
        boolean intakeOn = false;
        boolean trapdoorBoom = false;
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        if (isStopRequested()) return;
        double setpoint = 0;
        double lastError = 0;
        boolean prevRightBumper = gamepad1.right_bumper;
        boolean prevLeftBumper = gamepad1.left_bumper;

        while (opModeIsActive()) {
            //resetEncoders();

            double y = -gamepad1.left_stick_y; // Remember, Y stic/k value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
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
            prevLeftBumper = gamepad1.right_bumper;

            //operator buttons
            boolean opY = gamepad2.y;
            boolean opB = gamepad2.b;
            boolean opA = gamepad2.a;
            boolean opX = gamepad2.x;
            boolean opLB = gamepad2.left_bumper;
            boolean opRB = gamepad2.right_bumper;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

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
            //trapdoor
            if (leftBumperPressed) {
                if (trapdoorBoom) {
                    trapdoorBoom = false;
                } else {
                    trapdoorBoom = true;
                }
            }
            if (trapdoorBoom) {
                trapdoorServo.setPosition(1);
            } else {
                trapdoorServo.setPosition(0);
            }

            //elevator code
            setpoint = elevatorMotor.getTargetPosition() * kElevatorScale;
            double elevatorPosition = getElevatorPosition();

            double error = setpoint - elevatorPosition;
            double changeInError = error - lastError;
            double elevatorPower = kP * error + kD * changeInError;
            lastError = error;

            //elevator

            if (opA) {
                reZero(); //sets whatever position it's at to zero
            }
            if (buttonA && (elevatorPosition > 0)) { //stow
                elevatorMotor.setTargetPosition(0);
            }
            if (buttonX) {
                elevatorMotor.setTargetPosition((int)(12.0 / kElevatorScale)); // low
            }
            if (buttonB) {
                elevatorMotor.setTargetPosition((int)(27.0 / kElevatorScale)); // mid
            }
            if (buttonY && elevatorPosition < 3900 * kElevatorScale) { //high
                elevatorMotor.setTargetPosition((int)(39.0 / kElevatorScale));
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

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            telemetry.addData("Drone Servo position", droneServo.getPosition());
            telemetry.addData("Elevator Setpoint", setpoint);
            telemetry.addData("Elevator Raw Position", elevatorMotor.getCurrentPosition());
            telemetry.addData("Elevator Position", getElevatorPosition());
            telemetry.addData("Elevator Target Position", elevatorMotor.getTargetPosition());
            telemetry.addData("Error", error);
            telemetry.addData("Elevator Power", elevatorPower);
            telemetry.addData("kP", kP);
            telemetry.update();
        }
    }
}