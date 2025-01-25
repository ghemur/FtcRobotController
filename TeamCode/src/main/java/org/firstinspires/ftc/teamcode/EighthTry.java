package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * EighthTry uses gamePad1 for controlling the wheels.
 * The control arm and servo are controlled by gamePad2 if available, else by gamePad1.
 */
@TeleOp
public class EighthTry extends LinearOpMode {

    private static final double MAX_POWER = 1.0;
    private static final double ZERO_POWER = 0;
    private static final double MID_POWER = 0.5;
    private static final double POWER_STEP = 0.2;

    // Servo constants
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    private final ElapsedTime runtime = new ElapsedTime();

    // Wheel motors
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private double speedCoeff;

    // Arm motor
    private DcMotor armMotor;
    private double armPower;

    // Servo
    private Servo servo;
    double  position;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            updateParams();
            moveWheels();
            moveArm();
            moveServo();

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    private Gamepad getWheelGamePad() {
        return gamepad1;
    }

    private Gamepad getArmGamePad() {
        return gamepad2 != null ? gamepad2 : gamepad1;
    }

    private Gamepad getServoGamePad() {
        return gamepad2 != null ? gamepad2 : gamepad1;
    }

    /**
     * Read joystick buttons, change params
     */
    private void updateParams() {
        Gamepad wheelGamePad = getWheelGamePad();
        if (wheelGamePad.right_bumper) {
            speedCoeff += POWER_STEP;
            if (speedCoeff > MAX_POWER) {
                speedCoeff = MAX_POWER;
            }
        } else if (wheelGamePad.left_bumper) {
            speedCoeff -= POWER_STEP;
            if (speedCoeff < ZERO_POWER) {
                speedCoeff = ZERO_POWER;
            }
        }

        Gamepad armGamePad = getArmGamePad();
        if (armGamePad.right_bumper) {
            armPower += POWER_STEP;
            if (armPower > MAX_POWER) {
                armPower = MAX_POWER;
            }
        } else if (armGamePad.left_bumper) {
            armPower -= POWER_STEP;
            if (armPower < ZERO_POWER) {
                armPower = ZERO_POWER;
            }
        }
    }

    /**
     * "Button B" to go up "Button A" to go down, at the same set speed
     */
    private void moveArm() {
        if (armMotor == null) {
            telemetry.addData("Arm", "no motor");
            telemetry.update();
            return;
        }
        Gamepad armGamePad = getArmGamePad();
        if (armGamePad.b) {
            armMotor.setPower(armPower);
        } else if (armGamePad.a) {
            armMotor.setPower(-armPower);
        } else {
            armMotor.setPower(0);
        }
        telemetry.addData("Arm power", "%4.2f", armPower);
        telemetry.update();
    }

    /**
     * Rotate hand
     */
    private void moveServo() {
        Gamepad servoGamePad = getServoGamePad();
        float push = servoGamePad.right_trigger;
        servo.setPosition(push);
        telemetry.addData("Servo", "%4.2f", push);
        telemetry.update();
    }

    /**
     * "Right joystick X axis" to move to the right, positive for a clockwise, and left, negative for counterclockwise
     * "Left joystick Y axis" to go straight, move up, positive, for moving forward, and back, negative to move back.
     * "Left joystick X axis" to strafe, positive left, for moving left, and right, negative to move right.
     *
     * Switch off:
     * - leftFront, rightBack: fwdSpeed and sideSpeed have opposite signs
     * - leftBack, rightFront: fwdSpeed and sideSpeed have same signs
     */
    private void moveWheels() {
        Gamepad wheelGamePad = getWheelGamePad();
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -wheelGamePad.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = wheelGamePad.left_stick_x;
        double yaw = wheelGamePad.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > MAX_POWER) {
            leftFrontPower = leftFrontPower / max * speedCoeff;
            rightFrontPower = rightFrontPower / max * speedCoeff;
            leftBackPower = leftBackPower / max * speedCoeff;
            rightBackPower = rightBackPower / max * speedCoeff;
        }
        setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }

    private void initialize() {
        initializeParameters();
        initializeMotors();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void initializeParameters() {
        speedCoeff = MAX_POWER;
        armPower = MID_POWER;
        position = MIN_POS;
    }

    // Initialize motors (this is typically done in a setup or constructor method)
    private void initializeMotors() {
        leftFrontDrive = getMotor("leftFrontDrive", DcMotor.Direction.REVERSE);
        leftBackDrive = getMotor("leftBackDrive", DcMotor.Direction.REVERSE);
        rightFrontDrive = getMotor("rightFrontDrive", DcMotor.Direction.FORWARD);
        rightBackDrive = getMotor("rightBackDrive", DcMotor.Direction.FORWARD);

        armMotor = getMotor("armMotor", DcMotor.Direction.FORWARD);

        servo = getServo("servoMotor");
    }

    private DcMotor getMotor(String label, DcMotor.Direction direction) {
        DcMotor motor = null;
        try {
            motor = hardwareMap.get(DcMotor.class, label);
            motor.setDirection(direction);
        }
        catch (Exception e) {
            telemetry.addData("Failure", "Motor not found: " + label);
//            telemetry.update();
        }
        return motor;
    }

    private Servo getServo(String label) {
        Servo servo = null;
        try {
            servo = hardwareMap.get(Servo.class, label);
        }
        catch (Exception e) {
            telemetry.addData("Failure", "Servo not found: " + label);
//            telemetry.update();
        }
        return servo;
    }

    // Method to set motor powers
    private void setMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        if (leftFrontDrive != null) {
            leftFrontDrive.setPower(frontLeft);
        }
        if (rightFrontDrive != null) {
            rightFrontDrive.setPower(frontRight);
        }
        if (leftBackDrive != null) {
            leftBackDrive.setPower(backLeft);
        }
        if (rightBackDrive != null) {
            rightBackDrive.setPower(backRight);
        }
    }

}

