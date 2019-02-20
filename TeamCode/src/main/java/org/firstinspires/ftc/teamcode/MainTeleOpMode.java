package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MainTeleOpMode", group = "TeleOp")

public class MainTeleOpMode extends OpMode {

    //Creating variables
    private ElapsedTime runtime = new ElapsedTime();

    double leftJoyStick, rightJoyStick, leftMotorPower, rightMotorPower, liftMotorPower, clampMotorPower, leftServoPower, rightServoPower;

    final private static double JOYSTICK_DEADBAND = 0.1;

    //Encoder Ticks Variables
    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
        (WHEEL_DIAMETER_INCHES * 3.1415);


    double motorMovementMin = 0.0;
    double motorMovementMax = 0.0;


    private BotDawg robot;

    @Override
    public void init() {

        robot = new BotDawg();
        robot.init(hardwareMap);
//        robot.scoopMotor.setTargetPosition(0);
    }

    //Code that resets the elapsed time once the driver hits play
    @Override
    public void start() {
        runtime.reset();
    }


    public void loop() {

        //Assigning gamepad values
        leftJoyStick = -gamepad1.left_stick_y;
        rightJoyStick = -gamepad1.right_stick_x;


        //This is for limiting the speed of movement motors
//        if (gamepad1.a && !gamepad1.y){
//            motorMovementMin = -0.85;
//            motorMovementMax = 0.85;
//
//        }else if (gamepad1.y && !gamepad1.a){
//            motorMovementMin = -0.35;
//            motorMovementMax = 0.35;
//        }
        // Code for GamePad2 (Drive Base)
        motorMovementMin = -0.85;
        motorMovementMax = 0.85;

        if (Math.abs(leftJoyStick) < JOYSTICK_DEADBAND) leftJoyStick = 0;
        if (Math.abs(rightJoyStick) < JOYSTICK_DEADBAND) rightJoyStick = 0;

        //Assiging POV drive values

        leftMotorPower = Range.clip(leftJoyStick + rightJoyStick, motorMovementMin, motorMovementMax);
        rightMotorPower = Range.clip(leftJoyStick - rightJoyStick, motorMovementMin, motorMovementMax);

        robot.leftFrontMotor.setPower(leftMotorPower);
        robot.leftBackMotor.setPower(leftMotorPower);
        robot.rightFrontMotor.setPower(rightMotorPower);
        robot.rightBackMotor.setPower(rightMotorPower);


        // Setting the power of the lift motor to the y value of the gamepad1 right joystick

        robot.liftMotor.setPower(gamepad2.right_stick_y);


        // If dpad in gampepad1 is pressed turn the arm
        if(!gamepad2.dpad_down && gamepad2.dpad_up){
            robot.armMotor.setPower(.7);
            robot.armMotor.setTargetPosition(-270);
        }else if (gamepad2.dpad_down && !gamepad2.dpad_up){
            robot.armMotor.setPower(.7);
            robot.armMotor.setTargetPosition(270);

        }else{
            robot.armMotor.setPower(0);
        }


        //Setting the power of the intake motor to 1
        if(gamepad2.a){
            robot.intakeMotor.setPower(1);
        }
        if(gamepad2.b){
            robot.intakeMotor.setPower(0);
        }
        if(gamepad2.y){
            robot.intakeMotor.setPower(-1);
        }


        if(gamepad2.right_bumper && !gamepad2.left_bumper){
            robot.lockMotor.setPower(1);
            robot.lockMotor.setTargetPosition(-10);

        }else if(!gamepad2.right_bumper && gamepad2.left_bumper){
            robot.lockMotor.setPower(-1);
            robot.lockMotor.setTargetPosition(10);
        }




        //Telemetry is not used to control the robot, it is purely to help debug by showing
        //Information on the phone
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)",leftMotorPower, rightMotorPower);
        telemetry.addData("Value of joystick = ", gamepad2.right_stick_y);

    }
}