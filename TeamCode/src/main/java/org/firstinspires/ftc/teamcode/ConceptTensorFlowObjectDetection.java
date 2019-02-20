/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name = "Depot", group = "Concept")
//@Disabled
public class ConceptTensorFlowObjectDetection extends LinearOpMode {

  // The IMU sensor object

  // State used for updating telemetry
  Orientation angles;
  Acceleration gravity;

  /* Declare OpMode members. */
  private BotDawg robot = new BotDawg();   // Use a Pushbot's hardware
  private ElapsedTime runtime = new ElapsedTime();


  private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
  private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
  private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
  private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
          (WHEEL_DIAMETER_INCHES * 3.1415);
  static final double     DRIVE_SPEED             = 0.3;
  static final double     TURN_SPEED              = 0.3;
  static final double     Lift_Speed              = 0.4;

  private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
  private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
  private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

  /*
   * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
   * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
   * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
   * web site at https://developer.vuforia.com/license-manager.
   *
   * Vuforia license keys are always 380 characters long, and look as if they contain mostly
   * random data. As an example, here is a example of a fragment of a valid key:
   *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
   * Once you've obtained a license key, copy the string from the Vuforia web site
   * and paste it in to your code on the next line, between the double quotes.
   */
  private static final String VUFORIA_KEY = "AWmj6yz/////AAABmfWffA2XHkI1inDJh0chZecmL6yY3yRHSKsD9+mfYbQVgp2U7OpD5dl+bEAsi9S0X250X0V5q8k+euywF8obWxxwH/R2vhgOECD7x/i2I22NgD8mXR83z9riqwOdPuTEOyKXZsYLLPExzxzYPn+X+J1w7qarT/5B32XXq5OfrGDr4Ut3EimQWePJavRb6Drhiki7rnbJz6Q1DETY51gyvpy07jFSxImkNnMEcYvGMcOHZU79s4eSPmaoCx41ftp/v48AqGE2oEOVWo1WD15MuBG7i11qVpYoM4KKFf13DS5hABRQpRSYyj5HVC67kwGvub2tUruOwVxZOxoBax+ETFpfNvHFbz2so/yipxWNXUEG";

  /**
   * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
   * localization engine.
   */
  private VuforiaLocalizer vuforia;

  /**
   * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
   * Detection engine.
   */
  private TFObjectDetector tfod;

  @Override
  public void runOpMode() {

    robot.init(hardwareMap);
    // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
    // first.
    initVuforia();

    if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
      initTfod();
    } else {
      telemetry.addData("Sorry!", "This device is not compatible with TFOD");
    }

    /** Wait for the game to begin */
    telemetry.addData(">", "Press Play to start tracking");
    telemetry.update();
    waitForStart();
    robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    if (opModeIsActive()) {
      /** Activate Tensor Flow Object Detection. */
      if (tfod != null) {
        tfod.activate();
      }

      while (opModeIsActive()) {
        if (tfod != null) {
          // getUpdatedRecognitions() will return null if no new information is available since
          // the last time that call was made.
          List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
          if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            if (updatedRecognitions.size() == 2) {
              int goldMineralX = -1;
              int silverMineral1X = -1;
              int silverMineral2X = -1;
              for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                  goldMineralX = (int) recognition.getLeft();
                } else if (silverMineral1X == -1) {
                  silverMineral1X = (int) recognition.getLeft();
                } else {
                  silverMineral2X = (int) recognition.getLeft();
                }
              }
              telemetry.addData("MIneral",goldMineralX);
              telemetry.addData("MIneral",silverMineral1X);
                if (goldMineralX == -1) {
                  telemetry.addData("Gold Mineral Position", "Left");
                  leftPath();
                } else if (goldMineralX > silverMineral1X) {
                  telemetry.addData("Gold Mineral Position", "Right");
                  rightPath();
                } else {
                  telemetry.addData("Gold Mineral Position", "Center");
                  centerPath();
                }
            } else{
                telemetry.addData("Default", "center");
            }
            telemetry.update();
          }
        }
      }
    }

    if (tfod != null) {
      tfod.shutdown();
    }
  }
  String formatAngle(AngleUnit angleUnit, double angle) {
    return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
  }

  String formatDegrees(double degrees){
    return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
  }

  public void encoderDrive(double speed,
                           double leftInches, double rightInches,
                           double timeoutS) {
    int newLeftFrontTarget;
    int newRightFrontTarget;
    int newLeftBackTarget;
    int newRightBackTarget;

    // Ensure that the opmode is still active
    if (opModeIsActive()) {

      // Determine new target position, and pass to motor controller
      newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
      newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
      newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
      newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

      robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
      robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
      robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
      robot.rightBackMotor.setTargetPosition(newRightBackTarget);

      // Turn On RUN_TO_POSITION
      robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset the timeout time and start motion.
      runtime.reset();
      robot.leftFrontMotor.setPower(Math.abs(speed));
      robot.rightFrontMotor.setPower(Math.abs(speed));
      robot.leftBackMotor.setPower(Math.abs(speed));
      robot.rightBackMotor.setPower(Math.abs(speed));

      // keep looping while we are still active, and there is time left, and both motors are running.
      // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
      // its target position, the motion will stop.  This is "safer" in the event that the robot will
      // always end the motion as soon as possible.
      // However, if you require that BOTH motors have finished their moves before the robot continues
      // onto the next step, use (isBusy() || isBusy()) in the loop test.
      while (opModeIsActive() &&
              (runtime.seconds() < timeoutS) &&
              (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

        // Display it for the driver.
      }
      // Stop all motion;
      robot.leftFrontMotor.setPower(0);
      robot.rightFrontMotor.setPower(0);
      robot.leftBackMotor.setPower(0);
      robot.rightBackMotor.setPower(0);
      // Turn off RUN_TO_POSITION
      robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      //  sleep(250);   // optional pause after each move
    }
  }

  public void encoderLift(double speed,
                          double yInches,
                          double timeoutS) {
    int newYTarget;


    // Ensure that the opmode is still active
    if (opModeIsActive()) {

      // Determine new target position, and pass to motor controller
      newYTarget = robot.liftMotor.getCurrentPosition() + (int)(yInches * COUNTS_PER_INCH);


      robot.liftMotor.setTargetPosition(newYTarget);

      // Turn On RUN_TO_POSITION
      robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


      // reset the timeout time and start motion.
      runtime.reset();
      robot.liftMotor.setPower(Math.abs(speed));


      while (opModeIsActive() &&
              (runtime.seconds() < timeoutS) &&
              (robot.liftMotor.isBusy())) {

        // Display it for the driver.

      }
      // Stop all motion;
      robot.liftMotor.setPower(0);

      // Turn off RUN_TO_POSITION
      robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
  }

  public void turn(float degrees) {
    Orientation angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    float degreesMoved = 0;
    float direction = Math.signum(degrees);
    boolean done = false;
    float lastAngle = angles.firstAngle;
    while (opModeIsActive() && !done ) {
      angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      telemetry.addData("first angle", angles.firstAngle);
      telemetry.addData("target", degrees);
      telemetry.update();
      robot.leftFrontMotor.setPower(direction * TURN_SPEED);
      robot.rightFrontMotor.setPower(-direction * TURN_SPEED);
      robot.leftBackMotor.setPower(-direction * TURN_SPEED);
      robot.rightBackMotor.setPower(direction * TURN_SPEED);
      float currentAngle = angles.firstAngle;
      if(currentAngle - lastAngle > 90 ){
        currentAngle -= 360;
      }else if(currentAngle - lastAngle < -90){
        currentAngle += 360;
      }
      degreesMoved += currentAngle - lastAngle;
      if(direction < 0){
        done = degreesMoved < degrees;
      }else if(direction > 0) {
        done = degreesMoved > degrees;
      }
      lastAngle = currentAngle;
    }		robot.leftFrontMotor.setPower(0);
    robot.rightFrontMotor.setPower(Math.abs(0));
    robot.leftBackMotor.setPower(Math.abs(0));
    robot.rightBackMotor.setPower(Math.abs(0));
  }



    public void leftPath(){
        if (opModeIsActive()) {
            //            robot.lockMotor.setPower(0);
            encoderLift(Lift_Speed, 42,2);
            robot.lockMotor.setPower(-1);
            robot.lockMotor.setTargetPosition(-10);
            runtime.reset();
            while (opModeIsActive() &&
                    (runtime.seconds() < 1)
                    ) {
                telemetry.addData("encoder",robot.lockMotor.getCurrentPosition());
                telemetry.update();
            }
            telemetry.addData("centerPath", null);
            encoderLift(Lift_Speed, -6,3);
            telemetry.addData("encoder",robot.lockMotor.getTargetPosition());
            telemetry.update();
            sleep(100);
            encoderDrive(DRIVE_SPEED,-2,-2,2);
            sleep(100);
            encoderLift(Lift_Speed,2.5,1);
            sleep(500);
            turn(25);
            sleep(500);
            encoderDrive(DRIVE_SPEED,-8,-8,4);
            sleep(500);
            turn(-30);
            sleep(100);
            encoderDrive(DRIVE_SPEED,-10,-10,3);
//            turn(90);
//            sleep(500);
//            encoderDrive(DRIVE_SPEED, 20,20,2);
//            sleep(50);
//            robot.markerServo.setPosition(0);
//            sleep(100);
//            encoderDrive(DRIVE_SPEED, 5,5,2);
//            turn(10);


            // Going to the other alliance crater
//            sleep(200);
//            turn(135);
//            sleep(100);
//            encoderDrive(DRIVE_SPEED, -27,-27,10);
        }
    }
    public void rightPath(){
        if (opModeIsActive()) {
            //            robot.lockMotor.setPower(0);
            encoderLift(Lift_Speed, 3,2);
            robot.lockMotor.setPower(-1);
            robot.lockMotor.setTargetPosition(-10);
            runtime.reset();
            while (opModeIsActive() &&
                    (runtime.seconds() < 1)
                    ) {
                telemetry.addData("encoder",robot.lockMotor.getCurrentPosition());
                telemetry.update();
            }
            telemetry.addData("centerPath", null);
            encoderLift(Lift_Speed, -6,3);
            telemetry.addData("encoder",robot.lockMotor.getTargetPosition());
            telemetry.update();
            sleep(100);
            encoderDrive(DRIVE_SPEED,-3,-3,2);
            sleep(100);
            encoderLift(Lift_Speed,2.5,1);
            sleep(500);
            turn(-35);
            sleep(500);
            encoderDrive(DRIVE_SPEED,-15,-15,6);
            sleep(500);
            turn(60);
            sleep(500);
            encoderDrive(DRIVE_SPEED, -6,-6 ,2);
//            sleep(50);
//            robot.markerServo.setPosition(0);
//            sleep(100);
//            encoderDrive(DRIVE_SPEED,10,10,4);
//            sleep(100);
//            encoderDrive(DRIVE_SPEED, 10,10,3);
            // Going to the other alliance crater
//            sleep(200);
//            turn(80);
//            sleep(100);
//            encoderDrive(DRIVE_SPEED, -27,-27,10);
        }
    }
    public void centerPath(){
        if (opModeIsActive()) {

            //            robot.lockMotor.setPower(0);
            encoderLift(Lift_Speed, 42,2);
            robot.lockMotor.setPower(-1);
            robot.lockMotor.setTargetPosition(-10);
            runtime.reset();
            while (opModeIsActive() &&
                    (runtime.seconds() < 1)
                    ) {
                telemetry.addData("encoder",robot.lockMotor.getCurrentPosition());
                telemetry.update();
            }
            telemetry.addData("centerPath", null);
            encoderLift(Lift_Speed, -6,3);
            telemetry.addData("encoder",robot.lockMotor.getTargetPosition());
            telemetry.update();
            sleep(100);
            encoderLift(Lift_Speed,2.5,1);
            sleep(100);
            encoderDrive(DRIVE_SPEED,-21,-21,10);

            // Going to the other alliance crater
            sleep(100);
            robot.markerServo.setPosition(0);
            sleep(100);
            encoderDrive(DRIVE_SPEED,10,10,10);

//            turn(130);
//            sleep(100);
//            encoderDrive(DRIVE_SPEED, -27,-27,10);

        }
    }

  String format(OpenGLMatrix transformationMatrix) {
    return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
  }
  /**
   * Initialize the Vuforia localization engine.
   */
  private void initVuforia() {
    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     */
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraDirection = CameraDirection.BACK;

    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);

    // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
  }

  /**
   * Initialize the Tensor Flow Object Detection engine.
   */
  private void initTfod() {
    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
  }
}
