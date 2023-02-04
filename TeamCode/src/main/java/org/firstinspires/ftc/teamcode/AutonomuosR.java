package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is an example LinearOpMode that shows how to use a color sensor in a generic
 * way, regardless of which particular make or model of color sensor is used. The Op Mode
 * assumes that the color sensor is configured with a name of "sensor_color".
 *
 * There will be some variation in the values measured depending on the specific sensor you are using.
 *
 * You can increase the gain (a multiplier to make the sensor report higher values) by holding down
 * the A button on the gamepad, and decrease the gain by holding down the B button on the gamepad.
 *
 * If the color sensor has a light which is controllable from software, you can use the X button on
 * the gamepad to toggle the light on and off. The REV sensors don't support this, but instead have
 * a physical switch on them to turn the light on and off, beginning with REV Color Sensor V2.
 *
 * If the color sensor also supports short-range distance measurements (usually via an infrared
 * proximity sensor), the reported distance will be written to telemetry. As of September 2020,
 * the only color sensors that support this are the ones from REV Robotics. These infrared proximity
 * sensor measurements are only useful at very small distances, and are sensitive to ambient light
 * and surface reflectivity. You should use a different sensor if you need precise distance measurements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this Op Mode to the Driver Station OpMode list
 */
@Autonomous(name = "AutoR", group = "Autonomous")
public class AutonomuosR extends LinearOpMode {

  private DcMotor frontLeftDrive = null;
  private DcMotor backLeftDrive = null;
  private DcMotor frontRightDrive = null;
  private DcMotor backRightDrive = null;
  private DcMotor lift = null;
  private CRServo servo = null;
  private DcMotor turnMech = null;
  private TouchSensor frontLimit = null;
  private TouchSensor backLimit = null;

  public void setMotorMode(DcMotor.RunMode runMode){
    frontLeftDrive.setMode(runMode);
    backLeftDrive.setMode(runMode);
    frontRightDrive.setMode(runMode);
    backRightDrive.setMode(runMode);
  }
  public void setAllPower(double power){
    frontLeftDrive.setPower(power);
    frontRightDrive.setPower(power);
    backLeftDrive.setPower(power);
    backRightDrive.setPower(power);
  }
  public void strafe(boolean left, int position){
    setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    if (left == true){
      while (frontRightDrive.getCurrentPosition() < position && opModeIsActive()){
        frontLeftDrive.setPower(-0.3);
        frontRightDrive.setPower(0.3);
        backLeftDrive.setPower(0.3);
        backRightDrive.setPower(-0.3);
      }
    }
    else {
      while (frontLeftDrive.getCurrentPosition() < position && opModeIsActive()) {
        frontLeftDrive.setPower(0.25);
        frontRightDrive.setPower(-0.25);
        backRightDrive.setPower(0.25);
        backLeftDrive.setPower(-0.25);
      }
    }
  }
  public void goToPosition(int position)
  {
    setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

    double currentPosition = 0;
    if (position > 0) {
      while (currentPosition < position && opModeIsActive()) {
        currentPosition = (frontRightDrive.getCurrentPosition() + frontLeftDrive.getCurrentPosition() + backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition()) / 4;
        double pid = (1-currentPosition/position)*(0.25-0.1)+0.1;
        setAllPower(pid);
      }
    }
    else if (position < 0){
      while (currentPosition > position && opModeIsActive()){
        currentPosition = (frontRightDrive.getCurrentPosition() + frontLeftDrive.getCurrentPosition() + backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition()) / 4;
        double pid = (1-currentPosition/position)*(0.25-0.1)+0.1;
        setAllPower(-pid);
      }
    }

    setAllPower(0);
  }
  public void goToPositionPower(int position, double power)
  {
    setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

    double currentPosition = 0;
    if (position > 0) {
      while (currentPosition < position && opModeIsActive()) {
        setAllPower(power);
        currentPosition = (frontRightDrive.getCurrentPosition() + frontLeftDrive.getCurrentPosition() + backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition()) / 4;
        telemetry.addData("Current Position", currentPosition);
      }
    }
    else if (position < 0){
      while (currentPosition > position && opModeIsActive()){
        setAllPower(-power);
        currentPosition = (frontRightDrive.getCurrentPosition() + frontLeftDrive.getCurrentPosition() + backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition()) / 4;
      }
    }

    setAllPower(0);
  }

  /** The colorSensor field will contain a reference to our color sensor hardware object */
  NormalizedColorSensor colorSensor;

  /** The relativeLayout field is used to aid in providing interesting visual feedback
   * in this sample application; you probably *don't* need this when you use a color sensor on your
   * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
  View relativeLayout;

  /**
   * The runOpMode() method is the root of this Op Mode, as it is in all LinearOpModes.
   * Our implementation here, though is a bit unusual: we've decided to put all the actual work
   * in the runSample() method rather than directly in runOpMode() itself. The reason we do that is
   * that in this sample we're changing the background color of the robot controller screen as the
   * Op Mode runs, and we want to be able to *guarantee* that we restore it to something reasonable
   * and palatable when the Op Mode ends. The simplest way to do that is to use a try...finally
   * block around the main, core logic, and an easy way to make that all clear was to separate
   * the former from the latter in separate methods.
   */
  @Override public void runOpMode() {

    frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
    backLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
    frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
    backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
    lift = hardwareMap.get(DcMotor.class, "lift");
    servo = hardwareMap.crservo.get("servo1");
    turnMech = hardwareMap.get(DcMotor.class, "turnMech");
    frontLimit = hardwareMap.touchSensor.get("frontLimit");
    backLimit = hardwareMap.touchSensor.get("backLimit");
    frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
    backRightDrive.setDirection(DcMotor.Direction.FORWARD);
    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);


    // Get a reference to the RelativeLayout so we can later change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    try {
      runSample(); // actually execute the sample
    } finally {
      // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
      // as pure white, but it's too much work to dig out what actually was used, and this is good
      // enough to at least make the screen reasonable again.
      // Set the panel back to the default color
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.WHITE);
        }
      });
    }
  }

  protected void runSample() {
    // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
    // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
    // can give very low values (depending on the lighting conditions), which only use a small part
    // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
    // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
    // colors will report at or near 1, and you won't be able to determine what color you are
    // actually looking at. For this reason, it's better to err on the side of a lower gain
    // (but always greater than  or equal to 1).
    float gain = 2;

    // Once per loop, we will update this hsvValues array. The first element (0) will contain the
    // hue, the second element (1) will contain the saturation, and the third element (2) will
    // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
    // for an explanation of HSV color.
    final float[] hsvValues = new float[3];

    // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
    // state of the X button on the gamepad

    // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
    // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
    // the values you get from ColorSensor are dependent on the specific sensor you're using.
    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "eyes1");

    // If possible, turn the light on in the beginning (it might already be on anyway,
    // we just make sure it is if we can).
    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight)colorSensor).enableLight(true);
    }

    // Wait for the start button to be pressed.
    waitForStart();

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    String rgb = null;
    gain = 2;
    timer.reset();
    timer.startTime();
    while (opModeIsActive() && timer.time() < 0.25){
      servo.setPower(0.5);
    }
    servo.setPower(0);
    timer.reset();
    timer.startTime();
    while (opModeIsActive() && timer.time() < 0.5){
      lift.setPower(-0.5);
    }
    lift.setPower(0);

    // Loop until we are asked to stop
    while(opModeIsActive() && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) > 2 && (-frontRightDrive.getCurrentPosition()+frontLeftDrive.getCurrentPosition()-backLeftDrive.getCurrentPosition()+backRightDrive.getCurrentPosition())/4 < 450){
      frontLeftDrive.setPower(0.25);
      frontRightDrive.setPower(-0.25);
      backLeftDrive.setPower(-0.25);
      backRightDrive.setPower(0.25);
    }
    setAllPower(0);

    timer.reset();
    timer.startTime();
    while (opModeIsActive() && timer.time() < 2) {

      // Explain basic gain information via telemetry
      telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
      telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");



      // Show the gain value via telemetry
      telemetry.addData("Gain", gain);
      colorSensor.setGain(gain);
      // Get the normalized colors from the sensor
      NormalizedRGBA colors = colorSensor.getNormalizedColors();
      // Update the hsvValues array by passing it to Color.colorToHSV()
      Color.colorToHSV(colors.toColor(), hsvValues);
      telemetry.addLine()
              .addData("Red", "%.3f", colors.red)
              .addData("Green", "%.3f", colors.green)
              .addData("Blue", "%.3f", colors.blue);
      telemetry.addLine()
              .addData("Hue", "%.3f", hsvValues[0])
              .addData("Saturation", "%.3f", hsvValues[1])
              .addData("Value", "%.3f", hsvValues[2]);
      telemetry.addData("Alpha", "%.3f", colors.alpha);
      telemetry.addData("timer", timer.time());

      /* If this color sensor also has a distance sensor, display the measured distance.
       * Note that the reported distance is only useful at very close range, and is impacted by
       * ambient light and surface reflectivity. */
      if (colorSensor instanceof DistanceSensor) {
        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
      }

      telemetry.update();

      // Change the Robot Controller's background color to match the color detected by the color sensor.
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
        }
      });
      if (colors.red > colors.blue && colors.red > colors.green){
        rgb = "red";
      }
      else if (colors.blue > colors.green && colors.blue > colors.red){
        rgb = "blue";
      }
      else if (colors.green > colors.red && colors.green > colors.blue){
        rgb = "green";
      }
      else{
        rgb = "???";
      }
    }
    telemetry.addData("Final color:", rgb);
    telemetry.update();
    while(opModeIsActive()){
      strafe(false, 415);
      setAllPower(0);
      break;
    }
    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    while(opModeIsActive() && lift.getCurrentPosition() > -2500){
      lift.setPower(-0.75);
      telemetry.addData("lift encoder", lift.getCurrentPosition());
      telemetry.update();
    }
    lift.setPower(0);
    goToPosition(40);

    timer.reset();
    timer.startTime();
    while (opModeIsActive() && !frontLimit.isPressed() && timer.time() < 1){
      turnMech.setPower(-0.25);
      telemetry.addData("Button pressed?", frontLimit.isPressed());
    }
    turnMech.setPower(0);
    timer.reset();
    timer.startTime();
    while (opModeIsActive() && timer.time() < 2){
      servo.setPower(-0.5);
    }
    servo.setPower(0);
    while(opModeIsActive() && lift.getCurrentPosition() > -200){
      lift.setPower(-0.75);
    }
    goToPosition(-50);
    strafe(true, 180);
    timer.reset();
    timer.startTime();
    while(opModeIsActive() && timer.time() < 2){
      //heheheha
      setAllPower(0);
    }
    if (rgb.equals("red")){
      goToPosition(410);
    }
    else if (rgb.equals("green")){
      goToPosition(-450);
    }
    setAllPower(0);
    while (opModeIsActive() && backLimit.isPressed() == false){
      turnMech.setPower(0.1);
    }
    turnMech.setPower(0);

  }
}
