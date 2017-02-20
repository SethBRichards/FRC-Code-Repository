/*

   THE MIT LICENSE

   Copyright 2017 TEAM 3882 [Tad Luckey, Randy Arakawa, Jay Marie Baptista, Seth Richards, Diolo Pascual]

   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
   documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
   persons to whom the Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
   Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
   WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
   COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

package org.usfirst.frc.team3882.robot;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.kauailabs.navx.frc.AHRS;

public class Robot extends IterativeRobot {
final String defaultAuto = "Default";
final String customAuto = "My Auto";
Integer autoSelected;
SendableChooser<Integer> chooser;
Joystick jsLeft, jsRight;
XboxController xbox;
Compressor compressor;
Talon rightFront, rightMid, rightBack, leftFront, leftMid, leftBack;
Encoder encLeftDrive, encRightDrive;
NetworkTable table;
DoubleSolenoid driveChain, gearpiston, presser;
CANTalon climber, shooter, intake, feeder;
AHRS navx;

//JOYSTICKS

double rightJS, leftJS;
double rightJSAxis, leftJSAxis;

// BUTTONS
boolean rBTN1, rBTN2, rBTN3, rBTN4, rBTN5, lBTN1, lBTN2, lBTN3, lBTN4, lBTN5 = false;
boolean xBTNA, xBTNB, xBTNX, xBTNY, xBTN5, xBTN4;

//LATCH BOOLEANS
boolean rBTN1_old, rBTN3_old, lBTN1_old, shootLatch, lowLatch = false;

boolean xBTN5_old, xBTN4_old, xBTNA_old, xBTNY_old;

// PNEUMATIC CONTROLLER MODULE
final int pcm0 = 0;

// PULSE WIDTH MODS
final int pwm0 = 0;
final int pwm1 = 1;
final int pwm2 = 2;
final int pwm3 = 3;
final int pwm4 = 4;
final int pwm5 = 5;

//STATE MACHINE VARIABLES
double encLeftDriveDistance;
double nvYaw;
int state;

//AUTONOMOUS
double targetAquired;
double targetCenterDistance;

Victor secondShooterMotor;
PowerDistributionPanel pdp;
int condition;

final int usb0 = 0;
double motorPower = 0.0;
double talon2 = 0.0;
double talon1 = 0.0;
double shooterRPM = 0.0;

final int conversionFactor = (1080);

@Override
public void robotInit() {

        chooser = new SendableChooser<Integer>();
        chooser.addDefault("center red", 1);
        chooser.addObject("center blue", 2);
        chooser.addObject("boiler red", 3);
        chooser.addObject("boiler blue", 4);
        chooser.addObject("ret red", 5);
        chooser.addObject("ret blue", 6);
        chooser.addObject("current test", 7);

        SmartDashboard.putData("Auto choices", chooser);

        //NETWORK TABLE VARIABLES
        table = NetworkTable.getTable("dataTable");

        //POWER DIST PANEL
        pdp = new PowerDistributionPanel();

        //NAVX
        navx = new AHRS(SPI.Port.kMXP);

        // CONTROLLER
        jsLeft = new Joystick(0);
        jsRight = new Joystick(1);
        xbox = new XboxController(5);

        // MOTORS
        leftFront = new Talon(pwm5);
        leftMid = new Talon(pwm3);
        leftBack = new Talon(pwm1);
        rightFront = new Talon(pwm4);
        rightMid = new Talon(pwm2);
        rightBack = new Talon(pwm0);

        // ENCODERS
        encLeftDrive = new Encoder(0,1);
        encRightDrive = new Encoder(2,3);

        // COMPRESSOR
        compressor = new Compressor();
        compressor.start();

        //SOLENOIDs
        driveChain = new DoubleSolenoid(0,1);
        driveChain.set(Value.kReverse);
        presser = new DoubleSolenoid(2,3);
        presser.set(Value.kReverse);
        gearpiston = new DoubleSolenoid(4,5);
        gearpiston.set(Value.kReverse);


        //CANTALONS
        climber = new CANTalon(2);
        shooter = new CANTalon(4);
        intake = new CANTalon(9);
        feeder = new CANTalon(13);

        // CAMERA
        UsbCamera usbCam = new UsbCamera("mscam", 0);
        usbCam.setVideoMode(VideoMode.PixelFormat.kMJPEG, 160, 120, 20);
        MjpegServer server1 = new MjpegServer("cam1", 1181);
        server1.setSource(usbCam);
}

@Override
public void autonomousInit() {
        autoSelected = chooser.getSelected();
        System.out.println("Auto selected: " + autoSelected);
        state = 0;
        condition = 0;
        encLeftDrive.reset();
        navx.reset();
}
@Override
public void autonomousPeriodic() {
        encLeftDriveDistance = encLeftDrive.get();
        nvYaw = navx.getAngle();

        talon1 = pdp.getCurrent(1);
        SmartDashboard.putNumber("CURRENT2_T1", talon1);

        System.out.println("Auto Running: Ticks = " + encLeftDriveDistance);
        switch(autoSelected)
        {
        case 1:
        {
                centreRed();
                break;
        }
        case 2:
        {
                centreBlue();
                break;
        }
        case 3:
        {
                boilerRed();
                break;
        }
        case 4:
        {
                boilerBlue();
                break;
        }
        case 5:
        {
                retRed();
                break;
        }
        case 6:
        {
                retBlue();
                break;
        }
        case 7:
        {
                testAutonomous();
                break;
        }


        default:
        {
                //Put default auto code here

                break;
        }
        }
}


@Override
public void teleopPeriodic() {
        //TARGETING
        targetCenterDistance = table.getNumber("3882_DELTA", -4545.45);
        targetAquired = table.getNumber("3882_TARGET_FOUND", -3535.35);
        System.out.println("GOAL CENTER" + targetCenterDistance);
        encLeftDriveDistance = encLeftDrive.get();
        System.out.println("QUADRATURE" + encLeftDrive.get());
        nvYaw = navx.getYaw();

        // JS CONTROLS
        rightJSAxis = -1 * jsRight.getRawAxis(1);
        leftJSAxis = jsLeft.getRawAxis(1);
        rBTN1 = jsRight.getRawButton(1);
        rBTN2 = jsRight.getRawButton(2);
        rBTN3 = jsRight.getRawButton(3);
        rBTN4 = jsRight.getRawButton(4);
        rBTN5 = jsRight.getRawButton(5);
        lBTN1 = jsLeft.getRawButton(1);
        lBTN2 = jsLeft.getRawButton(2);
        lBTN3 = jsLeft.getRawButton(3);
        lBTN4 = jsLeft.getRawButton(4);
        lBTN5 = jsLeft.getRawButton(5);
        xBTNX = xbox.getXButton();
        xBTNY = xbox.getYButton();
        xBTNA = xbox.getAButton();
        xBTNB = xbox.getBButton();
        xBTN5 = xbox.getRawButton(5);

        //OPERATOR CONTROLS

        //PRESSER
        if (xBTN4 == true && xBTN4_old == false) {
          presser.set(Value.kForward);
        }
        else {
          presser.set(Value.kReverse);
        }
        xBTN4_old = xBTN4;

        //REDUCER
        if (xBTN5 == true && xBTN5_old == false) {
                lowLatch = !lowLatch;
        }
        xBTN5_old = xBTN5;

        //SHOOTER SPEED CHANGER
        if (xBTNY && !xBTNY_old) {
          motorPower = motorPower + 0.05;
        }
        else if (xBTNA && !xBTNA_old) {
          motorPower = motorPower - 0.05;
        }
        else if (xBTNB){
          motorPower = -0.8;
        }
        else if (xBTNX) {
          motorPower = 0.0;
        }

        shooter.set(motorPower);

        //PILOT CONTROLS

        //ALIGNER
        if (rBTN2) {
                if(targetAquired == 1.0)
                {
                        if (targetCenterDistance > 15)
                        {
                                leftFront.set(0.25);
                                leftMid.set(0.25);
                                leftBack.set(0.25);
                                rightFront.set(0.25);
                                rightMid.set(0.25);
                                rightBack.set(0.25);
                        }
                        else if (targetCenterDistance < -15)
                        {
                                leftFront.set(-0.25);
                                leftMid.set(-0.25);
                                leftBack.set(-0.25);
                                rightFront.set(-0.25);
                                rightMid.set(-0.25);
                                rightBack.set(-0.25);
                        }

                }
        }
        // REDUCTION CHAIN
        else if (lowLatch) {
                leftFront.set(leftJSAxis * .85);
                leftMid.set(leftJSAxis * .85);
                leftBack.set(leftJSAxis * .85);
                rightFront.set(rightJSAxis * .85);
                rightMid.set(rightJSAxis * .85);
                rightBack.set(rightJSAxis * .85);
        }
        // DEFAULT TO JOYSTICKS
        else {
                leftFront.set(leftJSAxis);
                leftMid.set(leftJSAxis);
                leftBack.set(leftJSAxis);
                rightFront.set(rightJSAxis);
                rightMid.set(rightJSAxis);
                rightBack.set(rightJSAxis);
        }

        //INTAKE IN LATCHED
        if (rBTN4) {
        	intake.set(0.7);
        }
        
        //INTAKE OUT LATCHED
        if (rBTN5) {
        	intake.set(0.0);
        }


        //SHIFTER LATCHED
        if (rBTN1 == true && rBTN1_old == false)
        {
                if(driveChain.get() == Value.kForward)
                {
                        driveChain.set(Value.kReverse);
                }
                else
                {
                        driveChain.set(Value.kForward);
                }
        }
        rBTN1_old = rBTN1;

        //SHOOTER LATCHED
        if (lBTN1 == true && lBTN1_old == false)
        {
                shootLatch = !shootLatch;
        }
        if(shootLatch)
        {
                feeder.set(-0.8);
        }
        else
        {
                feeder.set(-0.0);
        }
        lBTN1_old = lBTN1;

        //gearpiston BUTTON (LATCHED)
        if (rBTN3 == true && rBTN3_old == false)
        {
                if(gearpiston.get() == Value.kForward)
                {
                        gearpiston.set(Value.kReverse);
                }
                else
                {
                        gearpiston.set(Value.kForward);
                }
        }
        rBTN3_old = rBTN3;

        //CLIMBER BUTTONS
        if (lBTN3) {
                climber.set(-0.8);
        }
        else if (lBTN2) {
                climber.set(0.8);
        }
        else {
                climber.set(0.0);
        }

        SmartDashboard.putString("SHOOTER LATCH", "STATE" + shootLatch);
        SmartDashboard.putString("MOTOR LATCH", "STATE" + xBTN5_old);
        //shooter.set(motorPower);
        //secondShooterMotor.set(motorPower);
        shooterRPM = shooter.getPulseWidthVelocity() * (600.0/4096) * -1;
        talon2 = pdp.getCurrent(0);
        talon1 = pdp.getCurrent(1);
        System.out.println("ANGLE" + navx.getAngle());

        //SmartDashboard.putNumber("RPM", shooterRPM);
        //System.out.println(shooterPwr);
        SmartDashboard.putNumber("CURRENT T2", talon2);
        SmartDashboard.putNumber("CURRENT T1", talon1);
        SmartDashboard.putNumber("OUTPUT", motorPower);
        SmartDashboard.putNumber("speed", leftFront.get());
}


@Override
public void testPeriodic() {
}

//****************************STATE MACHINE COMMANDS*************
//AUTONOMOUS MOTION TURNING(Negative : Left, Positive : Right -- relative to front)

//ANGULAR MODULE
public int turnDirection(double c)
{
        int result = 0;
        if (c < 0)
        {
                if(nvYaw > c)
                {
                        rightFront.set(0.55);
                        rightMid.set(0.55);
                        rightBack.set(0.55);
                        leftFront.set(0.55);
                        leftMid.set(0.55);
                        leftBack.set(0.55);
                }
                else
                {
                        result = 1;

                }
        }
        else if (c > 0)
        {
                if(nvYaw < c)
                {
                        rightFront.set(-0.55);
                        rightMid.set(-0.55);
                        rightBack.set(-0.55);
                        leftFront.set(-0.55);
                        leftMid.set(-0.55);
                        leftBack.set(-0.55);
                }
                else
                {
                        result = 1;
                }
        }
        return result;
}

//DIRECTIONAL MODULE
public int straightDirection(double x)
{
        int result = 0;
        if (x * conversionFactor < 0)
        {
                if(encLeftDriveDistance >  x * conversionFactor)
                {
                        rightFront.set(-0.35);
                        rightMid.set(-0.35);
                        rightBack.set(-0.35);
                        leftFront.set(0.35);
                        leftMid.set(0.35);
                        leftBack.set(0.35);
                }
                else
                {
                        result = 1;
                }
        }
        else
        {
                if(encLeftDriveDistance <  x * conversionFactor)
                {
                        rightFront.set(0.35);
                        rightMid.set(0.35);
                        rightBack.set(0.35);
                        leftFront.set(-0.35);
                        leftMid.set(-0.35);
                        leftBack.set(-0.35);
                }
                else
                {
                        result = 1;
                }
        }
        return result;
}

//STOP MODULE
public void stopping()
{
        encLeftDrive.reset();
        navx.reset();
        rightFront.set(0.0);
        rightMid.set(0.0);
        rightBack.set(0.0);
        leftFront.set(-0.0);
        leftMid.set(-0.0);
        leftBack.set(-0.0);
}

//CURRENT-STOP MODULE
public int currentTest() {
        int result = 0;

        if (condition == 1)
        {
                leftFront.set(-0);
                leftMid.set(-0);
                leftBack.set(-0);
                rightFront.set(0);
                rightMid.set(0);
                rightBack.set(0);
                result = 1;
        }
        else if (pdp.getCurrent(1) < 18.5) {
                leftFront.set(-0.5);
                leftMid.set(-0.5);
                leftBack.set(-0.5);
                rightFront.set(0.5);
                rightMid.set(0.5);
                rightBack.set(0.5);

        }
        else if ( pdp.getCurrent(1) >= 18.5) {
                condition = 1;

        }

        return result;

}

//ALIGNER MODULE
public int targeting()
{
        int result = 0;
        if(targetAquired == 1.0)
        {
                if (targetCenterDistance > 15)
                {
                        leftFront.set(0.25);
                        leftMid.set(0.25);
                        leftBack.set(0.25);
                        rightFront.set(0.25);
                        rightMid.set(0.25);
                        rightBack.set(0.25);
                }
                else if (targetCenterDistance < -15)
                {
                        leftFront.set(-0.25);
                        leftMid.set(-0.25);
                        leftBack.set(-0.25);
                        rightFront.set(-0.25);
                        rightMid.set(-0.25);
                        rightBack.set(-0.25);
                }
                if (targetCenterDistance < 15 && targetCenterDistance > -15)
                {
                        result = 1;
                }
        }
        return result;
}


/* AUTONOMOUS FUNCTIONS BASED ON POSITION AND ALLIANCE */

public void centreRed()
{
        targetCenterDistance = table.getNumber("3882_DELTA", -4545.45);
        targetAquired = table.getNumber("3882_TARGET_FOUND", -3535.35);


        if ( state == 0 && straightDirection(-4.2) == 1)
        {
                state = 1;
        }
        else if (state == 1)
        {
                stopping();
                state = 2;
        }
        else if (state == 2)
        {
                //if (targeting() == 1)
                //{
                state = 3;
                //}
        }

        else if (state == 3)
        {
                stopping();
                gearpiston.set(Value.kForward);
                state = 4;
        }
        else if (state == 4 && straightDirection(-2)==1)
        {
                state = 5;
        }
        else if (state == 5)
        {
                stopping();
                state = 6;
        }
        else if (state == 6)
        {
                state = 7;

        }
        else if (state == 7)
        {
                stopping();
                state = 8;
        }
        else if (state == 8 && straightDirection(2.5 )== 1)
        {
                state = 9;
        }
        else if (state == 9)
        {
                stopping();
                state = 10;

        }
        else if (state == 10)//Current thingy here
        {
                state = 11;
        }
        else if (state == 11)
        {
                stopping();
                state = 12;

        }
        else if (state == 12 && straightDirection(4)== 1)
        {
                state = 14;

        }
        else if (state == 14)
        {
                stopping();
                state = 15;
        }
        else if (state == 15)
        {

                state = 16;
        }
        else if (state == 16)
        {
                stopping();
        }
        System.out.println("ticks" + encLeftDrive.get());
        System.out.println("state" + state);
        System.out.println("targetcenterdistance             " + targetCenterDistance);

}
public void centreBlue()
{
        targetCenterDistance = table.getNumber("3882_DELTA", -4545.45);
        targetAquired = table.getNumber("3882_TARGET_FOUND", -3535.35);


        if ( state == 0 && straightDirection(-4.2) == 1)
        {
                state = 1;
        }
        else if (state == 1)
        {
                stopping();
                state = 2;
        }
        else if (state == 2)
        {
                //if (targeting() == 1)
                //{
                state = 3;
                //}
        }

        else if (state == 3)
        {
                stopping();
                gearpiston.set(Value.kForward);
                state = 4;
        }
        else if (state == 4 && straightDirection(-2)==1)
        {
                state = 5;
        }
        else if (state == 5)
        {
                stopping();
                state = 6;
        }
        else if (state == 6)
        {
                state = 7;

        }
        else if (state == 7)
        {
                stopping();
                state = 8;
        }
        else if (state == 8 && straightDirection(2 )== 1)
        {
                state = 9;
        }
        else if (state == 9)
        {
                stopping();
                state = 10;

        }
        else if (state == 10 && turnDirection(167)==1)
        {
                state = 11;
        }
        else if (state == 11)
        {
                stopping();
                state = 12;

        }
        else if (state == 12 )//Current thingy here
        {
                state = 14;

        }
        else if (state == 14)
        {
                stopping();
                state = 15;
        }
        else if (state == 15)
        {

                state = 16;
        }
        else if (state == 16)
        {
                stopping();
        }
        System.out.println("ticks" + encLeftDrive.get());
        System.out.println("state" + state);
        System.out.println("targetcenterdistance             " + targetCenterDistance);
}

public void boilerRed()
{
        targetCenterDistance = table.getNumber("3882_DELTA", -4545.45);
        targetAquired = table.getNumber("3882_TARGET_FOUND", -3535.35);

        if ( state == 0 && straightDirection(-7.12 ) == 1)
        {
                state = 1;
        }
        else if (state == 1)
        {
                stopping();
                state = 2;
        }
        else if (state == 2 && turnDirection(-27 ) == 1)
        {
                state = 3;

        }
        else if (state == 3)
        {
                stopping();
                state = 4;
        }
        else if (state == 4 )
        {
                //TARGETING HERE
                gearpiston.set(Value.kForward);
                state = 5;
        }
        else if (state == 5)
        {
                stopping();
                state = 6;
        }
        else if (state == 6  && straightDirection(-1.1  )== 1)
        {
                state = 7;

        }
        else if (state == 7)
        {
                stopping();
                state = 8;
        }
        else if (state == 8 && straightDirection(2  )== 1)
        {
                state = 9;
        }
        else if (state == 9)
        {
                stopping();
                state = 10;

        }
        else if (state == 10 && turnDirection(80)==1)
        {
                state = 11;
        }
        else if (state == 11)
        {
                stopping();
                state = 12;

        }
        else if (state == 12 )
        {
                state = 14;

        }
        else if (state == 14)
        {
                stopping();
                state = 15;
        }
        else if (state == 15 )
        {

                state = 16;
        }
        else if (state == 16)
        {
                stopping();
        }
        System.out.println("ticks" + encLeftDrive.get());
        System.out.println("state" + state);
        System.out.println("targetcenterdistance             " + targetCenterDistance);
}
public void boilerBlue()
{
        targetCenterDistance = table.getNumber("3882_DELTA", -4545.45);
        targetAquired = table.getNumber("3882_TARGET_FOUND", -3535.35);

        if ( state == 0 && straightDirection(-7.12 ) == 1)
        {
                state = 1;
        }
        else if (state == 1)
        {
                stopping();
                state = 2;
        }
        else if (state == 2 && turnDirection(27 ) == 1)
        {
                state = 3;

        }
        else if (state == 3)
        {
                stopping();
                state = 4;
        }
        else if (state == 4 )
        {
                //TARGETING HERE
                gearpiston.set(Value.kForward);
                state = 5;
        }
        else if (state == 5)
        {
                stopping();
                state = 6;
        }
        else if (state == 6  && straightDirection(-1.1  )== 1)
        {
                state = 7;

        }
        else if (state == 7)
        {
                stopping();
                state = 8;
        }
        else if (state == 8 && straightDirection(2  )== 1)
        {
                state = 9;
        }
        else if (state == 9)
        {
                stopping();
                state = 10;

        }
        else if (state == 10 && turnDirection(38)==1)
        {
                state = 11;
        }
        else if (state == 11)
        {
                stopping();
                state = 12;

        }
        else if (state == 12 && straightDirection(1)==1)
        {
                state = 14;

        }
        else if (state == 14)
        {
                stopping();
                state = 15;
        }
        else if (state == 15 )
        {

                state = 16;
        }
        else if (state == 16)
        {
                stopping();
        }
        System.out.println("ticks" + encLeftDrive.get());
        System.out.println("state" + state);
        System.out.println("targetcenterdistance             " + targetCenterDistance);
}

public void retRed()
{
        targetCenterDistance = table.getNumber("3882_DELTA", -4545.45);
        targetAquired = table.getNumber("3882_TARGET_FOUND", -3535.35);
        if ( state == 0 && straightDirection(-7.0 ) == 1)
        {
                state = 1;
        }
        else if (state == 1)
        {
                stopping();
                state = 2;
        }
        else if (state == 2 && turnDirection(47) == 1)
        {
                state = 3;
        }
        else if (state == 3)
        {
                stopping();
                state = 4;
        }
        else if (state == 4 )
        {
                gearpiston.set(Value.kForward);
                //TARGETING GOES HERE
                state = 5;
        }
        else if (state == 5)
        {
                stopping();
                state = 6;
        }
        else if (state == 6 && straightDirection(-1.8 ) == 1)
        {
                state = 7;

        }
        else if (state == 7)
        {
                stopping();
                state = 8;
        }
        else if (state == 8 && straightDirection(2.5  )== 1)
        {
                state = 9;
        }
        else if (state == 9)
        {
                stopping();
                state = 10;

        }
        else if (state == 10 && turnDirection(-56 ) == 1)
        {
                state = 11;
        }
        else if (state == 11)
        {
                stopping();
                state = 12;

        }
        else if (state == 12 && straightDirection(-1  )== 1)
        {
                state = 14;

        }
        else if (state == 14)
        {
                stopping();
                state = 15;
        }
        else if (state == 15 )
        {

                state = 16;
        }
        else if (state == 16)
        {
                stopping();
        }
        System.out.println("ticks" + encLeftDrive.get());
        System.out.println("state" + state);
        System.out.println("targetcenterdistance             " + targetCenterDistance);
}
public void retBlue()
{
        targetCenterDistance = table.getNumber("3882_DELTA", -4545.45);
        targetAquired = table.getNumber("3882_TARGET_FOUND", -3535.35);
        if ( state == 0 && straightDirection(-5.2 ) == 1)
        {
                state = 1;
        }
        else if (state == 1)
        {
                stopping();
                state = 2;
        }
        else if (state == 2 && turnDirection(54) == 1)
        {
                state = 3;
        }
        else if (state == 3)
        {
                stopping();
                state = 4;
        }
        else if (state == 4 )
        {
                gearpiston.set(Value.kForward);
                //TARGETING GOES HERE
                state = 5;
        }
        else if (state == 5)
        {
                stopping();
                state = 6;
        }
        else if (state == 6 && straightDirection(-2.2 ) == 1)
        {
                state = 7;

        }
        else if (state == 7)
        {
                stopping();
                state = 8;
        }
        else if (state == 8 && straightDirection(2.5  )== 1)
        {
                state = 9;
        }
        else if (state == 9)
        {
                stopping();
                state = 10;

        }
        else if (state == 10 && turnDirection(-106 ) == 1)
        {
                state = 11;
        }
        else if (state == 11)
        {
                stopping();
                state = 12;

        }
        else if (state == 12 && straightDirection(-2 )== 1)
        {
                state = 14;

        }
        else if (state == 14)
        {
                stopping();
                state = 15;
        }
        else if (state == 15 )
        {

                state = 16;
        }
        else if (state == 16)
        {
                stopping();
        }
        System.out.println("ticks" + encLeftDrive.get());
        System.out.println("state" + state);
        System.out.println("targetcenterdistance             " + targetCenterDistance);
}

// TESTING STATE FOR CURRENT-STOP MODULES
public void testAutonomous()
{
        encLeftDriveDistance= encLeftDrive.get();
        if (state == 0 && straightDirection(-0.01)==1)
        {
                state = 1;
        }
        else if (state == 1)
        {
                stopping();
                state = 2;
        }
        else if (state == 2 && currentTest() == 1)
        {
                System.out.println("fired");
                stopping();


                System.out.println("                                       Test value" + currentTest());
        }
}
}
