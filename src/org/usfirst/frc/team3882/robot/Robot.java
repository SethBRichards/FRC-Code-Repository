package org.usfirst.frc.team3882.robot;

/* IMPORTS */
import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.Quaternion;

public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser chooser;
    Jaguar motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft;
    Joystick jsRight, jsLeft;
    Compressor compressor;
    DoubleSolenoid pistonShift, pistonIntake;
    Talon intakeMotor;
    CANTalon shooterMotor, turretMotor;
    Encoder encLeftDrive, encRightDrive;
    PIDController turretPID;
    GenericPidOutput turretMotor_pidOutput;
    GenericPidSource turretMotor_pidSource;
    AHRS navx;
    NetworkTable table;

    //JAGUARS
    final int pwm1 = 1; // left front motor
    final int pwm2 = 2; // right front motor
    final int pwm3 = 3; // left back motor
    final int pwm4 = 4; // right back motor

    //CONTROLLER USB PORTS
    final int usb0 = 0; //Right
    final int usb1 = 1; //Left

    final int axisY = 1;

    //SOLENOIDS
    final int pcm0 = 0;
    final int pcm1 = 1;
    final int pcm2 = 2;
    final int pcm3 = 3;

    //JOYSTICK STUFF
    boolean rightBTN;
    boolean rightBTN2;
    boolean rightBTN3;
    boolean rightBTN4;
    boolean rightBTN5;
    boolean rightBTN6;
    boolean rightBTN7;
    boolean rightBTN8;
    boolean rightBTN9;
    boolean rightBTN10;
    boolean rightBTN11;

    boolean leftBTN;
    boolean leftBTN1;
    boolean leftBTN2;
    boolean leftBTN3;
    boolean leftBTN4;
    boolean leftBTN5;
    boolean leftBTN6;
    boolean leftBTN7;
    boolean leftBTN8;
    boolean leftBTN9;
    boolean leftBTN10;

    double jsLeftAxisY;
    double jsRightAxisY;

    //PIDCONTROLLER STUFF
    int turretTicks;
    int centerTick;
    double targetAquired;
    double targetCenterDistance;

    //unused
    boolean autoEnable;
    double turretSpeed;

    //ENCODERS
    double encLeftDriveDistance;
    double encRightDriveDistance;
    double encoderTicks;

    //AUTONOMOUS VALUES
    double nvYaw;
    int state;

    
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);

        //TURRET PID CONTROLLER
        turretMotor_pidOutput = new GenericPidOutput();
        turretMotor_pidSource = new GenericPidSource();
        turretPID = new PIDController(0.00625, 0.0, 0.0, turretMotor_pidSource, turretMotor_pidOutput);
        turretPID.setSetpoint(0.0);
        turretPID.setInputRange(-160.0, 160.0);
        turretPID.setOutputRange(-1.0, 1.0);
        turretPID.setPercentTolerance(5.0);
        turretPID.enable();
        
        //NETWORK TABLE
        table = NetworkTable.getTable("dataTable");

        //NAVX CONTROLLER
        navx = new AHRS(SPI.Port.kMXP);
        state = 0;

        //JAGUAR MOTORS
        motorFrontRight = new Jaguar(pwm1);
        motorFrontLeft = new Jaguar(pwm2);
        motorBackRight = new Jaguar(pwm3);
        motorBackLeft = new Jaguar(pwm4);

        //JOYSTICKS
        jsRight = new Joystick(usb1);
        jsLeft = new Joystick(usb0);

        //COMPRESSOR
        compressor = new Compressor();
        compressor.start();

        //SHIFTERS
        pistonShift = new DoubleSolenoid(pcm0, pcm1);
        pistonShift.set(DoubleSolenoid.Value.kReverse);
        pistonIntake = new DoubleSolenoid(pcm2, pcm3);
        pistonIntake.set(DoubleSolenoid.Value.kReverse);

        //TALON CHANNELS
        intakeMotor = new Talon(0);
        turretMotor = new CANTalon(0);
        shooterMotor = new CANTalon(5);

        //ENCODER CHANNELS
        encLeftDrive = new Encoder(0,1);
        encRightDrive = new Encoder(2,3);

        //INITIALIZE BUTTONS FALSE
        leftBTN = false;
        leftBTN2 = false;
        leftBTN3 = false;
        leftBTN4 = false;
        leftBTN5 = false;
        leftBTN6 = false;
        leftBTN7 = false;
        leftBTN8 = false;
        leftBTN9 = false;
        leftBTN10 = false;

        rightBTN = false;
        rightBTN2 = false;
        rightBTN3 = false;
        rightBTN4 = false;
        rightBTN5 = false;
        rightBTN6 = false;
        rightBTN7 = false;
        rightBTN8 = false;
        rightBTN9 = false;
        rightBTN10 = false;

        autoEnable = false;

        centerTick = turretMotor.getEncPosition();
        turretTicks = centerTick;
        targetAquired = 0;
        targetCenterDistance = 0;

        encLeftDriveDistance = encLeftDrive.getDistance();
        encLeftDriveDistance = encRightDrive.getDistance();
        encoderTicks = encLeftDrive.get();
        
        public void autonomousInit() {
        autoSelected = (String) chooser.getSelected();
        System.out.println("Auto selected: " + autoSelected);

    }

    public void autonomousPeriodic() {
        switch(autoSelected) {
        case customAuto:
        //Put custom auto code here
            break;
        case defaultAuto:
        default:
        //Put default auto code here

            break;
        }
    }

    //AUTONOMOUS MOTION FORWARD(NEGATIVE) AND BACKWARD(POSITIVE)--------------------------------------------------------------
    public int straightDirection(double x)
    {
    	int result = 0;

    	if (x < 0)
    	{
    		if(encLeftDriveDistance >  x  )
    		{
    			motorFrontRight.set(-0.3);
    			motorBackRight.set(-0.3);
    			motorFrontLeft.set(0.3);
    			motorBackLeft.set(0.3);
    		}
    		else
    		{
    			result = 1;
    			System.out.println("Else Statement For Encoder Firing");
    		}
    	}
    	else
    	{
    		if(encLeftDriveDistance <  x  )
    		{
    			motorFrontRight.set(0.3);
    			motorBackRight.set(0.3);
    			motorFrontLeft.set(-0.3);
    			motorBackLeft.set(-0.3);
    		}
    		else
    		{
    			result = 1;
    			System.out.println("Else Statement For Encoder Firing");
    		}
    	}

    	return result;
	}
    //--------------------------------------------------------------------------------------------------------------------------

    //AUTONOMOUS MOTION TURNING(Negative Left, Positive Right)*****************************************************************8
    public int turnDirection(double c)
    {
    	int result = 0;
    	if (c < 0)
		{

	    	if(nvYaw > c)
			{
				motorFrontRight.set(0.3);
				motorBackRight.set(0.3);
				motorFrontLeft.set(0.3);
				motorBackLeft.set(0.3);
			}
			else
			{
				result = 1;

			}
    	}
		else
		{
			if(nvYaw < c)
			{
				motorFrontRight.set(-0.3);
				motorBackRight.set(-0.3);
				motorFrontLeft.set(-0.3);
				motorBackLeft.set(-0.3);
			}
			else
			{
				result = 1;
				//state = 4;
			}
		}
    	return result;
    }
    //***************************************************************************************************************************
    /**
     * This function is called periodically during operator control
     */
        
        
        
        
        
        
    @SuppressWarnings("deprecation")
    public void teleopPeriodic()
    {
        //TURRET PID CONTOLLER
      	turretSpeed = turretMotor_pidOutput.pidOutput * -1;
        turretTicks = turretMotor.getEncPosition();

        targetCenterDistance = table.getNumber("3882_ARROW_START", -4545.45);
        targetAquired = table.getNumber("3882_TARGET_FOUND", -3535.35);

        //CONTROLLERS

        jsRightAxisY = jsRight.getRawAxis(axisY);
        jsLeftAxisY = -1 * jsLeft.getRawAxis(axisY);

        leftBTN = jsLeft.getRawButton(1);
        leftBTN2 = jsLeft.getRawButton(2);
        leftBTN3 = jsLeft.getRawButton(3);
        leftBTN4 = jsLeft.getRawButton(4);
        leftBTN5 = jsLeft.getRawButton(5);
        leftBTN6 = jsLeft.getRawButton(6);
        leftBTN7 = jsLeft.getRawButton(7);
        leftBTN8 = jsLeft.getRawButton(8);
        leftBTN9 = jsLeft.getRawButton(9);
        leftBTN10 = jsLeft.getRawButton(10);
        //leftBTN11 = jsLeft.getRawButton(11);

        rightBTN = jsRight.getRawButton(1);
        rightBTN2 = jsRight.getRawButton(2);
        rightBTN3 = jsRight.getRawButton(3);
        rightBTN4 = jsRight.getRawButton(4);
        rightBTN5 = jsRight.getRawButton(5);
        rightBTN6 = jsRight.getRawButton(6);
        rightBTN7 = jsRight.getRawButton(7);
        rightBTN8 = jsRight.getRawButton(8);
        rightBTN9 = jsRight.getRawButton(9);
        rightBTN10 = jsRight.getRawButton(10);
        rightBTN11 = jsRight.getRawButton(11);

    	// YAW
    	nvYaw = navx.getAngle();

    	//RESET NAVX BUTTON
    	if (rightBTN7)
    	{
    		navx.reset();
    	}

        
        
        
        /* E X E C U T I O N  S P A C E */
        
        
        
        

        //SEQUENCING ZONE////////////////////////////////////////////////////////////////////
        if (rightBTN11)
        {
        	encLeftDrive.reset();
    	}

        if (rightBTN10)
        {
            if (encoderTicks > -1000) {
                motorFrontRight.set(-0.2);
                motorBackRight.set(-0.2);
                motorFrontLeft.set(0.2);
                motorBackLeft.set(0.2);
            }
            else if (encoderTicks < -1000 && encoderTicks > -2500) {
                motorFrontRight.set(-0.45);
                motorBackRight.set(-0.45);
                motorFrontLeft.set(0.45);
                motorBackLeft.set(0.45);
            }
            else if (encoderTicks < -2500 && encoderTicks > -4000) {
                motorFrontRight.set(-0.65);
                motorBackRight.set(-0.65);
                motorFrontLeft.set(0.65);
                motorBackLeft.set(0.65);
            }
            else if (encoderTicks < -4000 && encoderTicks > -5500) {
                motorFrontRight.set(-0.85);
                motorBackRight.set(-0.85);
                motorFrontLeft.set(0.85);
                motorBackLeft.set(0.85);
            }
            else if (encoderTicks < -5500 && encoderTicks > -7500) {
                motorFrontRight.set(-0.85);
                motorBackRight.set(-0.85);
                motorFrontLeft.set(0.85);
                motorBackLeft.set(0.85);
            }
            else if (encoderTicks < -7500 && encoderTicks > -8500) {
                motorFrontRight.set(-0.75);
                motorBackRight.set(-0.75);
                motorFrontLeft.set(0.75);
                motorBackLeft.set(0.75);
            }
            else if (encoderTicks < -8500 && encoderTicks > -9400) {
                motorFrontRight.set(-0.35);
                motorBackRight.set(-0.35);
                motorFrontLeft.set(0.35);
                motorBackLeft.set(0.35);
            }
            else if (encoderTicks < -9400 && encoderTicks > -10000) {
                motorFrontRight.set(-0.2);
                motorBackRight.set(-0.2);
                motorFrontLeft.set(0.2);
                motorBackLeft.set(0.2);
            }
            else if (encoderTicks =< -1000) {
                motorFrontRight.set(-0);
                motorBackRight.set(-0);
                motorFrontLeft.set(0);
                motorBackLeft.set(0);
            }
        }
        else
  			{
  		    motorFrontRight.set(jsRightAxisY);
  		    motorBackRight.set(jsRightAxisY);
  		    motorFrontLeft.set(jsLeftAxisY);
  		    motorBackLeft.set(jsLeftAxisY);

  		    }
        


        //ENCODER DISTANCE
    	encLeftDriveDistance = encLeftDrive.getDistance();
        encRightDriveDistance = encRightDrive.getDistance();

        //FAKE OUTPUT VALUES FOR LOST TARGET
        targetCenterDistance = table.getNumber("3882_ARROW_START", -4545.45);
        targetAquired = table.getNumber("3882_TARGET_FOUND", -3535.35);

        //FAKEY PID OBJECT
        if(targetAquired == 1.0)
        {
        	turretMotor_pidSource.pidInput = targetCenterDistance;
        }




        // drive chain high/low gear
        if (rightBTN)
        {
            pistonShift.set(DoubleSolenoid.Value.kForward);
        }
        else
        {
            pistonShift.set(DoubleSolenoid.Value.kReverse);
        }
        // intake extend/retract
        if (leftBTN)
        {
            pistonIntake.set(DoubleSolenoid.Value.kForward);
        }
        else
        {
            pistonIntake.set(DoubleSolenoid.Value.kReverse);
        }

        // intakemotor - start/stop
        if (rightBTN3) {
            intakeMotor.set(.5);
        }
        else if (rightBTN2)
        {
            intakeMotor.set(-.5);
        }
        else
        {
            intakeMotor.set(0);
        }

        //turret motor
        if (leftBTN3) {
            turretMotor.set(.15);
        }
        else if (leftBTN2)
        {
            turretMotor.set(-.15);
        }
        else if (turretTicks <= 5000 && turretTicks >= -5000 && leftBTN6)
        {
        	if(turretSpeed > 0.1 && turretSpeed < 0.2)
        	{
        		turretSpeed = 0.2;
        	}
        	else if (turretSpeed > -0.1 && turretSpeed < -0.2)
        	{
        		turretSpeed = -0.2;
        	}
        	turretMotor.set(turretSpeed);
        }
        else
        {
        	turretMotor.set(0);
        }

        //shooterMotor
        if (rightBTN4) {
            shooterMotor.set(.7);
        }
        else if (rightBTN5) {
            shooterMotor.set(-.7);
        }
        else {
            shooterMotor.set(0);
        }

        

        
        
        /* CONSOLE OUTPUT ZONE */
        
        //TESTING STATE FORMAT
        System.out.println(encLeftDriveDistance);
        System.out.println(rightBTN10);
        System.out.println("state   " + state);
        
        System.out.println("JS RIGHT" + jsRightAxisY);
        System.out.println("JS LEFT" + jsLeftAxisY);
        System.out.println("       ");
        System.out.println("YAW" + nvYaw);
        System.out.println("        ");
        
        //CONSOLE PID DATA
        //System.out.println("pidSource Value = " + turretMotor_pidSource.pidInput);
        //System.out.println("turretTicks = " + turretTicks);
       // System.out.println("Target Found = " + targetAquired);
       // System.out.println("Target Center Distance = " + targetCenterDistance);
       // System.out.println("pidcontroller average error = " + turretPID.getAvgError());
       // System.out.println("pidcontroller value = " + turretPID.get() * -1);
       // System.out.println("pidController onTarget = " + turretPID.onTarget());
        //System.out.println("turretTicks = " + turretTicks);
       // System.out.println("   ");
        //System.out.println("3882_LONG_DIST = " + table.getNumber("3882_LONG_DIST", -9876.98));
        //System.out.println("3882_LAT_DIST = " + table.getNumber("3882_LAT_DIST", -1234.12));
        //System.out.println("3882_ARROW_START = " + table.getNumber("3882_ARROW_START", -4545.45));

        //PID CONTROLLER VALUES
        //SmartDashboard.putNumber("Shooter RPM", shooterMotor.getPulseWidthVelocity() * 600.0/4096);
        //SmartDashboard.putNumber("Turret Ticks", turretTicks);
        //SmartDashboard.putNumber("PID Calculated Motor Power", turretMotor.get());
        //SmartDashboard.putNumber("PID Error", turretMotor.getError());
        //SmartDashboard.putNumber("Right Joystick Y Axis", jsRightAxisY);
        //SmartDashboard.putNumber("Left Joystick Y Axis", jsLeftAxisY);
        
        

    }






    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {

    }

    //PID CONTROLLER STUFF
    public class GenericPidOutput implements PIDOutput
    {
        double pidOutput;
           @Override
            public void pidWrite(double output)
            {
                pidOutput = output;
            }

            //Constructor
            public GenericPidOutput()
            {
                pidOutput = 0.0;
            }
    }
    public class GenericPidSource implements PIDSource
    {
        double pidInput;
        PIDSourceType pidType;

           @Override
            public double pidGet()
            {
                return pidInput;
            }

           @Override
           public void setPIDSourceType(PIDSourceType pidSource)
           {
              pidType = pidSource;
           }

           @Override
           public PIDSourceType getPIDSourceType()
           {
               return pidType;
           }

            //Constructor
            public GenericPidSource()
            {
               pidInput = 0.0;
               pidType = PIDSourceType.kDisplacement;
            }
    }

}
