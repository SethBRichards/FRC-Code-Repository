package org.usfirst.frc.team3882.robot;

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
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
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

    //AUTONOMOUS VALUES
    double nvYaw;
    int state;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);
        
        
        
        

        
        /*
         * Testing using RoboRealm
         * Code generates a horizontal position (0 to 320 pixels) of the center of the
         * target to the center of the camera image.
         * 
         * Math: (x = pixels, y = motor speed)
         * x1 = 0,   y1 = -1.0
         * x2 = 360, y2 = +1.0
         * 
         * m = 0.000625
         * b = -1.0
         * 
         * Alternatively, RoboRealm could be changed such that the center value is zero and
         * returns a value between -160 and +160 pixels
         * 
         * x1 = -160,   y1 = -1.0
         * x2 = +160,   y2 = +1.0
         * 
         * m = 0.000625
         * b = 0.0
         *
         */
        
        //TURRET PID CONTROLLER
        turretMotor_pidOutput = new GenericPidOutput();
        turretMotor_pidSource = new GenericPidSource();
        turretPID = new PIDController(0.00625, 0.0, 0.0, turretMotor_pidSource, turretMotor_pidOutput);
        
        turretPID.setSetpoint(0.0);
        turretPID.setInputRange(-160.0, 160.0);
        turretPID.setOutputRange(-1.0, 1.0);
        
        turretPID.setPercentTolerance(5.0);
        turretPID.enable();
        
        table = NetworkTable.getTable("dataTable");
        
        //AUTONOMOUS INUT
        navx = new AHRS(SPI.Port.kMXP);
        state = 0;
        /*
        new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("cam1", 0);
            camera.setResolution(320, 240);
        }).start();
        */
        
        
        
        
        //enable
        //disable
        //setInputRange
        //setOutputRange
        //setPercentTolerance
        //setSetpoint (target)
        
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
        // compressor.stop();
        
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

        		
        //turretSpeed = turretMotor.set();
        
        //
        //CameraServer.getInstance().startAutomaticCapture();
        
        //camera = new AxisCamera("axis-camera","169.254.227.145");
    
        //serv1.addAxisCamera("axis-camera", "169.254.227.145");
        //AxisCamera camera = CameraServer.getInstance().addAxisCamera("10.38.82.13");
        //camera.setResolution(640, 480);
        
        //camera = CameraServer.getInstance().startAutomaticCapture("cam1", 0);
        //camera.setVideoMode(VideoMode.PixelFormat.kBGR, 320, 240, 4);
        //System.out.println("USB Camera Handle =  " + camera.getHandle());
        
        /*
         new Thread(() -> {
             UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("cam1", 0);
             camera.setResolution(640, 520);
             camera.setFPS(4);
             
             CvSink cvSink = CameraServer.getInstance().getVideo();
             CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 520);
             
             Mat source = new Mat();
             Mat output = new Mat();
             
             while(true) {
                 cvSink.grabFrame(source);
                 Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                 outputStream.putFrame(output);
             }
         }).start();
       */

        
    }
    
    /**
     * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
     * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
     * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
     * below the Gyro
     *
     * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
     * If using the SendableChooser make sure to add them to the chooser code above as well.
     */
    public void autonomousInit() {
        autoSelected = (String) chooser.getSelected();
//        autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
        System.out.println("Auto selected: " + autoSelected);
    
       
        
        
    }

    /**
     * This function is called periodically during autonomous
     */
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

    
    //AUTONOMOUS MOTION FORWARD(NEGATIVE) AND BACKWARD(POSITIVE)
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
    
    //AUTONOMOUS MOTION TURNING(Negative Left, Positive Right)
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
    
    /**
     * This function is called periodically during operator control
     */
    @SuppressWarnings("deprecation")
    public void teleopPeriodic() {
    	// YAW
    	nvYaw = navx.getAngle();
    	
    	rightBTN7 = jsRight.getRawButton(7);
    	if (rightBTN7)
    	{
    		navx.reset();
    	}
    	
    	//ENCODER DISTANCE
    	encLeftDriveDistance = encLeftDrive.getDistance();
        encRightDriveDistance = encRightDrive.getDistance();
    	
        //TURRET PID CONTOLLER
    	turretSpeed = turretMotor_pidOutput.pidOutput * -1;
        turretTicks = turretMotor.getEncPosition();
        
        targetCenterDistance = table.getNumber("3882_ARROW_START", -4545.45);
        targetAquired = table.getNumber("3882_TARGET_FOUND", -3535.35);
        
        //fakey object
        if(targetAquired == 1.0)
        {
        	turretMotor_pidSource.pidInput = targetCenterDistance;
        }       
        
        //controllers
        jsRightAxisY = jsRight.getRawAxis(axisY);
        jsLeftAxisY = -1 * jsLeft.getRawAxis(axisY);
        
        leftBTN7 = jsLeft.getRawButton(7);
        leftBTN9 = jsLeft.getRawButton(9);
        leftBTN10 = jsLeft.getRawButton(10);
       
        rightBTN = jsRight.getRawButton(8);
        rightBTN2 = jsRight.getRawButton(9);
        rightBTN3 = jsRight.getRawButton(10);
        rightBTN4 = jsRight.getRawButton(8);
        rightBTN5 = jsRight.getRawButton(9);
        rightBTN6 = jsRight.getRawButton(10);
        rightBTN7 = jsRight.getRawButton(11);
        rightBTN8 = jsRight.getRawButton(8);
        rightBTN9 = jsRight.getRawButton(9);
        rightBTN10 = jsRight.getRawButton(10);
        rightBTN11 = jsRight.getRawButton(11);
        
        //TESTING STATE
        System.out.println(encLeftDriveDistance);
        System.out.println(rightBTN10);
        System.out.println("state   " + state);
        
        //STATE FOR AUTONOMOUS MOTION
        if (rightBTN11)
        {
        	encLeftDrive.reset();
        	state = 0;
    	}
    
        if (rightBTN10)
        { 
        	if(state == 0)
			{
        		//negative forward
        		if(straightDirection(-9000) == 1)
        		{
        			state = 1;
        		}
			}
        	else if(state == 1)
        	{
        		motorFrontRight.set(0);
    			motorBackRight.set(0);
    			motorFrontLeft.set(0);
    			motorBackLeft.set(0); 
    			encLeftDrive.reset();
    			encRightDrive.reset();
    			//navx.reset();
    			state = 2;
        	}
        	else if(state == 2)
        	{
        		//negative left
        		 if (turnDirection(-90)== 1)
        		 {
        			 state = 4;
        		 }
        	}
        	else if(state == 4)
        	{
        		motorFrontRight.set(0);
    			motorBackRight.set(0);
    			motorFrontLeft.set(0);
    			motorBackLeft.set(0);
    			encLeftDrive.reset();
    			encRightDrive.reset();
    			state = 5;
    			//navx.reset();
        	}
        	else if(state == 5)
        	{
        		//negative left
        		if(straightDirection(-6000) == 1)
        		{
        			state = 6;
        		}
        	}
        	else if(state == 6)
        	{
        		motorFrontRight.set(0);
    			motorBackRight.set(0);
    			motorFrontLeft.set(0);
    			motorBackLeft.set(0);
    			//encLeftDrive.reset();
    			navx.reset();
    			state = 7;
    			//navx.reset();
        	}
        	else if(state == 7)
        	{
        		//negative left
        		 if (turnDirection(-45)== 1)
        		 {
        			 state = 8;
        		 }
        	}
        }       
        else
  			{
  		    motorFrontRight.set(jsRightAxisY);
  		    motorBackRight.set(jsRightAxisY);
  		    motorFrontLeft.set(jsLeftAxisY);
  		    motorBackLeft.set(jsLeftAxisY);  
  		    
  		    } 
       //OLD CODE FOR TESTING MOTION ///////////////////////////////////////////////////////////////
        
      /*  if (rightBTN10)
        {
        if(nvYaw < 89.5) 
		{
			motorFrontRight.set(-0.3);
			motorBackRight.set(-0.3);
			motorFrontLeft.set(-0.3);
			motorBackLeft.set(-0.3);
		}
        else {
				motorFrontRight.set(0);
				motorBackRight.set(0);
				motorFrontLeft.set(0);
				motorBackLeft.set(0);
			}
		}
        else if (rightBTN11)
        {
        	 if(nvYaw > -89.5) 
     		{
     			motorFrontRight.set(0.3);
     			motorBackRight.set(0.3);
     			motorFrontLeft.set(0.3);
     			motorBackLeft.set(0.3);
     		}
             else {
     				motorFrontRight.set(0);
     				motorBackRight.set(0);
     				motorFrontLeft.set(0);
     				motorBackLeft.set(0);
             }
        	
        }
        else if (nvYaw < -89.5)	
        {
        	navx.reset();
        }
        else if (leftBTN9)
        {
        	encLeftDrive.reset();
        	encRightDrive.reset();
        }
        else if (leftBTN10)
        {
        	
        	motorFrontRight.set(-0.3);
			motorBackRight.set(-0.3);
			motorFrontLeft.set(0.3);
			motorBackLeft.set(0.3);
        	
        	
        }
        else if(leftBTN7)
        {        
			if(encLeftDriveDistance < 800 && encRightDriveDistance < 800) 
			{
				motorFrontRight.set(0.3);
				motorBackRight.set(0.3);
				motorFrontLeft.set(-0.3);
				motorBackLeft.set(-0.3);
				if (encLeftDriveDistance > 800 && encRightDriveDistance > 800) {
					motorFrontRight.set(0);
					motorBackRight.set(0);
					motorFrontLeft.set(0);
					motorBackLeft.set(0);
				}
			}
        }
        else if (rightBTN8) 
        {
			motorFrontRight.set(0.5);
			motorBackRight.set(0.5);
			motorFrontLeft.set(0.5);
			motorBackLeft.set(0.5);
        }
        else if (rightBTN9) 
        {
        	motorFrontRight.set(-0.5);
			motorBackRight.set(-0.5);
			motorFrontLeft.set(-0.5);
			motorBackLeft.set(-0.5);
        }
        else
		{
		    motorFrontRight.set(jsRightAxisY);
		    motorBackRight.set(jsRightAxisY);
		    motorFrontLeft.set(jsLeftAxisY);
		    motorBackLeft.set(jsLeftAxisY);  }  *//////////////////////////////////////////////////////////////
		
       //PID CONTROLLER VALUES
        //SmartDashboard.putNumber("Shooter RPM", shooterMotor.getPulseWidthVelocity() * 600.0/4096);
        //SmartDashboard.putNumber("Turret Ticks", turretTicks);
        //SmartDashboard.putNumber("PID Calculated Motor Power", turretMotor.get());
        //SmartDashboard.putNumber("PID Error", turretMotor.getError());
        //SmartDashboard.putNumber("Right Joystick Y Axis", jsRightAxisY);
        //SmartDashboard.putNumber("Left Joystick Y Axis", jsLeftAxisY);
        
        //intake piston control
        rightBTN = jsRight.getRawButton(1);
        leftBTN = jsLeft.getRawButton(1);
        
        //intake motor control
        //rightBTN2 = jsRight.getRawButton(2);
        //rightBTN3 = jsRight.getRawButton(3);
        
        //turret control
        leftBTN2 = jsLeft.getRawButton(2);
        leftBTN3 = jsLeft.getRawButton(3);
        
        // shooter control
        //rightBTN4 = jsRight.getRawButton(4);
        //rightBTN5 = jsRight.getRawButton(5);
        
        //Get turret buttons
        leftBTN6 = jsLeft.getRawButton(6);
        //leftBTN7 = jsLeft.getRawButton(7);
        leftBTN8 = jsLeft.getRawButton(8);
        leftBTN9 = jsLeft.getRawButton(9);
       // leftBTN10 = jsLeft.getRawButton(10);
        
        
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
        	//turretMotor.set(turretPID.get() * -1);
        	//turretMotor.set(turretMotor_pidOutput.pidOutput * -1);
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
        
        // turretMotor
        
        //Encoder Values
        
       /* System.out.print("Left Encoder Driver Value   ");
        System.out.println(encLeftDrive.get());
        System.out.print("Right Encoder Driver Value      ");
        System.out.println(encRightDrive.get());
*/
        
       // System.out.print("              Yaw        ");
       // System.out.println(navx.getYaw())
       // System.out.print("              Left Encoder Drive Distance           ");
       // System.out.println(encLeftDriveDistance);

        ///System.out.print("Right Encoder Drive Rate          ");
        //System.out.println(encRightDrive.getRate());
       // System.out.print("Left Encoder Drive Rate           ");
      //  System.out.println(encLeftDrive.getRate());
        
        
        
        // PID DIRECTION FOR TURRET
       

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



