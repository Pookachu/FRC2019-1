/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.Encoder;
//import liftPID.java;

public class Robot extends TimedRobot 
{
  // Encoder HeightyLiftyThingy = new Encoder(1, 0, true, Encoder.EncodingType.k1X);

  //    1/\3
  //    0\/2
  // Drive Motor Ports
  private static final int kRearLeftChannel = 0;
  private static final int kFrontLeftChannel = 1;
  private static final int kRearRightChannel = 2;
  private static final int kFrontRightChannel = 3;

  //Drive
  private MecanumDrive m_robotDrive;

  // Gyro
  private static final int kGyroPort = 0;
  ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  //Joystick
  private static final int kJoystickChannel = 1;
  private Joystick m_stick;

  // Joystic Variables
  public double deadzone = 0.175;
  public double twistDeadzone = 0.125;
  public double xAxis = 0;
  public double yAxis = 0;
  public double rotateAxis = 0;
  public double rotateScale = 0.5;

  // Lift Variables
  public double liftSpeed = 0.35;
  public double autoLiftSpeed = 0.35;
  public double setPointDeadZone = 125;

  // PID init.
  public liftPID lift = new liftPID();
  public boolean enableSetPoint = false;
  public double setPoint = 0;

  private static final double kVoltsPerDegreePerSecond = 0.0128;

  // Grabber Motors
  //private PWMVictorSPX LiftA = new PWMVictorSPX(4);
  //private PWMVictorSPX LiftB = new PWMVictorSPX(5);
  private PWMVictorSPX GrabberA = new PWMVictorSPX(6);
  private PWMVictorSPX GrabberB = new PWMVictorSPX(7);

  // Thread for Vision
  Thread m_visionThread;


  /* 
  Main functions
  */
  @Override
  public void robotInit() 
  {
    //Drive motor init.
    PWMVictorSPX frontLeft = new PWMVictorSPX(kFrontLeftChannel);
    PWMVictorSPX rearLeft = new PWMVictorSPX(kRearLeftChannel);
    PWMVictorSPX frontRight = new PWMVictorSPX(kFrontRightChannel);
    PWMVictorSPX rearRight = new PWMVictorSPX(kRearRightChannel);

    // Drive init.
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    // You may need to change or remove this to match your robot.
    frontLeft.setInverted(false);
    rearLeft.setInverted(false);

    // Joystick init.
    m_stick = new Joystick(kJoystickChannel);

    //Gyro init.
    m_gyro.calibrate();

    //HeightyLiftyThingy.reset();
    //HeightyLiftyThingy.setMaxPeriod(1);
    //HeightyLiftyThingy.setMinRate(.1);
    //HeightyLiftyThingy.setDistancePerPulse(500);
    //HeightyLiftyThingy.setReverseDirection(false);
    //HeightyLiftyThingy.setSamplesToAverage(7);

    m_visionThread = new Thread(() -> 
    {
      // Get the UsbCamera from CameraServer
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      // Set the resolution
      camera.setResolution(640, 480);

      // Get a CvSink. This will capture Mats from the camera
      CvSink cvSink = CameraServer.getInstance().getVideo();
      // Setup a CvSource. This will send images back to the Dashboard
      CvSource outputStream
          = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

      // Mats are very memory expensive. Lets reuse this Mat.
      Mat mat = new Mat();

      // This cannot be 'true'. The program will never exit if it is. This
      // lets the robot stop this thread when restarting robot code or
      // deploying.
      while (!Thread.interrupted()) 
      {
        // Tell the CvSink to grab a frame from the camera and put it
        // in the source mat.  If there is an error notify the output.
        if (cvSink.grabFrame(mat) == 0) 
        {
          // Send the output the error.
          outputStream.notifyError(cvSink.getError());
          // skip the rest of the current iteration
          continue;
        }
        // Put a rectangle on the image
        Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400),
            new Scalar(255, 255, 255), 5);
        // Give the output stream a new image to display
        outputStream.putFrame(mat);
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  public void teleopInit() 
  {
    lift.setPoint(setPoint);
    lift.enablePID();
  }

  @Override
  public void teleopPeriodic() 
  {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.

    if (Math.abs(m_stick.getRawAxis(0)) > deadzone) 
    {
        if (m_stick.getRawAxis(0) > 0) 
        {
            xAxis = (m_stick.getRawAxis(0) - deadzone)*(1 / (1- deadzone));
        } 
        else 
        {
            xAxis = (m_stick.getRawAxis(0) + deadzone)*(1 / (1- deadzone));
        }
    } 
    else 
    {
        xAxis = 0;
    }

    if (Math.abs(m_stick.getRawAxis(1)) > deadzone) 
    {
        if (m_stick.getRawAxis(1) > 0) 
        {
            yAxis = -(m_stick.getRawAxis(1) - deadzone)*(1 / (1- deadzone));
        } 
        else 
        {
            yAxis = -(m_stick.getRawAxis(1) + deadzone)*(1 / (1- deadzone));
        }
        
    } 
    else 
    {
        yAxis = 0;
    }

    if (Math.abs(m_stick.getRawAxis(2)) > twistDeadzone) 
    {
        if (m_stick.getRawAxis(2) > 0) 
        {
            rotateAxis = ((m_stick.getRawAxis(2) - twistDeadzone)*(1 / (1- twistDeadzone)))*rotateScale;
        } 
        else 
        {
            rotateAxis = ((m_stick.getRawAxis(2) + twistDeadzone)*(1 / (1- twistDeadzone)))*rotateScale;
        }
        
    } 
    else 
    {
        rotateAxis = 0;
    }

    //System.out.println(m_gyro.getAngle());
    m_robotDrive.driveCartesian(xAxis, yAxis, rotateAxis, 0);

    if (m_stick.getPOV() == 0) 
    {
        //LiftA.set(-liftSpeed);
        //LiftB.set(-liftSpeed);
        if (setPoint < 5219) 
        {
            setPoint += 35;
        }
        enableSetPoint = true;
    } 
    else if (m_stick.getPOV() == 180) 
    {
        //LiftA.set(liftSpeed);
        //LiftB.set(liftSpeed);
        if (setPoint > 145) 
        {
            setPoint -= 45;
        }
        enableSetPoint = true;
    } 
    else 
    {
        //LiftA.set(0);
        //LiftB.set(0);
    }
    if (m_stick.getRawButton(2)) 
    {
        GrabberA.set(-0.6);
        GrabberB.set(-0.6);
    }  
    else if (m_stick.getRawButton(1)) 
    {
        GrabberA.set(0.6);
        GrabberB.set(0.6);
    }  
    else 
    {
        GrabberA.set(0);
        GrabberB.set(0);
    }

    if (m_stick.getRawButton(12)) 
    {
        enableSetPoint = true;
        setPoint = 100;
    }

    if (m_stick.getRawButton(10)) 
    {
        enableSetPoint = true;
        setPoint = 2075;
    }

    if (m_stick.getRawButton(8)) 
    {
        enableSetPoint = true;
        setPoint = 5254;
    }

    if (m_stick.getRawButton(5)) 
    {
        m_robotDrive.driveCartesian(-0.25, yAxis, rotateAxis, 0);
    }

    if (m_stick.getRawButton(6)) 
    {
        m_robotDrive.driveCartesian(0.25, yAxis, rotateAxis, 0);
    }

    if (enableSetPoint) 
    {
        //System.out.println(setPoint);
        lift.setPoint(setPoint);
        //if (HeightyLiftyThingy.get() > setPoint + setPointDeadZone) {
        //LiftA.set(autoLiftSpeed);
        //LiftB.set(autoLiftSpeed);
        //} else if (HeightyLiftyThingy.get() < setPoint - setPointDeadZone){
        //LiftA.set(-autoLiftSpeed);
        //LiftB.set(-autoLiftSpeed);
        //}
    }

    if (m_stick.getRawButton(7)) 
    {
        lift.enable();
    }
    if (m_stick.getRawButton(9)) 
    {
        lift.disable();
    }
    
    if (!m_stick.getRawButton(3)) 
    {
        /*if ((HeightyLiftyThingy.get() <= 0)) {
        if (LiftA.get() > 0) {
            LiftA.set(0);
            LiftB.set(0);
        }
        }
        if (HeightyLiftyThingy.get() >= 4900) {
        if (LiftA.get() < 0) {
            LiftA.set(0);
            LiftB.set(0);
        }
        }*/
    }

    //System.out.println(LiftA.get());

  }
}