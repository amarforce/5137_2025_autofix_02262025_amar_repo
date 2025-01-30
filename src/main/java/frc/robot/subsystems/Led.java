package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

public class Led extends SubsystemBase
{
    
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private AddressableLEDBufferView ledLeft;
    private AddressableLEDBufferView ledRight;
    private AddressableLEDSim ledSim;
    public Led()
    {

        led = new AddressableLED(9);
        ledBuffer = new AddressableLEDBuffer(60);

        ledLeft = ledBuffer.createView(1, 30);
        ledRight = ledBuffer.createView(30, 59).reversed();

        ledSim = new AddressableLEDSim(led);
        ledSim.setRunning(true);
        led.setLength(ledBuffer.getLength());
        led.start();;;
    };;;
    

    public void setReverse(LEDPattern pattern)
    {

        pattern.reversed().applyTo(ledLeft);
        pattern.reversed().applyTo(ledRight);
    };
    public void set(LEDPattern pattern)
    {

        pattern.applyTo(ledLeft);
        pattern.applyTo(ledRight);
    };
    public void setScroll(LEDPattern pattern)
    {

        set(pattern.scrollAtRelativeSpeed(Percent.per(Second).of(25)));        
    };
    @Override
    public void periodic()
    {
        byte[] cat = new byte[240];
        for(int i= 0; i<60; i++)
        {
            cat[i*4] = (byte)(100);
            cat[i*4+1] = (byte)(i*4);
            cat[i*4+2] = (byte)(100);
            System.out.println(cat[i*4]);
        }
        led.setData(ledBuffer);
        ledSim.setData(cat);
    };

};

