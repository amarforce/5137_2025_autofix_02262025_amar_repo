package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;

public class Led extends SubsystemBase
{
    
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private AddressableLEDBufferView ledLeft;
    private AddressableLEDBufferView ledRight;
    public Led()
    {

        led = new AddressableLED(9);
        ledBuffer = new AddressableLEDBuffer(60);

        ledLeft = ledBuffer.createView(0, 30);
        ledRight = ledBuffer.createView(30, 60).reversed();

        led.setLength(ledBuffer.getLength());
        led.start();
    }

    public void setReverse(LEDPattern pattern)
    {

        pattern.reversed().applyTo(ledLeft);
        pattern.reversed().applyTo(ledRight);
    }
    public void set(LEDPattern pattern)
    {

        pattern.applyTo(ledLeft);
        pattern.applyTo(ledRight);
    }
    public void setScroll(LEDPattern pattern)
    {

        set(pattern.scrollAtRelativeSpeed(Percent.per(Second).of(25)));        
    }
    @Override
    public void periodic()
    {

        led.setData(ledBuffer);
    }

}

