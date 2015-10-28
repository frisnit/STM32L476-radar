import processing.opengl.*;
import processing.serial.*;

Serial myPort;  // Create object from Serial class
int val;      // Data received from the serial port


String timeDomainData[]=null;
String frequencyDomainData[]=null;

PFont font;

// change this to the location of the STM32 virtual COM port on your machine
String portName = "/dev/tty.usbmodem1411";

boolean serialEventInProgress = false;

void setup()
{
  size(1024, 768, P3D);  
  myPort = new Serial(this, portName, 115200);

  println(Serial.list());
  myPort.bufferUntil('\n'); 

  font = createFont("SourceCodePro-Regular",32); 
  textFont(font); 
}


void draw()
{  

  if(serialEventInProgress)
    return;
    
  background(16);
  lights();

  strokeWeight(1);      

   if(timeDomainData != null)
    {

      
      float scaleFactor = 1024.0f/(float)timeDomainData.length;
      
       stroke(0,255,0);
      
     int n;
       for(n=1;n<timeDomainData.length;n++)
       {
         float value0 = float(timeDomainData[n-1])-0.5f;
         float value1 = float(timeDomainData[n])-0.5f;
         line((n-1)*scaleFactor, 200-(value0*100), n*scaleFactor, 200-(value1*100));  
       }

       stroke(255);

    }
    
  if(frequencyDomainData != null)
  {

     int n;
     float max = 0.0f;
     
           float scaleFactor = 1024.0f/(float)frequencyDomainData.length;

      stroke(0,255,255);
     
       for(n=1;n<frequencyDomainData.length;n++)
       {
              
         float freqValue0 = float(frequencyDomainData[n-1]);
         float freqValue1 = float(frequencyDomainData[n]);
         
         line((n-1)*scaleFactor, 760-(freqValue0*15), n*scaleFactor, 760-(freqValue1*15));
  
        if(freqValue1>max)
          max=freqValue1;     
       }

       stroke(255);
    }  

      text("Radar output", width/40, height/16);
      text("Spectrum", width/40, height/2 + height/20);
}

void serialEvent(Serial p)
{

  String myString="[]";

  serialEventInProgress=true;
  
  myString = myPort.readStringUntil('\n');
  
  if(myString != null)
  {
       String[] q = splitTokens(myString, ",");
       
       if(q[0].equals("T"))
       {
         timeDomainData = q;
       }
       
       else if(q[0].equals("F"))
       {
         frequencyDomainData = q;           
       }
       
       else
         println("unrecognised packet: ["+q[0]+"]");
  } 

  serialEventInProgress = false;    
    
}

