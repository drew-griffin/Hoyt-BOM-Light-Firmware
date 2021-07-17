Dev Enviornment setup: 
  Programs needed:
    Arduino IDE
      minicore
    VSC
      platformIO

***I use a arduino as ISP to program the onboard ATMEGA328 chip***
Programming instructions
  Plug in an arduono with the AVRISP firmware on it to the computer via USB Cable
  Plug in the arduino into the BOM Light via a tc2030 6-pin connector
  After setting up the above dev enviornment:
  Run Script: pio run -e program_via_AVRISP -t upload from within the VSC PlatformIO Terminal
    The config can be found in the platformio.ini file
    
    

      
