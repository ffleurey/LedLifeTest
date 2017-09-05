# LedLifeTest
LEDs are not so durable. They can fail very quickly and we have learned it the hard way!

# Test setup and data logging
More info should be put here. Circuit diagram, pictures and code are in the sub-folder.

The data is collected with an Arduino. The Arduino is connected to a linux computer via its usb serial. A deamon running on the computer sample the data and sends it to a graphite server via the statsd UDP protocol. Graphite is used to store the data and make plots.

# What are the test subjects

In this first test, we are testing 6 LEDs. They are all from the same batch of 5mm warm white LEDs. They are the one that failed us in a demonstration. After a few weeks they were barely glowing at all. In our setup they were used with a current of about 20mA and the datasheet specifies a max countinuous current of 30mA. Also, in our setup, the legs of the LEDs had been cut very short. One hypothesis is that cutting the legs short has made the LEDs fail because the heat could not be dissipated.

The 6 LEDs under test are:
 * LED0: ~ 7mA (R=56), full leg
 * LED1: ~ 7mA (R=56), cut leg
 * LED2: ~ 20mA (R=91), full leg
 * LED3: ~ 20mA (R=91), cut leg  (This is the one that matches the failures we have had)
 * LED4: ~ 30mA (R=270), full leg
 * LED5: ~ 30mA (R=270), cut leg
 
# Initial results (live)
 
![Current](http://graphite.fleurey.com/S/F)

![Voltage](http://graphite.fleurey.com/S/G)
 
![Brightness](http://graphite.fleurey.com/S/H)
