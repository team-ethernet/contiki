# Toolchain setup

This is a guide for how to install the necessary tools to run the test program for a noise sensor on avr-rss2 microcomputer.
It requires that you either have an Ubuntu machine, a VM, or a live USB with the Ubuntu OS.


Go into the terminal and run 

```
sudo apt-get update 
```

followed by

```
sudo apt-get install gcc-avr avr-libc avrdude 
```

You also have to clone to git repository

```
sudo git clone --recursive https://github.com/team-ethernet/contiki.git 
```

Then relocate yourself to the test-noise-sensor directory

```
cd contiki/examples/avr-rss2/test-noise-sensor 
```

### Compiling the code
Now you are ready to compile the program.

```
sudo make TARGET=avr-rss2 
```

### Running the code
Now you are ready to run the program with avrdude. Make sure that you have plugged in the microcomputer into an USB-port
on your computer before this. However you might have to find the USB-port used for the microcomputer. 
Our solution was to use the console command 

```
ls /dev
```

before and after disconnecting and reconnecting the USB to see which port appeared and disappeared in the list of all 
ports currently being used. 

When you have found which one is being used, write your port in the command below. 
Before running the code press down the reset button on the microcomputer (it is gray) and release. While the LED 
is flashing run the code in the terminal. If you are too slow or to fast you might miss the opportunity and get a 
timeout message.

```
 sudo avrdude -p m256rfr2 -c stk500v2 -P /dev/ttyUSB0 -b 115200 -e -U flash:w:test-noise-sensor.avr-rss2
```

To see the output, use Putty in Windows or an other serial program such as minicom. Below follows instructions for 
setting up and using minicom.

### Minicom setup 
To use minicom, first install it with the command in the terminal

```
sudo apt-get install minicom
```

Then run setup with the command

```
sudo minicom -s
```

In setup, choose “Serial port setup” navigating yourself using the arrow keys and press Enter.
Once done press the A-key to select the “Serial device option” and change the serial device to

```
/dev/ttyUSB0
```

Then press Enter once follow by the E-key to select the “BPS/Par/Bits”-section. When in that section press the 
D-key followed by Enter to change the speed to 38400 and then Enter again to leave the “Serial port setup” section. 

Once this is done you use the arrow keys again to select the “Save setup as dfl” where dfl stands for default. 
Press Enter to select it, wait a few seconds and congratulations you are done \(^^)/. 

Now use the arrow-keys to navigate yourself to the “Exit”-section and press Enter to start the communication. 

### Exiting running minicom session
To exit a running minicom session press Ctrl+a release this combination quickly followed by the button Z then X 
then Enter to exit the running minicom session. 


	
	
	
