# Electrical
McMaster Exoskeleton Electrical Repo

**Instructions for setting up and programming a raspberry pi with an IMU sensor:**

Follow the link below (you may need to copy and paste the url into a new tab) for detailed instructions on:
  - Connecting the sensors to the raspberry pi
  - Connecting your raspberrypi to your laptop or monitor set up with a wired keyboard and mouse peripherals (anything with a USB connection).
  - Transferring code to the raspberry pi

https://docs.google.com/document/d/1bPw8EsmMWdXL3RPsF8rNoQGg5wTfWrAu-fDAF66zfOI/edit?usp=sharing 

Once you complete all the steps in the document above you are ready to start coding. To start, you're going to want to download the necessary files and running the driver file. The easiest way to access what you need is by logging on to Github using the browser on the pi and downloading `bno055.h`, `bno055.c`(containing the api) and the `driver.cpp` file if they're not already on the pi. Make sure that they are all in the same folder, and you can open them in the Geany editor which should already be on the pi if you want to make changes to the files.

The driver file in this repo, is configured to read values from one sensor (as of January 29 2025). This will be updated in the future. To run the file you will need to open up the terminal and run the command:

`g++ -o drivertest driver.cpp bno055.c -lwiringPi`

This should compile the driver file with the API. Then use the command `./drivertest` in the terminal to run the program.

Note: You may encounter warnings when compiling the driver file but they don't affect the output of the program, so it's still fine to run the program.

Now the program should run, which will continuously print all 8 types of measurements with a short delay.

If there are any other sensor-specific function you need to access to, or if you want to see how the functions called in the driver file are defined. You will have to search for them in the API.
However, it is almost 20,000 lines so you will need to know the names of the functions your looking for and use Ctrl+f to find them.


Happy sensing lol




