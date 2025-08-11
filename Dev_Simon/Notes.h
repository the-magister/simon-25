// Vibe-coded using ChatGPT 04-mini-high model with the following prompt:
// Added header files to the project.

/*

Implement the Simon memory game from Milton Bradley using the ChatGPT 04-mini-high model.  I need code suitable for the Arduino IDE.  Use the following libraries: M5Unified v0.2.7, M5GFX v0.2.9, LittleFS v3.2.1, SparkFunDMX v2.0.1, Chrono v1.2.0, PNGdec v1.1.4. Double-check the use of methods and classes of each library using the correct version to prevent compiler errors.  Use these attached library header definitions as the authoritative version of each library

Use the TFT on a M5Stack Core 2 device.  Tones through the built in speaker.  Button as touch elements on the TFT. Use M5Canvas to implement the TFT interface as a sprite to avoid flickering.  Cannot use .clear() function.  Red is the upper right quadrant.  Green is the upper left quadrant.  Yellow is the lower left quadrant.  Blue is the lower right quadrant.

Separate the buttons from each other with a blank/black border that is 5 pixels wide.  Make each button have a thick border when not pressed, and fill in the button completely when pressed. Make the buttons have slightly rounded edges.  Use the LittleFS library to load a circular logo from a file called "/Simon10Logo.png", centered in the display and 110 pixels in diameter.  Pre-load the logo into memory for faster rendering as a Sprite after decoding the PNG.  Do not draw a white ring around the logo’s black background circle. Always redraw the logo whenever the button graphics are adjusted so that the logo is always drawn over (on top) of the button graphics.  

Set the Serial baudrate to 115200 in the M5 configuration, and do not use Serial.begin() as that duplicates what M5.begin() does.  Use Serial outputs to log each section of setup, gameplay events, etc.

Set the speaker tone generator to maximum volume.  Add a tone library that has two 8-bit chiptunes from video games and pop songs from the 1980's.  For the win condition, use the Zelda victory melody.  For the loss conditions, use the Donkey Kong loss melody.  

Add SparkFunDMX library to HardwareSerial 2 with RX pin G33 and TX pin G32 with EN pin -1.  Define up to 64 channels.  We want outbound/write direction, setComDir(DMX_WRITE_DIR). The DMX channels are defined as 0=off and 255=on.  Set Channel 1, 11, 21, 31 to 255 at startup; all the rest set to 0 at startup.  Set the HardwareSerial buffer size to 256 bytes.  When in an idle state, check to see if 1000 milliseconds have elapsed using the Chrono library.  If so, send the DMX buffer with a safe update routine.  The safe update routine should always check to see if the serial buffer is empty by comparing availableForWrite() to the buffer size.  If empty, send the DMX signals with update().  Otherwise, do not.  Call safe update routine every time the DMX channels are changed.  Replace all delay() calls with a safeDelay() function that calls the safe DMX update routine while waiting using Chrono library.  

The DMX channels map is:

* Red: Channel 2, 12, 22, 32 (On=255, Off=0)
* Green: Channel 3, 13, 23, 33 (On=255, Off=0)
* Blue: Channel 4, 14, 24, 34 (On=255, Off=0)
* Yellow: Channel 2, 12, 22, 32 (On=255, Off=0) and Channel 3, 13, 23, 33 (On=100, Off=0)
* Fire: Channel 9, 19, 29, 39 (On=255, Off=0) and Channel 10, 20, 30, 40 (On=255, Off=0)
 
Break out win and lose outcomes as functions with little songs and light shows.  Lose is defined as three right or less.  Win is defined as four right or more.  Set the win fanfare duration proportional to the number of correct sequences: 3 seconds for 4 correct up to 10 seconds for 20 correct or more.  In the win fanfare, send Fire channel On(=255) and Off (=0) signals lasting between 100 ms and 500 ms in duration while the song plays for 4 to 10 seconds.  In either win or lose conditions, randomly the colors Red, Green, Blue, Yellow while the song plays.  

Provide a complete set of code, not deltas.  Do not truncate code for length.  Check all class functions and members against the most current version of the library.  Add comments beside each library to note the version referenced.  You are a professional coder with ten years of experience.  Use literate programming techniques, adding plain-English comments throughout the code to explain what the code does and why.  Code in a way to avoid compilation errors, making sure that the functions used follow the library prototypes exactly, including data types.  

*/

// post prompt clean-up.

// Chiptunes need to be included in the code as tones.