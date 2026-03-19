package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Notes - adjust LEDConstants.LED_BRIGHTNESS to limit current draw
 * Correspondigly keep brightness brightness in the range of 0.1-0.2
 * Color can be changed to custom parameteres via standard RGB scale */ 


public class LEDController extends SubsystemBase {
  // Progress fill animation
  private static final double kFillDurationSeconds = 1.0; // Duration of the fill animation
  private static final double kProgressLoopPauseSeconds = kFillDurationSeconds;
  private boolean colorProgressActive = false; 

  private enum LimitSwitchPatternState { INACTIVE, PROGRESS, OFF }
  private LimitSwitchPatternState limitSwitchPatternState = LimitSwitchPatternState.INACTIVE;
  private final Timer limitSwitchPatternTimer = new Timer();
  private boolean limitSwitchProgressStarted = false;

  
  // Blinking pattern
  private final Timer blinkTimer = new Timer(); // Blink timer
  private boolean blinkingColor = false; // Boolean value for blink start/stop
  private double blinkRunSeconds = 0.0; // How long the lights stay on/off for
  private int blinkColorR = 0, blinkColorG = 0, blinkColorB = 0;
  private int blinkColorR2 = 0, blinkColorG2 = 0, blinkColorB2 = 0;

  // Snake animation state
  private static final int kSnakeLength = 20;
  private final Timer snakeTimer = new Timer();
  private boolean snakeActive = false;
  private boolean snakeLooping = false;
  private double snakeSpeedLedsPerSecond = 45.0;
  private int snakeR = 0, snakeG = 0, snakeB = 0;
  private int snakeR2 = 0, snakeG2 = 0, snakeB2 = 0;

  // LED strip components
  private final AddressableLED m_led; // The LED strip
  private final AddressableLEDBuffer m_ledBuffer; // Buffer for LED data
  private final Timer m_timer = new Timer(); // Timer for animation timing

  // Enum for preset colors
  public enum LEDColor { // Define some preset colors
    RED(255, 0, 0),
    BLUE(0, 0, 255),
    GREEN(0, 255, 0),
    YELLOW(255, 255, 0),
    ORANGE(255, 128, 0),
    TEAL (0, 255, 255),
    PURPLE(127, 0, 128),
    PINK(255, 51, 255),
    WHITE(255, 255, 255),
    OFF(0, 0, 0);
    
    public final int r, g, b;
    LEDColor(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  /** Called once at the beginning of the robot program. */
  public LEDController() {
    // PWM port must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(LEDConstants.PWM_PORT);

    // Set the strip length once, then update data as needed
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.NUM_OF_LEDS); 
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    // Begin timing the fill animation
    m_timer.start();
  }

  @Override
  // Calling of methods (patterns) can be omitted from here if they're called as inputs 
  public void periodic() { 


    runSnakeAnimation();
    runBlinkPattern(); // leave this running to allow blink pattern to function
    m_led.setData(m_ledBuffer);
  }

  // Overload for applyProgress to accept LEDColor enum
  // - LEDController.applyProgress(LEDColor.TR_RED, 0.5) will light up half the strip red
  public void applyProgress(LEDColor color, double m_progress) {
    applyProgress(color.r, color.g, color.b, m_progress);
  }

  public boolean isColorProgressActive() {
    return colorProgressActive;
  }

  /**  
   * Progressive light up method - LEDController.applyProgress(255, 0, 0, 0.5) will light up half the strip red
   * 
   * @param r Red component (0-255), g Green component (0-255), b Blue component (0-255)
   * @param m_progress Progress value from 0.0 to 1.0 indicating how much of the strip to light up
   */
  private void applyProgress(int r, int g, int b, double m_progress) {
    if (!colorProgressActive){
        m_timer.reset(); m_timer.start(); colorProgressActive = true; } // Flag for one-time progress start
    m_progress = Math.min(1.0, m_timer.get() / kFillDurationSeconds);
    int litLeds = (int) Math.ceil(m_progress * m_ledBuffer.getLength());
    litLeds = Math.max(0, Math.min(litLeds, m_ledBuffer.getLength()));

    r = (int) (r * LEDConstants.LED_BRIGHTNESS);
    g = (int) (g * LEDConstants.LED_BRIGHTNESS);
    b = (int) (b * LEDConstants.LED_BRIGHTNESS);
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i < litLeds) {
        m_ledBuffer.setRGB(i, r, g, b); 
      } else {
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }
    }
    m_led.setData(m_ledBuffer);
     if (m_progress >= 1.0) {
        colorProgressActive = false; // reset flag when progress is complete
        m_timer.stop();
     }
  }

   // Overload for applyColorSolid to accept LEDColor enum 
   // - LEDController.applyColorSolid(LEDColor.TR_RED) will set the whole strip red
  public void applyColorSolid(LEDColor color){
    applyColorSolid(color.r, color.g, color.b, LEDConstants.NUM_OF_LEDS);
  }

  /**
   * Solid color method - LEDController.applyColorSolid(255, 0, 0) will set the whole strip red
   * 
   * @param r Red component (0-255), g Green component (0-255), b Blue component (0-255)
   * @param LEDConstants.NUM_OF_LEDS Number of LEDs in the strip
   */
  private void applyColorSolid(int r, int g, int b, int ledLength){ 
    r = (int) (r * LEDConstants.LED_BRIGHTNESS);
   g = (int) (g * LEDConstants.LED_BRIGHTNESS);
   b = (int) (b * LEDConstants.LED_BRIGHTNESS); // keep brightness param low (.1-.2) to avoid current draw
    for (int i = 0; i < LEDConstants.NUM_OF_LEDS; i++) {
        m_ledBuffer.setRGB(i, r, g, b); // instantaneous blue at reduced brightness
    }
    m_led.setData(m_ledBuffer);
  }

  // Turn off all LEDs - LEDController.applyOff() will turn off the strip
  private void applyOff() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);  // all channels off
    }
    m_led.setData(m_ledBuffer);
  }

  /** Starts the teal progress loop used while the intake limit switch is held. */
  public void startLimitSwitchProgressLoop() {
    if(limitSwitchPatternState != LimitSwitchPatternState.INACTIVE) return;
    blinkingColor = false;
    snakeActive = false;
    blinkTimer.stop();
    snakeTimer.stop();
    startLimitSwitchProgressPhase();
  }

  /** Runs the teal progress loop, swapping between progress and off after each fill. */
  public void runLimitSwitchProgressLoop() {
    if (limitSwitchPatternState == LimitSwitchPatternState.INACTIVE) {
      startLimitSwitchProgressPhase();
    }

    if (limitSwitchPatternState == LimitSwitchPatternState.PROGRESS) {
      applyProgress(LEDColor.GREEN, 0.0);
      if (!limitSwitchProgressStarted && colorProgressActive) {
        limitSwitchProgressStarted = true;
      }
      if (limitSwitchProgressStarted && !colorProgressActive) {
        enterLimitSwitchOffState();
      }
    } else if (limitSwitchPatternState == LimitSwitchPatternState.OFF
        && limitSwitchPatternTimer.hasElapsed(kProgressLoopPauseSeconds)) {
      startLimitSwitchProgressPhase();
    }
  }

  private void enterLimitSwitchOffState() {
    limitSwitchPatternState = LimitSwitchPatternState.OFF;
    limitSwitchProgressStarted = false;
    limitSwitchPatternTimer.reset();
    limitSwitchPatternTimer.start();
    colorProgressActive = false;
    m_timer.stop();
    applyOff();
  }

  private void startLimitSwitchProgressPhase() {
    limitSwitchPatternState = LimitSwitchPatternState.PROGRESS;
    limitSwitchProgressStarted = false;
    limitSwitchPatternTimer.stop();
    limitSwitchPatternTimer.reset();
    colorProgressActive = false;
  }

  /** Stops the teal progress loop and clears the strip. */
  public void stopLimitSwitchProgressLoop() {
    limitSwitchPatternState = LimitSwitchPatternState.INACTIVE;
    limitSwitchProgressStarted = false;
    limitSwitchPatternTimer.stop();
    limitSwitchPatternTimer.reset();
    colorProgressActive = false;
    m_timer.stop();
    applyOff();
  }

  /** Stop all animations and turn the strip off. */
  public void turnOffAll() {
    limitSwitchPatternState = LimitSwitchPatternState.INACTIVE;
    limitSwitchPatternTimer.stop();
    limitSwitchPatternTimer.reset();
    limitSwitchProgressStarted = false;
    colorProgressActive = false;
    blinkingColor = false;
    snakeActive = false;
    blinkTimer.stop();
    snakeTimer.stop();
    m_timer.stop();
    applyOff();
  }
  
  // Overload for applyColorBlink to accept LEDColor enum 
  // - LEDController.applyColorBlink(LEDColor.TR_RED, LEDColor.OFF, 10.0) will blink red for 10 seconds
 public void applyColorBlink(LEDColor color, LEDColor color2, double blinkRunSeconds){
    applyColorBlink(color.r, color.g, color.b, color2.r, color2.g, color2.b, blinkRunSeconds);
  }

 public void applyBlinkColor(LEDColor color){
    applyColorBlink(color, LEDColor.OFF, blinkRunSeconds);
 }
  /**
   * Blinking color method - LEDController.applyColorBlink(255, 0, 0, 0, 0, 0, 10.0) will blink red for 10 seconds
   * @param r Red component of first color (0-255), g Green component (0-255), b Blue component (0-255)
   * @param r2 Red component of second color (0-255), g2 Green component (0-255), b2 Blue component (0-255)
   * @param blinkRunSeconds Duration in seconds for the blinking effect
   */
  public void applyColorBlink(int r, int g, int b, int r2, int g2, int b2, double blinkRunSeconds){
    this.blinkRunSeconds = blinkRunSeconds;
    blinkingColor = true;
    snakeActive = false;
    snakeTimer.stop();
    blinkColorR = r;
    blinkColorG = g;
    blinkColorB = b;
    blinkColorR2 = r2;
    blinkColorG2 = g2;
    blinkColorB2 = b2;
    
    blinkTimer.reset();
    blinkTimer.start();
  }

  private void runBlinkPattern() {
    if (!blinkingColor) return;
    
    double t = blinkTimer.get();
    if (blinkRunSeconds > 0 && t >= blinkRunSeconds){
        blinkingColor = false;
        blinkTimer.stop();
        applyOff();
        return;
    }
    boolean on = ((int) (t/0.2)) % 2 == 0; //0.5s on, 0.5s off
    if (on) {
        applyColorSolid(blinkColorR, blinkColorG, blinkColorB, LEDConstants.NUM_OF_LEDS); 
    } else {
        applyColorSolid(blinkColorR2, blinkColorG2, blinkColorB2, LEDConstants.NUM_OF_LEDS);
    }
 }

 // Overload for startSnakeAnimation to accept LEDColor enum 
 // - LEDController.startSnakeAnimation(LEDColor.TR_RED, LEDColor.OFF, true) will start a red snake on a black background
 public void startSnakeAnimation(LEDColor color, LEDColor bgcolor, boolean loop) {
    startSnakeAnimation(color.r, color.g, color.b, bgcolor.r, bgcolor.g, bgcolor.b, 40.0, loop);
   }
 
  /**
   * Starts a snake-like animation that moves a short lit segment across the strip. 
   * - LEDController.startSnakeAnimation(255, 0, 0, 0, 0, 0, 40.0, true) will start a red snake on a black background
   *
   * @param r Red component of the snake color (0-255), g Green component (0-255), b Blue component (0-255)
   * @param r2 Red component of the background color (0-255), g2 Green component (0-255), b2 Blue component (0-255) 
   * @param ledsPerSecond Speed of the segment head in LEDs per second
   * @param loop If true the segment wraps around when it reaches the end
   */
  public void startSnakeAnimation(int r, int g, int b, int r2, int g2, int b2, double ledsPerSecond, boolean loop) {
    snakeR = (int) (r * LEDConstants.LED_BRIGHTNESS);
    snakeG = (int) (g * LEDConstants.LED_BRIGHTNESS);
    snakeB = (int) (b * LEDConstants.LED_BRIGHTNESS);
    snakeR2 = (int) (r2 * LEDConstants.LED_BRIGHTNESS);
    snakeG2 = (int) (g2 * LEDConstants.LED_BRIGHTNESS);
    snakeB2 = (int) (b2 * LEDConstants.LED_BRIGHTNESS);
    snakeSpeedLedsPerSecond = Math.max(1.0, ledsPerSecond);
    snakeLooping = loop;
    snakeActive = true;
    snakeTimer.reset();
    snakeTimer.start();
    // Draw the first frame immediately so the strip updates even before periodic runs.
    runSnakeAnimation();
    m_led.setData(m_ledBuffer);

    // Pause other effects so the snake remains visible
    blinkingColor = false;
    blinkTimer.stop();
    }

  // Runs the snake animation, updating LED colors based on elapsed time
  private void runSnakeAnimation() {
    if (!snakeActive) return;

    int stripLength = m_ledBuffer.getLength();
    double elapsed = snakeTimer.get();
    int headIndex = (int) Math.floor(elapsed * snakeSpeedLedsPerSecond);

    if (!snakeLooping && headIndex - kSnakeLength >= stripLength) {
      snakeActive = false;
      applyOff();
      return;
    }

    for (int i = 0; i < stripLength; i++) {
        m_ledBuffer.setRGB(i, snakeR2, snakeG2, snakeB2); // Set background color
    }

    int wrappedHead = snakeLooping ? Math.floorMod(headIndex, stripLength) : headIndex;
    for (int offset = 0; offset < kSnakeLength; offset++) {
      int ledIndex = wrappedHead - offset;
      if (snakeLooping) {
        ledIndex = Math.floorMod(ledIndex, stripLength);
      }

      if (ledIndex < 0 || ledIndex >= stripLength) {
        continue;
      }

      double intensity = 1.0 - (double) offset / kSnakeLength; // Slight fade down the tail
      int r = (int) (snakeR * intensity);
      int g = (int) (snakeG * intensity);
      int b = (int) (snakeB * intensity);
      m_ledBuffer.setRGB(ledIndex, r, g, b);
    }
  }

 
}
