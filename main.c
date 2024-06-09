// Standard includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "pin.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "spi.h"
#include "uart.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "timer.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"

// Common interface includes
#include "oled_test.h"
#include "uart_if.h"
#include "i2c_if.h"
#include "gpio_if.h"
#include "timer_if.h"

#include "pin_mux_config.h"

///////////////////////////Scale includes
// Standard includes
#include <string.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"

// Common interface includes
#include "uart_if.h"
#include "pin_mux_config.h"
#include "stdint.h"

#include "i2c_if.h"

#include <stdio.h>

#define NUM_SAMPLES 10
#define SAMPLE_SIZE 24
///////////////////////////End Scale includes

#define APPLICATION_VERSION     "1.4.0"
#define UART_PRINT              Report
#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

// Define macros for GPIO pins and ports
#define IR_REMOTE_GPIO_BASE GPIOA0_BASE
#define IR_REMOTE_GPIO_PIN  0x80 // Assuming PIN_61 corresponds to GPIOA0 and pin 6 (0x40)

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

#define ZERO  0b1001000001101111
#define ONE   0b0000000011111111
#define TWO   0b1000000001111111
#define THREE 0b0100000010111111
#define FOUR  0b1100000000111111
#define FIVE  0b0010000011011111
#define SIX   0b1010000001011111
#define SEVEN 0b0110000010011111
#define EIGHT 0b1110000000011111
#define NINE  0b0001000011101111
#define ENTER 0b0111000010001111 // changed ENTER to MUTE for Jen
#define LAST  0b0000100011110111

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

// OLED Display Definitions
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define CHAR_WIDTH 6      // Width of a character in pixels
#define CHAR_HEIGHT 8     // Height of a character in pixels
#define MAX_CHARS_PER_LINE (SCREEN_WIDTH / CHAR_WIDTH)  // Calculate max chars per line

// Item Definitions
#define MAX_ITEM_NAME_LENGTH 100

// Global variables to store pulse width information
volatile unsigned long buffer[100];
volatile int result = 50;
volatile int bitcount = 0;
volatile int x = 0;
volatile int readyForProcess = 0;
volatile int disable = 0;
volatile unsigned long previousChar = 0;
volatile int text_x = 5, text_y = 68;
volatile int get_x = 5, get_y = 4;
volatile int textSize = 0;
volatile int textReady = 0;
volatile int getSize = 0;
volatile int prevSize = 0;
volatile int state = 0;
volatile float calibrationFactor;
volatile unsigned long tareValue;
unsigned long knownWeight = 66;

typedef struct textLStruct {
    unsigned int x;
    unsigned int y;
    char letter;
} textLStruct;

textLStruct textMessage[100];
textLStruct getMessage[100];
textLStruct prevMessage[100];

typedef enum {
    STATE_START_SCREEN,
    STATE_ENTER_ITEM_NAME,
    STATE_PLACE_SINGLE_ITEM,
    STATE_PLACE_REST_ITEMS,
    STATE_INVENTORY_TRACKING,
    STATE_EMAIL_ALERT,
    STATE_MAX
} State;

State currentState = STATE_START_SCREEN;

bool stateInitialized = false;

char itemName[100]; // item name

bool itemInitialized = false;

///////////////////////////Scale functions
///////////////////////////Scale functions
///////////////////////////Scale functions
///////////////////////////Scale functions
//  function delays 3*ulCount cycles
static void delay(unsigned long ulCount){
	int i;

  do{
    ulCount--;
		for (i=0; i< 65535; i++) ;
	}while(ulCount);
}

void clk(void){
  GPIOPinWrite(GPIOA2_BASE, 0x2, 0xFF);
  GPIOPinWrite(GPIOA2_BASE, 0x2, 0x00);
}

// Function to read a single sample from the sensor
long ReadSingleSample(void) {
    int i;
    unsigned long sample[SAMPLE_SIZE];
    GPIOPinWrite(GPIOA2_BASE, 0x2, 0x00); // SCK = Low
    while (GPIOPinRead(GPIOA0_BASE, 0x40) != 0x00) {
        // Wait until data line goes low
    }

    for (i = 0; i < 24; i++) {
        clk();
        sample[i] = GPIOPinRead(GPIOA0_BASE, 0x40) ? 1 : 0;
    }
    clk();

    // Convert the 24-bit array to a signed 32-bit integer
    long result = 0;
    for (i = 0; i < SAMPLE_SIZE; i++) {
        result |= sample[i] << (SAMPLE_SIZE - 1 - i);
    }

    // If the sign bit (MSB) is set, extend the sign to the 32-bit signed integer
    if (result & 0x800000) { // 0x800000 is the sign bit for 24-bit data
        result |= ~0xFFFFFF; // Extend the sign to 32-bit
    }

    return result;
}

// Function to read samples and calculate averages
void ReadScale(long averages[NUM_SAMPLES]) {
    long samples[NUM_SAMPLES];
    int j = 0;

    // Report("reading samples\n\r");
    for (j = 0; j < NUM_SAMPLES; j++) {
        samples[j] = ReadSingleSample();
        // Report("Data stored in array at index %d is: %ld\n\r", j, samples[j]);
    }
    // Report("done reading samples\n\r");

    // Calculate the average of the values in samples array
    long sum = 0;
    for (j = 0; j < NUM_SAMPLES; j++) {
        sum += samples[j];
    }
    long average = sum / NUM_SAMPLES;

    // Report("Average of the 10 samples: %ld\n\r", average);
    
    for (j = 0; j < NUM_SAMPLES; j++) {
        averages[j] = samples[j];
    }

    // delay(500);
}

// Function to initialize the scale
void InitializeScale(void) {
    // Initialize GPIO and any other required peripherals
    // Placeholder for actual initialization code
}

// Function to tare the scale (zero it)
void TareScale(long *tareValue) {
    int i;
    long averages[NUM_SAMPLES];
    // Report("going into read scale\n\r");
    ReadScale(averages);
    // Report("out of read scale\n\r");
    *tareValue = 0;
    for (i = 0; i < NUM_SAMPLES; i++) {
        *tareValue += averages[i];
    }
    *tareValue /= NUM_SAMPLES;

    Report("Tare value set to %ld\n\r", *tareValue);
}

// Function to calibrate the scale with a known weight
void CalibrateScale(long tareValue, unsigned long knownWeight, float *calibrationFactor) {
    int i;
    long averages[NUM_SAMPLES];
    ReadScale(averages);

    long rawValue = 0;
    for (i = 0; i < NUM_SAMPLES; i++) {
        rawValue += averages[i];
    }
    rawValue /= NUM_SAMPLES;

    *calibrationFactor = (float)knownWeight / (rawValue - tareValue);

    Report("Calibration factor set to %f\n\r", *calibrationFactor);
}

// Function to get weight using the calibration factor and tare value
float GetWeight(long tareValue, float calibrationFactor) {
    int i;
    long averages[NUM_SAMPLES];
    ReadScale(averages);

    long rawValue = 0;
    for (i = 0; i < NUM_SAMPLES; i++) {
        rawValue += averages[i];
    }
    rawValue /= NUM_SAMPLES;

    return (rawValue - tareValue) * calibrationFactor;
}

// Function to display the weight
void DisplayWeight(float weight) {
    Report("Weight: %.2f\n\r", weight);
}

void test(void)
{
    Message("Welcome to scale application\n\r");

    unsigned long tareValue;
    TareScale(&tareValue);

    Message("Place known weight on scale\n\r");

    delay(1000);

    Message("Calibrating...\n\r");

    float calibrationFactor;
    unsigned long knownWeight = 66; // Example known weight in grams
    CalibrateScale(tareValue, knownWeight, &calibrationFactor);

    Message("Remove known weight from scale\n\r");

    delay(1000);

    Message("Scale ready for use!\n\r");
   
    while(1) {
    // long averages[NUM_SAMPLES];
    // ReadScale(averages);
    float weight = GetWeight(tareValue, calibrationFactor);
    DisplayWeight(weight);
    }

}
///////////////////////////End Scale functions
///////////////////////////End Scale functions
///////////////////////////End Scale functions
///////////////////////////End Scale functions

// UART interupt handler
void UARTIntHandler(void) {

    UART_PRINT("inside UART handler\n\r");

    UARTIntStatus(UARTA1_BASE, true);

    // While there are available chars over UART1
    while(UARTCharsAvail(UARTA1_BASE))
    {
        char c = UARTCharGet(UARTA1_BASE);
        if(c == '\0') {
            // Flag that enables DisplayGet function
            textReady = 1;
        }
        else {
            // stores the get message information such as the letter and letter positioning
            getMessage[getSize].letter = c;
            getMessage[getSize].x = get_x;
            getMessage[getSize].y = get_y;
            getSize++;
            get_x += 7;
            if(get_x >= 124) {
                get_x = 5;
                get_y += 10;
            }
        }
    }
}

void send(void) {
    if(textSize > 0) {
        int i;
        for(i = 0; i < textSize; i++) {
            while(UARTBusy(UARTA1_BASE));
            // UART_PRINT("Put char in UART1\n\r");
            UARTCharPut(UARTA1_BASE, textMessage[i].letter);
        }
        UARTCharPut(UARTA1_BASE,'\0');
        // erases the message that was just send by the user
        for(i = 0; i < textSize; i++) {
            drawChar(textMessage[i].x, textMessage[i].y, textMessage[i].letter, BLACK, BLACK, 1);
        }
        textSize = 0;
        text_x = 5;
        text_y = 68;
    }
}

char findLetter(unsigned long value) {
    char letter;
    switch(value) {
        case ZERO:
            letter = ' ';
            break;
        case ONE:
            letter = '?';
            break;
        case TWO:
            letter = 'a';
            break;
        case THREE:
            letter = 'd';
            break;
        case FOUR:
            letter = 'g';
            break;
        case FIVE:
            letter = 'j';
            break;
        case SIX:
            letter = 'm';
            break;
        case SEVEN:
            letter = 'p';
            break;
        case EIGHT:
            letter = 't';
            break;
        case NINE:
            letter = 'w';
            break;
        case ENTER:
            letter = '+';
            break;
        case LAST:
            letter = '-';
            break;
        default:
            letter = '/';
            break;
    }
    return letter;
}

char multiPress(char letter, unsigned long value) {
    char newLetter;
    switch(value) {
        case ONE:
            if(letter == '?') { newLetter = '.'; }
            else if(letter == '.') { newLetter = ','; }
            else { newLetter = '?'; }
            break;
        case TWO:
            if(letter == 'c') { newLetter = 'a'; }
            else { newLetter = letter + 1; }
            break;
        case THREE:
            if(letter == 'f') { newLetter = 'd'; }
            else { newLetter = letter + 1; }
            break;
        case FOUR:
            if(letter == 'i') { newLetter = 'g'; }
            else {newLetter = letter + 1; }
            break;
        case FIVE:
            if(letter == 'l') { newLetter = 'j'; }
            else { newLetter = letter + 1; }
            break;
        case SIX:
            if(letter == 'o') { newLetter = 'm'; }
            else { newLetter = letter + 1; }
            break;
        case SEVEN:
            if(letter == 's') { newLetter = 'p'; }
            else { newLetter = letter + 1; }
            break;
        case EIGHT:
            if(letter == 'v') { newLetter = 't'; }
            else { newLetter = letter + 1; }
            break;
        case NINE:
            if(letter == 'z') { newLetter = 'w'; }
            else { newLetter = letter + 1; }
            break;
        default:
            newLetter = letter;
            break;
    }
    return newLetter;
}


// Initialize SysTick timer
void initSysTick(void) {
    SysTickPeriodSet(0xFFFFFF); // Set the SysTick period to its maximum value for longer duration measurement
    SysTickEnable();
    SysTickIntDisable();
}

void displayGet() {
    if(textReady) {
        textReady = 0;
        int i;
        for(i = 0; i < prevSize; i++) {
            drawChar(prevMessage[i].x, prevMessage[i].y, prevMessage[i].letter, BLACK, BLACK, 1);
        }
        for(i = 0; i < getSize; i++) {
            drawChar(getMessage[i].x, getMessage[i].y, getMessage[i].letter, RED, RED, 1);
            prevMessage[i] = getMessage[i];
        }
        prevSize = getSize;
        get_x = 5;
        get_y = 4;
        getSize = 0;
    }
}

// dup interupt handler that skips over the extra bits of the IR remote pattern that we do not need
void IRRemoteInterruptHandlerdup(void) {
    MAP_GPIOIntClear(IR_REMOTE_GPIO_BASE, IR_REMOTE_GPIO_PIN);
}

// Interrupt handler for IR remote input
void IRRemoteInterruptHandler(void) {
    uint32_t currentRisingEdgeTime = SysTickValueGet();  // gets the current time since the reset
    SysTickIntDisable();
    uint32_t pulseWidth = 0xFFFFFF - currentRisingEdgeTime;  // gets the pulse width by subtracting the max value from the current time

    // criteria to determine if pulseWidth is a 0 or a 1
    if (pulseWidth > (0) && pulseWidth < (100000)) {
        result = 0;
    }
    else{
        result = 1;
    }

    // ensures that no garbage bits are stored in the buffer while the first pattern is being processed
    if(!(disable)){
        buffer[bitcount] = result;
        bitcount++;
    }
    
    // once all vital bits are in the buffer it is time to process them
    if(bitcount == 35){
        disable = 1;
        // Flag to enable pattern processing
        readyForProcess = 1;
        bitcount = 0;
        //this is the timer that effects how long the user has to multi press
        Timer_IF_Start(TIMERA2_BASE, TIMER_A, 2000);
    }
    
    MAP_GPIOIntClear(IR_REMOTE_GPIO_BASE, IR_REMOTE_GPIO_PIN);
    reloadSysTickImmediately(0xFFFFFF); 
    SysTickEnable(); 
}


void formItem(char letter, int dup, unsigned long letterBin){
    if (letter == '-') {
        if (textSize > 0) {
            // delete a character
            // itemName[textSize] - '\0';
            textSize--;
            drawChar(textMessage[textSize].x, textMessage[textSize].y, textMessage[textSize].letter, BLACK, BLACK, 1);
        }
        if(text_x >= 12) {
                text_x -= 7;
            }
            else if(text_x == 5) {
                if(text_y >= 78) {
                    text_y -= 10;
                    text_x = 117;
                }
            }
    } else if (letter == '+') {
        int i;
        for (i = 0; i < textSize; i++){
            UART_PRINT("%c", textMessage[i].letter);
        }
        UART_PRINT("\n\r");
        send();
        itemInitialized = true;
    } else {
        if (textSize < 100) {
            if (dup == 1) {
                int index = textSize - 1;
                char let = textMessage[index].letter;

                drawChar(textMessage[index].x, textMessage[index].y, let, BLACK, BLACK, 1);

                textMessage[index].letter = multiPress(let, letterBin);
                drawChar(textMessage[index].x, textMessage[index].y, textMessage[index].letter, BLUE, BLUE, 1);
                
            } else {
                textLStruct newLet;
                newLet.x = text_x;
                newLet.y = text_y;
                newLet.letter = letter;
                textMessage[textSize] = newLet;
                drawChar(text_x, text_y, letter, BLUE, BLUE, 1);

                text_x += 7;
                if(text_x >= 124) {
                    text_x = 5;
                    text_y += 10;
                }

                textSize++;
            }
        }
    }
}

static void TimeoutHandler(void)
{
    Timer_IF_InterruptClear(TIMERA2_BASE);
    previousChar = -1;
}


// Function to print a string to the OLED using drawChar
void printToOLED(const char* str) {
    int x = 0;  // Starting x position
    int y = 0;  // Starting y position

    while (*str) {
        // Draw the character at the current position
        drawChar(x, y, *str, WHITE, BLACK, 1);

        // Move to the next position
        x += CHAR_WIDTH;

        // If we reach the end of a line, move to the next line
        if (x >= SCREEN_WIDTH) {
            x = 0;
            y += CHAR_HEIGHT;

            // If we reach the bottom of the screen, stop drawing (or handle scrolling)
            if (y >= SCREEN_HEIGHT) {
                break;
            }
        }

        // Move to the next character in the string
        str++;
    }

}

void handleStartScreen() {
    fillScreen(BLACK);
    printToOLED("Welcome to Smart Inventory Management System!");
}

void handleEnterItemName() {
    fillScreen(BLACK);
    printToOLED("Enter the name of your item.");

    disable = 0;
    readyForProcess = 0;
    x = 0;

    GPIO_IF_ConfigureNIntEnable(GPIOA0_BASE, 0x80, GPIO_FALLING_EDGE, IRRemoteInterruptHandler);

    while(1) {
            if (readyForProcess) {
                GPIO_IF_ConfigureNIntEnable(GPIOA0_BASE, 0x80, GPIO_FALLING_EDGE, IRRemoteInterruptHandlerdup);

                delay(25);

                int dup = 0;
                unsigned long bufConcat = 0;
                char letter = NULL;

                for (x = 18; x < 34; x++) {
                    bufConcat <<= 1;
                    bufConcat |= buffer[x];
                    if(x == 33) {}
                }

                letter = findLetter(bufConcat);
                if (previousChar == 0 || (previousChar != bufConcat && letter != '/')) {
                    previousChar = bufConcat;
                } else if (previousChar == bufConcat) {
                    if (letter == '/') {
                        previousChar = previousChar;
                    } else {
                        previousChar = bufConcat;
                    }

                    dup = 1;
                }

                if (letter != '/') {
                    formItem(letter, dup, bufConcat);
                }

                disable = 0;
                readyForProcess = 0;
                x = 0;

                if (itemInitialized == true){
                    break;
                }


                GPIO_IF_ConfigureNIntEnable(GPIOA0_BASE, 0x80, GPIO_FALLING_EDGE, IRRemoteInterruptHandler);
            }
        }
}


void handlePlaceSingleItem() {
    fillScreen(BLACK);
    printToOLED("Calibrate system by placing calibration item on scale now.");


    // Report("going into tar\n\r");
    TareScale(&tareValue);
    // Report("out of tar\n\r");

    delay(500);

    fillScreen(BLACK);
    printToOLED("Calibrating...");

    CalibrateScale(tareValue, knownWeight, &calibrationFactor);

    delay(500);

    fillScreen(BLACK);
    printToOLED("Remove calibration item from the scale now.");

    delay(500);

    fillScreen(BLACK);
    printToOLED("Scale ready for use!");    

    delay(500);

    fillScreen(BLACK);
    printToOLED("Place single item on the scale now.");

    delay(500);

    float weight = GetWeight(tareValue, calibrationFactor);
    DisplayWeight(weight);

    fillScreen(BLACK);
    printToOLED("Press enter when done.");
    // calibrate weight here
}

void handlePlaceRestItems() {
    fillScreen(BLACK);
    printToOLED("Place the rest of the items. Press enter when done");

    // calculate total weight
}

void handleInventoryTracking() {
    fillScreen(BLACK);
    printToOLED("Inventory Tracking Active");

    while(1){
        float weight = GetWeight(tareValue, calibrationFactor);
        DisplayWeight(weight);
        if (weight > -5 && weight < 5 ){
            break;
        }
    }

    fillScreen(BLACK);
    printToOLED("Inventory Empty!");

    // calculate the total number of items
}

void handleEmailAlert() {
    fillScreen(BLACK);
    printToOLED("Threshold reached! Sending email alert...");
}

void MasterMain()
{
    MAP_SPIReset(GSPI_BASE);

    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    MAP_SPIEnable(GSPI_BASE);

    Message("Send/Recieve Message App\n\r");

    Adafruit_Init();
    fillScreen(BLACK);
    printToOLED("Welcome to Smart Inventory Management System, press enter to continue!");


    UART_PRINT("OLED Screen is ready\n\r");
    
    initSysTick();

    GPIO_IF_ConfigureNIntEnable(GPIOA0_BASE, 0x80, GPIO_FALLING_EDGE, IRRemoteInterruptHandler);

    Timer_IF_Init(PRCM_TIMERA2, TIMERA2_BASE, TIMER_CFG_ONE_SHOT, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA2_BASE, TIMER_A, TimeoutHandler);

    UARTIntRegister(UARTA1_BASE, UARTIntHandler);
    MAP_UARTIntEnable(UARTA1_BASE, UART_INT_RX | UART_INT_RT);

    //display start screen and don't move forward until enter is pressed



    //Once enter is pressed, go to next screen which asks user to enter the item name, the enter button will store the item name

    while(1) {
        if(readyForProcess){
            GPIO_IF_ConfigureNIntEnable(GPIOA0_BASE, 0x80, GPIO_FALLING_EDGE, IRRemoteInterruptHandlerdup);
            //delay to get rid of the garbage bits after the pattern is buffered
            delay(25);

            // UART_PRINT("Result is:\n\r");
            int dup = 0;
            unsigned long bufConcat = 0;
            char letter = NULL;
            for(x = 18; x < 34; x++) {
                // UART_PRINT("%d", buffer[x]);
                bufConcat <<= 1;
                bufConcat |= buffer[x];
                if(x == 33){
                    // UART_PRINT("\n\r");
                }
            }

            letter = findLetter(bufConcat);


            // Handle state transitions based on the detected letter
            switch (currentState) {
                case STATE_START_SCREEN:
                    if (letter == '+') {
                        UART_PRINT("Going to state 1\n\r");
                        currentState = STATE_ENTER_ITEM_NAME;
                        stateInitialized = false;
                    }
                    break;
                case STATE_ENTER_ITEM_NAME:
                    if (letter == '+') {
                        UART_PRINT("Going to state 2\n\r");
                        currentState = STATE_PLACE_SINGLE_ITEM;
                        stateInitialized = false;
                    }
                    break;
                case STATE_PLACE_SINGLE_ITEM:
                    if (letter == '+') {
                        UART_PRINT("Going to state 3\n\r");
                        currentState = STATE_PLACE_REST_ITEMS;
                        stateInitialized = false;
                    }
                    break;
                case STATE_PLACE_REST_ITEMS:
                    if (letter == '+') {
                        UART_PRINT("Going to state 4\n\r");
                        currentState = STATE_INVENTORY_TRACKING;
                        stateInitialized = false;
                    }
                    break;
                case STATE_INVENTORY_TRACKING:
                    // Implement threshold check here
                    if (letter == '+') {
                        UART_PRINT("Going to state 5\n\r");
                        currentState = STATE_EMAIL_ALERT;
                        stateInitialized = false;
                    }
                    break;
                case STATE_EMAIL_ALERT:
                    if (letter == '+') {
                            UART_PRINT("Reset\n\r");
                            currentState = STATE_START_SCREEN;
                            stateInitialized = false;
                        // Handle email alert
                    }
                    break;
                default:
                    currentState = STATE_START_SCREEN;
                    break;
            }

            // Call the appropriate state function based on the current state
            switch (currentState) {
                case STATE_START_SCREEN:
                    if (!stateInitialized) {
                        handleStartScreen();
                        stateInitialized = true;
                    }
                    break;
                case STATE_ENTER_ITEM_NAME:
                    if (!stateInitialized) {
                        bool itemInitialized = false;
                        handleEnterItemName();
                        stateInitialized = true;
                    }
                    break;
                case STATE_PLACE_SINGLE_ITEM:
                    if (!stateInitialized) {
                        handlePlaceSingleItem();
                        stateInitialized = true;
                    }
                    break;
                case STATE_PLACE_REST_ITEMS:
                    if (!stateInitialized) {
                        handlePlaceRestItems();
                        stateInitialized = true;
                    }
                    break;
                case STATE_INVENTORY_TRACKING:
                    if (!stateInitialized) {
                        handleInventoryTracking();
                        stateInitialized = true;
                    }
                    break;
                case STATE_EMAIL_ALERT:
                    if (!stateInitialized) {
                        handleEmailAlert();
                        stateInitialized = true;
                    }
                    break;
                default:
                    currentState = STATE_START_SCREEN;
                    stateInitialized = false;
                    break;
            }


            if(previousChar == 0 || (previousChar != bufConcat && letter != '/')){
                previousChar = bufConcat;
            }
            else if (previousChar == bufConcat) {
                if(letter == '/'){
                    previousChar = previousChar;
                }
                else{
                    previousChar = bufConcat;
                }
                dup = 1;
            }

            disable = 0;
            readyForProcess = 0;
            x = 0;
            GPIO_IF_ConfigureNIntEnable(GPIOA0_BASE, 0x80, GPIO_FALLING_EDGE, IRRemoteInterruptHandler);
        }
        //displayGet();
    }
}

static void
BoardInit(void)
{
#ifndef USE_TIRTOS

#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif

    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

static void
InitBoardUART(void)
{
    MAP_UARTConfigSetExpClk(UARTA1_BASE,MAP_PRCMPeripheralClockGet(UARTA1_BASE),
                            UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE));
}

void main()
{
    BoardInit();

    PinMuxConfig();

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    InitTerm();

    ClearTerm();

    InitBoardUART();

    Message("\n\n\n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\t\t        CC3200 Send/Recieve Application  \n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\n\n\n\r");

    MAP_PRCMPeripheralReset(PRCM_GSPI);

#if MASTER_MODE

    MasterMain();

#else

#endif

    while(1)
    {

    }
}

