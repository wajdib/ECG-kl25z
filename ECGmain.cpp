

#include "mbed.h"
//#include "MMA8451Q.h"
#include "TSISensor.h"
#include <math.h>
#include "ILI9341.h"
#include "SPI_STMPE610.h"
#include "Arial12x12.h"
#include "Arial24x23.h"
#include "Arial28x28.h"
#include "Arial43x48_numb.h"
#include <string>
#include "EKG.h"

#include "SeeedTouchScreen.h"

#include "PublicTypes.h"

#define PIN_XP          PTB3
#define PIN_XM          PTB1
#define PIN_YP          PTB2
#define PIN_YM          PTB0
#define PIN_MOSI        PTD2
#define PIN_MISO        PTD3
#define PIN_SCLK        PTD1
#define PIN_CS_TFT      PTA5
#define PIN_DC_TFT      PTD4
#define PIN_BL_TFT      PTA1
#define PIN_CS_SD       PTA4
#define PIN_CS_TSC      PTA13
#define PIN_TSC_INTR    PTD4
#define PIN_RESET_TFT   PTB10

//#define PIN_SDA PTC9
//#define PIN_SCL PTC8

#define MMA8451_I2C_ADDRESS (0x1d<<1)
//MMA8451Q *acc = 0 ;
TSISensor tsi;

DigitalOut backlight(PTA12);
ILI9341 TFT(SPI_8, 10000000,
	PIN_MOSI, PIN_MISO, PIN_SCLK,
	PIN_CS_TFT, PIN_RESET_TFT, PIN_DC_TFT, "Adafruit2.8");
SPI_STMPE610 TSC(PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS_TSC);

//initialize touch, only need x+, x-, y+ and y-
SeeedStudioTFTv2 pTouch(PIN_XP, PIN_XM, PIN_YP, PIN_YM,
	PIN_YM, PIN_YM, PIN_YM, PIN_YM, PIN_YM, PIN_YM, PIN_YM);

//DigitalIn pinD7(PTA12) ;

//serial communication to host PC
Serial pc(USBTX, USBRX); // tx, rx
void Tx_interrupt();
void Rx_interrupt();
void send_line();
void read_line();


// Circular buffers for serial TX and RX data - used by interrupt routines
const int buffer_size = 255;
// might need to increase buffer size for high baud rates
char tx_buffer[buffer_size + 1];
char rx_buffer[buffer_size + 1];
// Circular buffer pointers
// volatile makes read-modify-write atomic
volatile int tx_in = 0;
volatile int tx_out = 0;
volatile int rx_in = 0;
volatile int rx_out = 0;
// Line buffers for sprintf and sscanf
char tx_line[80];
char rx_line[80];


//i2c protocol for the MED-EKG

Ticker SendPacketTimer;


//LO+
//DigitalIn LOPlus(PTD5);
//LO-
DigitalIn LOMinus(PTD0);
//read analog in of the ECG sensor
AnalogIn ECGIN(PTE20);


bool PollECG;
int page = 0;
int prevPage = 99;
int numPage = 5;


extern void doMaze(void);

void initTFT(void)
{
	//Configure the display driver
	TFT.BusEnable(true);
	TFT.FastWindow(true);
	TFT.background(Black);
	TFT.foreground(White);
	TFT.set_orientation(1);
	wait(0.01);
	TFT.cls();
	TFT.BusEnable(false);
}

void screen1(void) // Welcome Screen
{
	TFT.BusEnable(true);
	backlight = 0;
	TFT.background(White);
	wait(0.1);
	TFT.cls();
	wait(0.1);

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.foreground(Red);
	TFT.locate(120, 8);
	TFT.printf("E.C.G.I");
	TFT.set_font((unsigned char*)Arial12x12);
	TFT.foreground(Blue);
	TFT.locate(109, 40);
	TFT.printf("By Wajdi Bazuhair");
	//TFT.locate(40, 120) ;
	//TFT.printf("with touch") ;





	//TFT.fillrect(50, 70, 229, 180, White) ;
	TFT.rect(50, 60, 270,/*70+50*/ 105, Red);

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Black);
	TFT.locate(100, 75);
	TFT.printf("Register");




	///TFT.fillrect(10, 190, 229, 240, White) ;
	TFT.rect(50, 115, 270, 160, Red);

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Black);
	TFT.locate(100, 130);
	TFT.printf("Login");


	// TFT.fillrect(10, 250, 229, 300, White) ;
	TFT.rect(50, 170, 270, 215, Red);


	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Black);
	TFT.locate(100, 185);
	TFT.printf("List Users");





	TFT.set_font((unsigned char*)Arial12x12);
	TFT.foreground(Black);
	TFT.foreground(Blue);
	TFT.locate(50, 225);
	TFT.printf("University Of Texas at San antonio");
	// TFT.locate(30, 200) ;
	// TFT.printf("freescale FRDM-KL25Z with") ;
	//TFT.locate(30, 220) ;
	// TFT.printf("a program developed in mbed") ;
	// TFT.foreground(Green) ;
	//TFT.locate(30, 260) ;
	//TFT.printf("To advance demo page, touch") ;
	// TFT.locate(30, 280) ;
	//TFT.printf("and hold right side of screen") ;
	//TFT.locate(30, 300) ;
	//TFT.printf("until the next screen starts") ;
	TFT.BusEnable(false);
	backlight = 1;
}
float clip(float src)
{
	float value;
	value = src;
	if (value < 0.0) {
		value = 0.0;
	}
	else if (value > 2.0) {
		value = 2.0;
	}
	return(value);
}




void screen2() // Graphics
{

	int t = 0;
	int pt = 0; // previous t
	// int  x, y ;
	unsigned int data[3]; // for x, y, z
	unsigned int prev[3];
	unsigned short signalHeight = 75;
	unsigned short xoffset = 25;

	unsigned short paneX[2] = { 25, 224 };

	//Draw some graphics
	int i, x[2], y[2];
	backlight = 1;
	TFT.BusEnable(true);
	TFT.background(Black);
	TFT.foreground(White);

	TFT.set_font((unsigned char*)Arial12x12);
	TFT.locate(90, 0);
	TFT.printf("ECG Capture");

	x[0] = 25;
	x[1] = 224;
	y[0] = 20;
	y[1] = 219;
	for (i = 20; i < 220; i += 10) {
		TFT.line(i + 5, y[0], i + 5, y[1], Blue);
		TFT.line(x[0], i, x[1], i, Blue);
	}

	TFT.rect(x[0], y[0], x[1], y[1], White);
	TFT.locate(10, 20);
	TFT.printf("A");
	TFT.locate(0, 115);
	TFT.printf("0.0");
	TFT.locate(0, 225);
	TFT.printf("0.0");
	TFT.locate(215, 225);
	TFT.printf("T");


	TFT.fillrect(10, 230, 229, 280, White);
	TFT.rect(10, 230, 229, 280, Red);
	TFT.rect(11, 231, 230, 281, Red);

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Black);
	TFT.locate(65, 244);
	TFT.printf("Cancel");



	prev[0] = xoffset + (signalHeight * clip((1.0 + (-0.04)/*+ acc->getAccX()*/)));

	pt = paneX[0];
	backlight = 1;


	for (t = 21; t < paneX[1]; t++) {
		//0.04
		data[0] = xoffset + 15 + (signalHeight * clip((1.0 + (-0.04)/*acc->getAccX()*/)));
		//  data[1] = yoffset + (signalHeight * clip((1.0 + acc->getAccY()))) ;
		// data[2] = zoffset + (signalHeight * clip((1.0 + acc->getAccZ()))) ;
		TFT.line(pt, prev[0], t, data[0], Red);
		// TFT.line(pt, prev[1], t, data[1], Green) ;
		// TFT.line(pt, prev[2], t, data[2], Yellow) ;
		prev[0] = data[0];
		//  prev[1] = data[1] ;
		//  prev[2] = data[2] ;
		pt = t;
		wait(0.01);
	}




	TFT.BusEnable(false);

}


void screenUsers()
{
	backlight = 1;
	TFT.BusEnable(true);
	TFT.background(Black);
	TFT.foreground(White);
	TFT.cls();

	TFT.fillrect(10, 10, 229, 280, White);
	TFT.rect(10, 10, 229, 280, Red);
	TFT.rect(11, 11, 230, 281, Red);

	TFT.set_font((unsigned char*)Arial12x12);
	TFT.background(White);
	TFT.foreground(Black);
	TFT.locate(13, 15);
	TFT.printf("Currently registered users:");

	TFT.set_font((unsigned char*)Arial12x12);
	TFT.background(White);
	TFT.foreground(Black);
	TFT.locate(13, 40);
	TFT.printf("---Wajdi Bazuhair");



	TFT.locate(13, 60);
	TFT.printf("---Test user 1");


	TFT.locate(13, 80);
	TFT.printf("---Test user 2");


	TFT.locate(13, 100);
	TFT.printf("---Test user 3");


	TFT.foreground(Black);
	TFT.locate(13, 120);
	TFT.printf("---Test user 4");



	TFT.fillrect(10, 230, 229, 280, White);
	TFT.rect(10, 230, 229, 280, Red);
	TFT.rect(11, 231, 230, 281, Red);

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Black);
	TFT.locate(65, 244);
	TFT.printf("Cancel");


	TFT.BusEnable(false);
}

void screenFailedLogin()
{
	backlight = 1;
	TFT.BusEnable(true);
	TFT.background(Black);
	TFT.foreground(White);
	TFT.cls();

	TFT.fillrect(10, 10, 229, 280, White);
	TFT.rect(10, 10, 229, 280, Red);
	TFT.rect(11, 11, 230, 281, Red);

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Red);
	TFT.locate(30, 100);
	TFT.printf("User");

	TFT.locate(30, 130);
	TFT.printf("Not found...");






	TFT.fillrect(10, 230, 229, 280, White);
	TFT.rect(10, 230, 229, 280, Red);
	TFT.rect(11, 231, 230, 281, Red);

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Black);
	TFT.locate(65, 244);
	TFT.printf("Confirm");


	TFT.BusEnable(false);
}
void screenSuccessLogin(string username)
{
	backlight = 1;
	TFT.BusEnable(true);
	TFT.background(Black);
	TFT.foreground(White);
	TFT.cls();

	TFT.fillrect(10, 10, 229, 280, White);
	TFT.rect(10, 10, 229, 280, Red);
	TFT.rect(11, 11, 230, 281, Red);

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Green);
	TFT.locate(30, 100);
	TFT.printf("Authenticated ");

	TFT.locate(30, 130);
	TFT.printf(username.c_str());






	TFT.fillrect(10, 230, 229, 280, White);
	TFT.rect(10, 230, 229, 280, Red);
	TFT.rect(11, 231, 230, 281, Red);

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Black);
	TFT.locate(65, 244);
	TFT.printf("Confirm");


	TFT.BusEnable(false);
}

void screenSuccessRegistration()
{
	backlight = 1;
	TFT.BusEnable(true);
	TFT.background(Black);
	TFT.foreground(White);
	TFT.cls();

	TFT.fillrect(10, 10, 229, 280, White);
	TFT.rect(10, 10, 229, 280, Red);
	TFT.rect(11, 11, 230, 281, Red);

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Green);
	TFT.locate(30, 100);
	TFT.printf("Registration");

	TFT.locate(30, 130);
	TFT.printf("successful!");






	TFT.fillrect(10, 230, 229, 280, White);
	TFT.rect(10, 230, 229, 280, Red);
	TFT.rect(11, 231, 230, 281, Red);

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Black);
	TFT.locate(65, 244);
	TFT.printf("Confirm");


	TFT.BusEnable(false);
}


void screen3(uint8_t buffer[96], uint8_t buffer2[96], uint8_t buffer3[96],
	int size, int size2, int size3)
{
	int t = 0;
	int pt = 0; // previous t
	int i, x, y;
	unsigned int data[3]; // for x, y, z
	unsigned int prev[3];
	unsigned short signalHeight = 75;
	unsigned short xoffset = 30;
	unsigned short yoffset = 120;
	unsigned short zoffset = 210;
	unsigned short paneX[2] = { 20, 200 };
	unsigned short paneH = 150;

	backlight = 1;
	TFT.BusEnable(true);
	TFT.background(Black);
	TFT.foreground(White);
	//    TFT.cls() ;
	TFT.set_font((unsigned char*)Arial12x12);
	TFT.locate(90, 0);
	TFT.printf("ECG Capture");

	TFT.fillrect(paneX[0], xoffset, paneX[1], xoffset + paneH, Black);

	for (i = 0; i < 19; i++) {
		y = i * 8;
		TFT.line(paneX[0], xoffset + y, paneX[1], xoffset + y, Blue);

	}
	for (x = 30; x < paneX[1]; x += 10) {
		TFT.line(x, xoffset, x, xoffset + paneH, Blue);

	}
	TFT.rect(paneX[0], xoffset, paneX[1], xoffset + paneH, White);

	TFT.set_font((unsigned char*)Arial12x12);
	TFT.locate(5, xoffset + 30);
	TFT.printf("X");




	TFT.locate(10, 20);
	TFT.printf("A");
	TFT.locate(0, 115);
	TFT.printf("0.0");


	TFT.fillrect(10, 230, 229, 280, White);
	TFT.rect(10, 230, 229, 280, Red);
	TFT.rect(11, 231, 230, 281, Red);

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Black);
	TFT.locate(65, 244);
	TFT.printf("Cancel");


	prev[0] = xoffset + (signalHeight * clip((1.0 + (-0.04)/*acc->getAccX()*/)));

	pt = paneX[0];
	backlight = 1;


#ifdef SerialConsole
	printf("Data size = %d \r\n", size);
	printf("Data size2 = %d \r\n", size2);
#endif

	for (t = 21; t < size - 3 + 16; t += 2) {
		unsigned short dataPoint;
		dataPoint = (buffer[t - 16] << 8) | (buffer[t - 15] & 0xff);
		float FinalData = dataPoint;
		FinalData /= 100000.0f;
#ifdef SerialConsole
		printf("Data point = %f\r\n", FinalData);
#endif
		data[0] = xoffset + (signalHeight * clip((1.0 + FinalData/*acc->getAccX()*/)));
		TFT.line(pt, prev[0], t, data[0], Red);
		prev[0] = data[0];
		pt = t;
		wait(0.01);

	}

	int diff = pt - 5;
	for (t = pt; t < size2 - 3 + diff; t += 2) {
		unsigned short dataPoint;
		dataPoint = (buffer2[t - diff] << 8) | (buffer2[t - diff + 1] & 0xff);
		float FinalData = dataPoint;
		FinalData /= 100000.0f;
#ifdef SerialConsole
		printf("Data point = %f\r\n", FinalData);
#endif
		data[0] = xoffset + (signalHeight * clip((1.0 + FinalData)));
		TFT.line(pt, prev[0], t, data[0], Red);
		prev[0] = data[0];
		pt = t;
		wait(0.01);
	}

	diff = pt - 5;
	int limit = 0;
	if (size2 - 3 + diff > paneX[1])
		limit = paneX[1];
	else
		limit = size2 - 3 + diff;
	for (t = pt; t < limit; t += 2) {
		unsigned short dataPoint;
		dataPoint = (buffer3[t - diff] << 8) | (buffer3[t - diff + 1] & 0xff);
		float FinalData = dataPoint;
		FinalData /= 100000.0f;
#ifdef SerialConsole
		printf("Data point = %f\r\n", FinalData);
#endif
		data[0] = xoffset + (signalHeight * clip((1.0 + FinalData)));
		TFT.line(pt, prev[0], t, data[0], Red);
		prev[0] = data[0];
		pt = t;
		wait(0.01);
	}




	TFT.BusEnable(false);
}
int BPM = 0;




void screen4(void)
{
	int dx, px;
	float delta = 0.0;
	dx = 0;
	px = 0;
	backlight = 0;
	TFT.BusEnable(true);
	TFT.background(Black);
	wait(0.1);
	TFT.foreground(White);
	wait(0.1);
	TFT.cls();
	wait(0.1);

	TFT.set_font((unsigned char*)Arial12x12);
	TFT.foreground(Blue);
	TFT.locate(60, 10);
	TFT.printf("<< TSI demo >>");
	TFT.locate(30, 280);
	TFT.printf("Use FRDM touch slider");
	TFT.locate(30, 300);
	TFT.printf("Touch right edge to end");

	TFT.fillcircle(120, 160, 100, Green);
	TFT.fillcircle(60, 160, 50, Black);
	TFT.fillcircle(60, 160, 45, White);
	TFT.fillcircle(180, 160, 50, Black);
	TFT.fillcircle(180, 160, 45, White);
	TFT.fillcircle(60, 160, 5, Black);
	TFT.fillcircle(180, 160, 5, Black);
	backlight = 1;

	while (dx < 38) {
		delta = (80.0 * (tsi.readPercentage() - 0.5));
		dx = (int)(delta + 0.5);
		TFT.fillcircle(60 + px, 160, 5, White);
		TFT.fillcircle(180 + px, 160, 5, White);
		TFT.fillcircle(60 + dx, 160, 5, Black);
		TFT.fillcircle(180 + dx, 160, 5, Black);
		px = dx;
		wait(0.1);
	}
	TFT.fillcircle(60 + px, 160, 5, White);
	TFT.fillcircle(180 + px, 160, 5, White);
	TFT.line(15, 160, 105, 160, Black);
	TFT.line(135, 160, 225, 160, Black);
	TFT.foreground(Yellow);
	TFT.locate(30, 300);
	//    TFT.printf("Use FRDM touch slider") ;
	TFT.printf("       Wake Up!          ");
	TFT.locate(5, 300);
	TFT.printf("<< Prev");
	TFT.locate(180, 300);
	TFT.printf("Next >>");
	TFT.BusEnable(false);
}

void incPage(void)
{
	page++;
	if (page >= numPage) {
		page = 0;
	}
}

void decPage(void)
{
	page--;
	if (page < 0) {
		page = numPage - 1;
	}
}



// Copy tx line buffer to large tx buffer for tx interrupt routine
void send_line()
{
	int i;
	char temp_char;
	bool empty;
	i = 0;
	// Start Critical Section - don't interrupt while changing global buffer variables
	NVIC_DisableIRQ(UART1_IRQn);
	empty = (tx_in == tx_out);
	while ((i == 0) || (tx_line[i - 1] != '\n')) {
		// Wait if buffer full
		if (((tx_in + 1) % buffer_size) == tx_out) {
			// End Critical Section - need to let interrupt routine empty buffer by sending
			NVIC_EnableIRQ(UART1_IRQn);
			while (((tx_in + 1) % buffer_size) == tx_out) {
			}
			// Start Critical Section - don't interrupt while changing global buffer variables
			NVIC_DisableIRQ(UART1_IRQn);
		}
		tx_buffer[tx_in] = tx_line[i];
		i++;
		tx_in = (tx_in + 1) % buffer_size;
	}
	if (pc.writeable() && (empty)) {
		temp_char = tx_buffer[tx_out];
		tx_out = (tx_out + 1) % buffer_size;
		// Send first character to start tx interrupts, if stopped
		pc.putc(temp_char);
	}
	// End Critical Section
	NVIC_EnableIRQ(UART1_IRQn);
	return;
}


// Read a line from the large rx buffer from rx interrupt routine
void read_line()
{
	int i;
	i = 0;
	// Start Critical Section - don't interrupt while changing global buffer variables
	NVIC_DisableIRQ(UART1_IRQn);
	// Loop reading rx buffer characters until end of line character
	while ((i == 0) || (rx_line[i - 1] != '\r')) {
		// Wait if buffer empty
		if (rx_in == rx_out) {
			// End Critical Section - need to allow rx interrupt to get new characters for buffer
			NVIC_EnableIRQ(UART1_IRQn);
			while (rx_in == rx_out) {
			}
			// Start Critical Section - don't interrupt while changing global buffer variables
			NVIC_DisableIRQ(UART1_IRQn);
		}
		rx_line[i] = rx_buffer[rx_out];
		i++;
		rx_out = (rx_out + 1) % buffer_size;
	}
	// End Critical Section
	NVIC_EnableIRQ(UART1_IRQn);
	rx_line[i - 1] = 0;
	return;
}


// Interupt Routine to read in data from serial port
void Rx_interrupt()
{

	// Loop just in case more than one character is in UART's receive FIFO buffer
	// Stop if buffer full
	while ((pc.readable()) && (((rx_in + 1) % buffer_size) != rx_out)) {
		rx_buffer[rx_in] = pc.getc();
		// Uncomment to Echo to USB serial to watch data flow
		//        monitor_device.putc(rx_buffer[rx_in]);
		rx_in = (rx_in + 1) % buffer_size;
	}

	return;
}


// Interupt Routine to write out data to serial port
void Tx_interrupt()
{

	// Loop to fill more than one character in UART's transmit FIFO buffer
	// Stop if buffer empty
	while ((pc.writeable()) && (tx_in != tx_out)) {
		pc.putc(tx_buffer[tx_out]);
		tx_out = (tx_out + 1) % buffer_size;
	}

	return;
}

//attributes - program variables
char stringOverSerialBuffer[21];    // buffer to store received string over pc
int bytesRecieved = 0;

volatile bool newCommandFlag = false;             // flag for ISR - volatile so that the main loop picks up changes from the ISR.

void serialDataCallback()
{

	while (pc.readable()) { // read all available data
		if (/*(bytesRecieved  == 20) || */newCommandFlag)  // avoid buffer overflow
			pc.getc();
		else {
			stringOverSerialBuffer[bytesRecieved] = pc.getc();   // get waiting data
			bytesRecieved++;
			if (/*(bytesRecieved == 20) ||*/ (stringOverSerialBuffer[bytesRecieved - 1] == '\n')) {   // buffer full or a new line
				stringOverSerialBuffer[bytesRecieved] = 0;                                                              // append a null
				newCommandFlag = true;
			}
		}
	}
}

struct Olimexino328_packet
{
	uint8_t   sync0;      // = 0xa5
	uint8_t   sync1;      // = 0x5a
	uint8_t   version;    // = 2 (packet version)
	uint8_t   count;      // packet counter. Increases by 1 each packet.
	uint16_t  data[6];    // 10-bit sample (= 0 - 1023) in big endian (Motorola) format.
	uint8_t   switches;   // State of PD5 to PD2, in bits 3 to 0.
};
// All definitions
#define NUMCHANNELS 6
#define HEADERLEN 4
#define PACKETLEN (NUMCHANNELS * 2 + HEADERLEN + 1)
#define SAMPFREQ 256                      // ADC sampling rate 256
#define TIMER2VAL (1024/(SAMPFREQ))       // Set 256Hz sampling frequency     
// Global constants and variables
volatile unsigned char TXBuf[PACKETLEN];  //The transmission packet
volatile unsigned char TXIndex;           //Next byte to write in the transmission packet.
volatile unsigned char CurrentCh;         //Current channel being sampled.
volatile unsigned char counter = 0;   //Additional divider used to generate CAL_SIG
volatile  float ADC_Value = 0;      //ADC current value

DigitalInOut  LOPlus(PTD5);
void toggle_GAL_SIG(void) {
	LOPlus.input();
	if (LOPlus == 1) {
		wait_us(500);
		LOPlus.output();
		LOPlus = 0;//digitalWrite(CAL_SIG, LOW); 
	}
	else {
		wait_us(500);
		LOPlus.output();
		LOPlus = 1;//digitalWrite(CAL_SIG, HIGH);
	}

}

void SendPacketInterrupt()
{


	// pc.printf("interrupt called\r\n");
	 //Read the 6 ADC inputs and store current values in Packet
	for (CurrentCh = 0; CurrentCh < 6; CurrentCh++) {
		ADC_Value = ECGIN.read();//analogRead(CurrentCh);
		ADC_Value *= 1000;

		TXBuf[((2 * CurrentCh) + HEADERLEN)] = ((unsigned char)(((unsigned int)ADC_Value & 0xFF00) >> 8));  // Write High Byte
		TXBuf[((2 * CurrentCh) + HEADERLEN + 1)] = ((unsigned char)((unsigned int)ADC_Value & 0x00FF)); // Write Low Byte
	}

	// Send Packet
	for (TXIndex = 0; TXIndex < 17; TXIndex++) {
		pc.putc(TXBuf[TXIndex]);
	}

	// Increment the packet counter
	TXBuf[3]++;

	// Generate the CAL_SIGnal
	counter++;        // increment the devider counter
	if (counter == 12) {    // 250/12/2 = 10.4Hz ->Toggle frequency
		counter = 0;
		toggle_GAL_SIG();   // Generate CAL signal with frequ ~10Hz
	}

}


unsigned short signalHeight = 40;
float scaleMul = 1.0f;
void screenECG(string Title)
{
	int t = 0;
	int pt = 0; // previous t
	int i, x, y;
	unsigned int data[3]; // for x, y, z
	unsigned int prev[3];

	unsigned short xoffset = 20;
	unsigned short yoffset = 120;
	unsigned short zoffset = 210;
	unsigned short paneX[2] = { 5, 310 };
	unsigned short paneH = 110;

	backlight = 1;
	TFT.BusEnable(true);
	TFT.background(Black);
	TFT.foreground(White);
	//    TFT.cls() ;
	TFT.set_font((unsigned char*)Arial12x12);
	TFT.locate(120, 0);
	TFT.printf(Title.c_str());

	TFT.fillrect(paneX[0], xoffset, paneX[1], xoffset + paneH, Black);

	for (i = 0; i < 13; i++) {
		y = i * 8;
		TFT.line(paneX[0], xoffset + y, paneX[1], xoffset + y, Blue);

	}
	for (x = 10; x < paneX[1]; x += 10) {
		TFT.line(x, xoffset, x, xoffset + paneH, Blue);

	}
	TFT.rect(paneX[0], xoffset, paneX[1], xoffset + paneH, White);
	//
	  //  TFT.set_font((unsigned char*) Arial12x12);
		//TFT.locate(5, xoffset+30) ;
		//TFT.printf("X") ;




	TFT.locate(10, 20);
	TFT.printf("A");
	TFT.locate(0, 115);
	TFT.printf("0.0");

	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(Black);
	TFT.foreground(Red);

	TFT.locate(5, xoffset + paneH + 5);
	TFT.printf("Base");


	TFT.rect(15, xoffset + paneH + 35, 50, xoffset + paneH + 63, White);
	TFT.locate(25, xoffset + paneH + 40);
	TFT.printf("+");

	TFT.rect(15, xoffset + paneH + 73, 50, xoffset + paneH + 101, White);
	TFT.locate(25, xoffset + paneH + 78);
	TFT.printf("-");



	TFT.locate(80, xoffset + paneH + 5);
	TFT.printf("Scale");


	TFT.rect(95, xoffset + paneH + 35, 130, xoffset + paneH + 63, White);
	TFT.locate(105, xoffset + paneH + 40);
	TFT.printf("+");

	TFT.rect(95, xoffset + paneH + 73, 130, xoffset + paneH + 101, White);
	TFT.locate(105, xoffset + paneH + 78);
	TFT.printf("-");




	TFT.foreground(Green);
	TFT.locate(215, xoffset + paneH + 10);
	TFT.printf("BPM : ");

	TFT.locate(215, xoffset + paneH + 38);
	char buf[10];
	sprintf(buf, "%d", BPM);
	TFT.printf(buf);


	TFT.fillrect(150, xoffset + paneH + 76, 300, xoffset + paneH + 110, White);
	TFT.rect(150, xoffset + paneH + 76, 300, xoffset + paneH + 110, Red);


	TFT.set_font((unsigned char*)Arial24x23);
	TFT.background(White);
	TFT.foreground(Black);
	TFT.locate(160, xoffset + paneH + 80);
	TFT.printf("Cancel");

	float result = ADC_Value;
	result /= 1000.0f;
	result *= scaleMul;

	prev[0] = xoffset + (signalHeight * clip((1.0 + result/*acc->getAccX()*/)));

	pt = paneX[0];
	backlight = 1;


#ifdef SerialConsole
	printf("Data size = %d \r\n", size);
	printf("Data size2 = %d \r\n", size2);
#endif
	//int data2[300] ={143, 88, 159, 105, 159, 101, 159, 102, 170, 108, 54, 118, 177, 127, 67, 137, 85, 141, 87, 141, 87, 150, 87, 164, 105, 52, 115, 174, 120, 67, 118, 175, 125, 175, 123, 71, 141, 87, 150, 87, 164, 105, 52, 114, 173, 119, 67, 118, 175, 125, 175, 123, 71, 142, 87, 150, 88, 164, 105, 52, 114, 173, 119, 67, 118, 175, 125, 175, 123, 71, 142, 87, 150, 88, 164, 105, 52, 114, 173, 119, 67, 118, 175, 125, 175, 123, 71, 142, 87, 150, 88, 164, 104, 52, 114, 173, 120, 67, 117, 175, 125, 175, 121, 71, 142, 87, 150, 87, 164, 104, 52, 114, 173, 120, 67, 117, 175, 125, 175, 121, 71, 142, 87, 150, 87, 164, 104, 52, 114, 173, 120, 67, 117, 175, 125, 175, 121, 71, 142, 87, 150, 87, 164, 106, 54, 115, 174, 120, 68, 118, 176, 126, 176, 123, 70, 141, 86, 150, 87, 163};
	for (t = 21; t < paneX[1]; t += 2) {


		result = ADC_Value;
		result /= 1000.0f;
		result *= scaleMul;

		point touchPoint;
		pTouch.getTouch(touchPoint);
		// printf("touch %d %d \n\r", touchPoint.x, touchPoint.y);
		if (touchPoint.x <= 26000 && touchPoint.x >= 16000
			&& touchPoint.y <= 43000 && touchPoint.y >= 31000) {
			//increase the base line
			signalHeight -= 1;
		}
		else if (touchPoint.x <= 26000 && touchPoint.x >= 16000
			&& touchPoint.y <= 23000 && touchPoint.y >= 16000) {
			//decrease the base line

			signalHeight += 1;
		}

		else if (touchPoint.x <= 54000 && touchPoint.x >= 41000
			&& touchPoint.y <= 44000 && touchPoint.y >= 31000) {
			//increase scale

			scaleMul *= 1.5;
			if (scaleMul > 10)
				scaleMul = 10;
		}
		else if (touchPoint.x <= 54000 && touchPoint.x >= 41000
			&& touchPoint.y <= 21000 && touchPoint.y >= 16000) {
			//decrease scale

			scaleMul /= 1.5;
			if (scaleMul < 1)
				scaleMul = 1;
		}

		else if (touchPoint.x <= 110000 && touchPoint.x >= 59000
			&& touchPoint.y <= 25000 && touchPoint.y >= 16000) {
			//cancel and go to main menu
			break;
		}



		// printf("value = %f\r\n", result);

		data[0] = xoffset + (signalHeight * clip(1.0 + result));/*(float)data2[t]/200)*/
		TFT.line(pt, prev[0], t, data[0], Red);
		prev[0] = data[0];
		pt = t;
		wait_ms(30);


	}






	TFT.BusEnable(false);
}



int main()
{
	pc.baud(57600);

	//Write packet header and footer
	TXBuf[0] = 0xa5;    //Sync 0
	TXBuf[1] = 0x5a;    //Sync 1
	TXBuf[2] = 2;       //Protocol version
	TXBuf[3] = 0;       //Packet counter
	TXBuf[4] = 0x02;    //CH1 High Byte
	TXBuf[5] = 0x00;    //CH1 Low Byte
	TXBuf[6] = 0x02;    //CH2 High Byte
	TXBuf[7] = 0x00;    //CH2 Low Byte
	TXBuf[8] = 0x02;    //CH3 High Byte
	TXBuf[9] = 0x00;    //CH3 Low Byte
	TXBuf[10] = 0x02;   //CH4 High Byte
	TXBuf[11] = 0x00;   //CH4 Low Byte
	TXBuf[12] = 0x02;   //CH5 High Byte
	TXBuf[13] = 0x00;   //CH5 Low Byte
	TXBuf[14] = 0x02;   //CH6 High Byte
	TXBuf[15] = 0x00;   //CH6 Low Byte 
	TXBuf[2 * NUMCHANNELS + HEADERLEN] = 0x01;    // Switches state





#ifdef SerialConsole
	pc.printf("System Initialized\r\n");
#endif
	uint16_t x, y, z;
	// int prevPage = 99 ;
	bool waitTouch = false;
	PollECG = false;

	//  SendPacketTimer.attach_us(&SendPacketInterrupt,10000);



	pc.attach(&serialDataCallback);  // attach pc ISR
	// Setup a serial interrupt function to receive data
	//pc.attach(&Rx_interrupt, Serial::RxIrq);
// Setup a serial interrupt function to transmit data
	//pc.attach(&Tx_interrupt, Serial::TxIrq);

	//acc = new MMA8451Q(PTE25, PTE24, MMA8451_I2C_ADDRESS) ;



	initTFT();


	TFT.BusEnable(true);
	TFT.background(Black);
	TFT.foreground(White);
	TFT.cls();
	TFT.BusEnable(false);

	//   screen0() ;

	  //  TestApp_Init();
	page = 0;
	point touchPoint;
	char tmp[256];

	string data = "";
	string title = "ECG registration";



	for (;;) {



		//  if(LOPlus || LOMinus)
		// if (!LOPlus || !LOMinus)
		 // pc.printf("%d \r\n", (int)(ECGIN.read()*1000));
		wait_ms(2);
		// pTouch.getTouch(touchPoint);
		//  printf("touch %d %d \n\r", touchPoint.x, touchPoint.y);
		switch (page) {
		case 0:
			if (prevPage != page) {
#ifdef SerialConsole
				printf("Clearing Display - Home Screen\r\n");
#endif
				prevPage = page;
				screen1();
				SendPacketTimer.detach();

			}
			pTouch.getTouch(touchPoint);
			if (touchPoint.x <= 100000 && touchPoint.x >= 25000
				&& touchPoint.y <= 92000 && touchPoint.y >= 72000) {
				page = 1;
				title = "ECG Registration";
				//send ecg started packet
				TXBuf[1] = 0x4a;
				//start registration
			   //  printf("$1111; \r\n");
			}
			else if (touchPoint.x <= 100000 && touchPoint.x >= 25000
				&& touchPoint.y <= 63000 && touchPoint.y >= 47000) {
				//Login 
				page = 1;
				title = "ECG Login";
				TXBuf[1] = 0x3a;
			}
			//list users button pressed
			else if (touchPoint.x <= 100000 && touchPoint.x >= 25000
				&& touchPoint.y <= 40000 && touchPoint.y >= 21000) {
				page = 2;
			}
			break;
		case 1:
			if (prevPage != page) {
#ifdef SerialConsole
				printf("Clearing Display - Registration mode\r\n");
#endif
				TFT.BusEnable(true);
				TFT.background(Black);
				TFT.foreground(White);
				TFT.cls();
				TFT.BusEnable(false);
				prevPage = page;
				SendPacketTimer.attach(&SendPacketInterrupt, 0.008);
			}

			//receive BPM

			if (newCommandFlag) {  // if command has been read
				pc.printf("received: %s \n\r", stringOverSerialBuffer);
				data = stringOverSerialBuffer;
				bytesRecieved = 0;  // clear the message size pointer so the next message is at the start of the buffer
				newCommandFlag = false;   // clear the flag.
			}



			if (data.find("BPM") != std::string::npos) {
				int start_position_to_erase = data.find("BPM");
				data.erase(start_position_to_erase, 3);

				BPM = atoi(data.c_str());
			}
			else if (data.find("DoneRegistering") != std::string::npos) {
				//received successfull registration message
				page = 3;
			}
			else if (data.find("LoggedInAs") != std::string::npos) {
				//received no match message :(
				//received logged in as message
				page = 4;
			}

			else if (data.find("NoMatch") != std::string::npos) {
				page = 5;
			}

			pTouch.getTouch(touchPoint);
			//if the cancel button is pressed, return to the home screen
			if (touchPoint.x <= 110000 && touchPoint.x >= 59000
				&& touchPoint.y <= 25000 && touchPoint.y >= 16000)
			{
				//send Stop command
				printf("$0000; \r\n");
				page = 0;
			}
			screenECG(title);
			break;
		case 2:
			if (prevPage != page) {
#ifdef SerialConsole
				printf("Clearing Display - Users screen\r\n");
#endif
				prevPage = page;
				screenUsers();
			}
			pTouch.getTouch(touchPoint);
			//if the cancel button is pressed, return to the home screen
			if (touchPoint.x <= 100000 && touchPoint.x >= 91000
				&& touchPoint.y <= 100000 && touchPoint.y >= 20000) {
				page = 0;
			}
			break;
		case 3:
			if (prevPage != page) {
				prevPage = page;
				screenSuccessRegistration();
			}
			pTouch.getTouch(touchPoint);
			//if the cancel button is pressed, return to the home screen
			if (touchPoint.x <= 100000 && touchPoint.x >= 91000
				&& touchPoint.y <= 100000 && touchPoint.y >= 20000) {
				page = 0;
			}
			break;


		case 4:
			if (prevPage != page) {
				prevPage = page;
				screenSuccessLogin("Wajdi");
			}
			pTouch.getTouch(touchPoint);
			//if the cancel button is pressed, return to the home screen
			if (touchPoint.x <= 100000 && touchPoint.x >= 91000
				&& touchPoint.y <= 100000 && touchPoint.y >= 20000) {
				page = 0;
			}
			break;
		case 5:
			if (prevPage != page) {
				prevPage = page;
				screenFailedLogin();
			}
			pTouch.getTouch(touchPoint);
			//if the cancel button is pressed, return to the home screen
			if (touchPoint.x <= 100000 && touchPoint.x >= 91000
				&& touchPoint.y <= 100000 && touchPoint.y >= 20000) {
				page = 0;
			}
			break;



		default:
			page = 0;
			break;
		}


	}

}
