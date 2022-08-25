/* SC16IS750 interface
 *   /////////////////////v1.0 Tedd OKANO, 18 Jul 2012, I2C I/F only, MIT License
 *   v1.1 WH, Nov 2013, Added SPI I/F and more methods, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef _SC16IS750_H
#define _SC16IS750_H

// Default I2C Slaveaddress
#define DEFAULT_SC16IS750_ADDR 0x9A

// Default baudrate
#define DEFAULT_BAUD_RATE 9600

#define ENABLE_BULK_TRANSFERS 0x01

//#include "Configuration.h"

#define XTAL_FREQUENCY 14745600UL // On-board crystal (New mid-2010 Version)

// See datasheet section 7.8 for configuring the
// "Programmable baud rate generator"
#define PRESCALER 1 // Default prescaler after reset
#define BAUD_RATE_DIVISOR(baud) ((XTAL_FREQUENCY / PRESCALER) / (baud * 16UL))

// See section 8.4 of the datasheet for definitions
// of bits in the Line Control Register (LCR)
#define LCR_BITS5 0x00
#define LCR_BITS6 0x01
#define LCR_BITS7 0x02
#define LCR_BITS8 0x03

#define LCR_BITS1 0x00
#define LCR_BITS2 0x04

#define LCR_NONE 0x00
#define LCR_ODD 0x08
#define LCR_EVEN 0x18
#define LCR_FORCED1 0x28
#define LCR_FORCED0 0x38

#define LCR_BRK_ENA 0x40
#define LCR_BRK_DIS 0x00

#define LCR_DIV_ENA 0x80
#define LCR_DIV_DIS 0x00

// See section 8.10 of the datasheet for definitions
// of bits in the Enhanced Features Register (EFR)
#define EFR_ENABLE_CTS (1 << 7)
#define EFR_ENABLE_RTS (1 << 6)
#define EFR_ENABLE_ENHANCED_FUNCTIONS (1 << 4)

// See Chapter 11 of datasheet
#define SPI_READ_MODE_FLAG 0x80

/** Abstract class SC16IS750 for a converter between either SPI or I2C and a Serial port
 *
 * Supports both SPI and I2C interfaces through derived classes
 *
 * @code
 *
 * @endcode
 */
// class SC16IS750 : public Serial { //Fout, geen Serial constr met Serial(NC, NC) toegestaan...
class SC16IS750
{

public:
  //  SC16IS750 Register definitions (shifted to align)
  enum RegisterName
  {
    RHR = 0x00 << 3,
    THR = 0x00 << 3,
    IER = 0x01 << 3,
    FCR = 0x02 << 3,
    IIR = 0x02 << 3,
    LCR = 0x03 << 3,
    MCR = 0x04 << 3,
    LSR = 0x05 << 3,
    MSR = 0x06 << 3,
    SPR = 0x07 << 3,
    TCR = 0x06 << 3,
    TLR = 0x07 << 3,
    TXLVL = 0x08 << 3,
    RXLVL = 0x09 << 3,
    IODIR = 0x0A << 3,
    IOSTATE = 0x0B << 3,
    IOINTMSK = 0x0C << 3,
    reserved = 0x0D << 3,
    IOCTRL = 0x0E << 3,
    EFCR = 0x0F << 3,
    DLL = 0x00 << 3,
    DLH = 0x01 << 3,
    EFR = 0x02 << 3,
    XON1 = 0x04 << 3,
    XON2 = 0x05 << 3,
    XOFF1 = 0x06 << 3,
    XOFF2 = 0x07 << 3,
  };

  // SC16IS750 configuration register values
  struct SC16IS750_cfg
  {
    char baudrate;
    char dataformat;
    char flowctrl;
    char fifoformat;
  };

  /** Constructor
   *
   */
  SC16IS750();

  /** Determine if there is a character available to read.
   *   @return 1 if there is a character available to read, 0 otherwise
   */
  int readable();

  /** Determine if how many characters available to read.
   *   @return int Characters available to read
   */
  int readableCount();

  /** Determine if there is space available to write a character.
   *   @return 1 if there is a space for a character to write, 0 otherwise
   */
  int writable();

  /** Determine if how much space is available to write characters.
   *   @return int Characterspace available for writing
   */
  int writableCount();

  char getc();
  void putc(char value);

  void write(const char *str);

  /** Set baudrate of the serial port.
   *  @param  baud integer baudrate (4800, 9600 etc)
   *  @return none
   */
  void baud(int baudrate = DEFAULT_BAUD_RATE);

  /** Set the transmission format used by the serial port.
   *   @param bits      The number of bits in a word (5-8; default = 8)
   *   @param parity    The parity used (Serial::None, Serial::Odd, Serial::Even, Serial::Forced1, Serial::Forced0; default = Serial::None)
   *   @param stop_bits The number of stop bits (1 or 2; default = 1)
   */
  void format(int bits = 8, Serial::Parity parity = Serial::None, int stop_bits = 1);

  /**
   * Check that UART is connected and operational.
   *   @param  none
   *   @return bool true when connected, false otherwise
   */
  bool connected();

#if ENABLE_BULK_TRANSFERS
  void write(const uint8_t *buffer, size_t size);
#else
  //   using Print::write;
#endif
  void flush();

  // required for Stream
  int peek() { return 0; };

  // These are specific to the SPI UART
  void ioSetDirection(unsigned char bits);
  void ioSetState(unsigned char bits);

  /** Write value to internal register.
   * Pure virtual, must be declared in derived class.
   *   @param register_address  The address of the Register (enum RegisterName)
   *   @param data              The 8bit value to write
   *   @return none
   */
  virtual void writeRegister(RegisterName register_address, char data) = 0;

  /** Read value from internal register.
   * Pure virtual, must be declared in derived class.
   *   @param register_address  The address of the Register (enum RegisterName)
   *   @return char             The 8bit value read from the register
   */
  virtual char readRegister(RegisterName register_address) = 0;

protected:
  // protected is accessible to derived classes, but not to external users

  SC16IS750_cfg _config;

private:
  // private is not accessible to derived classes, nor external users
  void init();
};

/** Class SC16IS750_SPI for a converter between SPI and a Serial port
 *
 * @code
 * #include "mbed.h"
 * #include "SC16IS750.h"
 *
 * SPI spi(PTD2, PTD3, PTD1); //MOSI, MISO, SCK
 * SC16IS750_SPI serial_spi(&spi, PTD0);
 *
 * Serial pc(USBTX,USBRX);
 *
 * int main() {
 *   pc.printf("\nHello World!\n");
 *
 *   while(1) {
 *     serial_spi.ioSetState(0x00);
 *     wait(0.5);
 *     pc.putc('*');
 *   }
 * }
 *
 * @endcode
 */
class SC16IS750_SPI : public SC16IS750
{
public:
  /** Create a SC16IS750_SPI object using a specified SPI bus and CS
   *
   * @param SPI &spi the SPI port to connect to
   * @param cs  the Pin of the CS
   */
  SC16IS750_SPI(SPI *spi, PinName cs);

  /** Write value to internal register.
   *   @param register_address  The address of the Register (enum RegisterName)
   *   @param data              The 8bit value to write
   *   @return none
   */
  virtual void writeRegister(SC16IS750::RegisterName registerAddress, char data);

  /** Read value from internal register.
   *   @param register_address  The address of the Register (enum RegisterName)
   *   @return char             The 8bit value read from the register
   */
  virtual char readRegister(SC16IS750::RegisterName registerAddress);

protected:
  // protected is accessible to derived classes, but not to external users

private:
  SPI *_spi;      // SPI bus reference
  DigitalOut _cs; // CS of SPI device
};

/** Class SC16IS750_I2C for a converter between I2C and a Serial port
 *
 * @code
 * #include "mbed.h"
 * #include "SC16IS750.h"
 *
 * I2C i2c(PTE0, PTE1);       //SDA, SCL
 * SC16IS750_I2C serial_i2c(&i2c, DEFAULT_SC16IS750_ADDR);
 *
 * Serial pc(USBTX,USBRX);
 *
 * int main() {
 *   pc.printf("\nHello World!\n");
 *
 *   while(1) {
 *     serial_i2c.ioSetState(0x00);
 *     wait(0.5);
 *     pc.putc('*');
 *   }
 * }
 *
 * @endcode
 */
class SC16IS750_I2C : public SC16IS750
{
public:
  /** Create a SC16IS750_I2C object using a specified I2C bus and slaveaddress
   *
   * @param I2C &i2c the I2C port to connect to
   * @param char deviceAddress the address of the SC16IS750
   */
  SC16IS750_I2C(I2C *i2c, uint8_t deviceAddress = DEFAULT_SC16IS750_ADDR);

  /** Write value to internal register.
   *   @param register_address  The address of the Register (enum RegisterName)
   *   @param data              The 8bit value to write
   *   @return none
   */
  virtual void writeRegister(SC16IS750::RegisterName register_address, char data);

  /** Read value from internal register.
   *   @param register_address  The address of the Register (enum RegisterName)
   *   @return char             The 8bit value read from the register
   */
  virtual char readRegister(SC16IS750::RegisterName register_address);

protected:
  // protected is accessible to derived classes, but not to external users

private:
  I2C *_i2c;             // I2C bus reference
  uint8_t _slaveAddress; // I2C Slave address of device
};

#endif //  _SC16IS750_H
