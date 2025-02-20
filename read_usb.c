// C library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <signal.h>

// Defining UBX message specific codes
#define UBX_PREAMBLE_1 0xb5
#define UBX_PREAMBLE_2 0x62
#define UBX_CLASS_ESF 0x10
#define UBX_TYPE_ESF_RAW 0x03

// Defining the data_type codes
#define ACC_X 16
#define ACC_Y 17
#define ACC_Z 18
#define GYR_X 14
#define GYR_Y 13
#define GYR_Z 5
#define GYR_T 12

// Defining the accelerometer and gyroscope digitizing parameters
#define LSB2ACC 1000.0
#define LSB2GYR 4096.0

// The log file to save the recorded data
FILE *logFile;

// Configure values if needed
char portName[] = "/dev/ttyACM0";
int baudRate = 38400;

// Close log file in case of being interrupted by user
void handleSigint(int sig)
{
  if (logFile)
  {
    printf("\nClosing file before exiting...\n");
    fclose(logFile);
  }
  exit(0);
}

/// @brief Check if UBX message has a valid checksum
/// @param msg
/// @return true or false
bool checksumUBX(char *msg, int numBytes)
{
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  for (int i = 2; i < numBytes - 2; i++)
  {
    ck_a = ck_a + msg[i];
    ck_b = ck_b + ck_a;
  }
  if (ck_a == read_buf[numBytes - 2] && ck_b == read_buf[numBytes - 1])
    return true;
  else
    return false;
}

/// @brief Handles NMEA messages, probably will stay empty cuz I don't need it.
/// @param msg - a pointer to the message char array.
/// @return
int parseNMEA(char *msg)
{
  char nmeaClass[3];
  strncpy(nmeaClass, read_buf + 3, 3);
  if (strcmp(nmeaClass, "GGA") == 0)
  {
  }
}

/// @brief Handles UBX messages, specifically UBX-ESF-RAW (0x10 0x03), might be extended to handle other types
/// @param msg - a pointer to the message char array.
/// @param numBytes - total number of bytes in the message
/// @return 0 if message is parsed and logged correctly, 1 if the checksum is wrong, 2 if the message is not UBX-ESF-RAW
int parseUBX(char *msg, int numBytes)
{
  uint8_t msg_class = read_buf[2];
  uint8_t msg_type = read_buf[3];
  uint16_t length = (uint16_t)read_buf[4] | ((uint16_t)read_buf[5] << 8);

  // Uncomment the line below to print the number of bytes, message class, type and length
  // printf("Read numBytes %d, class 0x%x, type 0x%x, length %d ", numBytes, msg_class, msg_type, length);

  if (msg_class == UBX_CLASS_ESF && msg_type == UBX_TYPE_ESF_RAW)
  {
    double acc_x = 0;
    double acc_y = 0;
    double acc_z = 0;
    double gyr_x = 0;
    double gyr_y = 0;
    double gyr_z = 0;
    uint32_t time_tag = 0;

    if
      ~(checksumUBX(msg, numBytes))
      {
        return 1;
      }

    for (int i = 6 + 4; i < numBytes - 2; i += 8)
    {
      // Parse data
      int32_t data = (int32_t)read_buf[i] | ((int32_t)read_buf[i + 1] << 8) | ((int32_t)read_buf[i + 2] << 16);
      if (read_buf[i + 2] & 0x80)
        data = data | 0xFF000000; // Add leading ones in case of a negative number
      double data_double = data;

      // Parse data type
      uint8_t data_type = (uint8_t)read_buf[i + 3];

      // Time tag is a counter that gets updated every 39 usec.
      time_tag = (uint32_t)read_buf[i + 4] | ((uint32_t)read_buf[i + 5] << 8) | ((uint32_t)read_buf[i + 6] << 16) | ((uint32_t)read_buf[i + 7] << 24);
      time_tag = time_tag * 39.0 / 1000; // In msec
      switch (data_type)
      {
      case ACC_X:
        acc_x = data_double / LSB2ACC;
        break;
      case ACC_Y:
        acc_y = data_double / LSB2ACC;
        break;
      case ACC_Z:
        acc_z = data_double / LSB2ACC;
        break;
      case GYR_X:
        gyr_x = data_double / LSB2GYR;
        break;
      case GYR_Y:
        gyr_y = data_double / LSB2GYR;
        break;
      case GYR_Z:
        gyr_z = data_double / LSB2GYR;
        break;
      }
    }
    struct tm *utc_time = gmtime(&ts.tv_sec);
    // Uncomment if wanna print
    // printf("UTC Time: %04d-%02d-%02d %02d:%02d:%02d.%ld ",
    //      utc_time->tm_year + 1900, utc_time->tm_mon + 1, utc_time->tm_mday,
    //     utc_time->tm_hour, utc_time->tm_min, utc_time->tm_sec, msec);
    // printf("Sensor time: %d, \tacc: [%.2f, %.2f, %.2f], \tgyr: [%.2f, %.2f, %.2f], \t%s\n", time_tag, acc_x, acc_y, acc_z, gyr_x / 4.096, gyr_y / 4.096, gyr_z / 4.096, checksum_str);
    fprintf(logFile, "%04d,%02d,%02d,%02d,%02d,%02d.%ld\t%.3f,%.3f,%.3f\t%.3f,%.3f,%.3f\n", utc_time->tm_year + 1900, utc_time->tm_mon + 1, utc_time->tm_mday,
            utc_time->tm_hour, utc_time->tm_min, utc_time->tm_sec, msec, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z);
    fflush(logFile);

    return 0;
  }

  return 2;
}

int main()
{
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serialPort = open(portName, O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;
  struct timespec ts;

  // Introduce signal handler
  signal(SIGINT, handleSigint);

  // Log file is rewritten every time
  logFile = fopen("imu_log.txt", "w");

  if (logFile == NULL)
  {
    perror("Error opening file");
    return 1; // Return error if file couldn't be opened
  }

  // Read in existing settings, and handle any error
  if (tcgetattr(serialPort, &tty) != 0)
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    return 1;
  }

  tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
  tty.c_cflag |= CS8;            // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;                                                        // Disable echo
  tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
  tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
  tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, baudRate);
  cfsetospeed(&tty, baudRate);

  // Save tty settings, also checking for error
  if (tcsetattr(serialPort, TCSANOW, &tty) != 0)
  {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    return 1;
  }

  // Reading loop
  while (1)
  {
    // Allocate memory for read buffer, set size according to your needs
    char read_buf[1024];

    // Normally you wouldn't do this memset() call, but since we will just receive
    // ASCII data for this example, we'll set everything to 0 so we can
    // call printf() easily.
    memset(&read_buf, '\0', sizeof(read_buf));

    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    int numBytes = read(serialPort, &read_buf, sizeof(read_buf));
    clock_gettime(CLOCK_REALTIME, &ts);
    long msec = ts.tv_nsec / 1000000;

    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (numBytes < 0)
    {
      printf("Error reading: %s", strerror(errno));
      return 1;
    }

    if (read_buf[0] == '$') // NMEA messages start with a $ sign
    {
      parseNMEA(read_buf);
    }
    else if (read_buf[0] == UBX_PREAMBLE_1 && read_buf[1] == UBX_PREAMBLE_2) // UBX messages start with these two bytes
    {
      parseUBX(read_buf);
    }
  }
  close(serialPort);
  return 0; // success
};
