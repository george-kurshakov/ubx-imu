// C library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <signal.h>

#define ACC_X 16
#define ACC_Y 17
#define ACC_Z 18
#define GYR_X 14
#define GYR_Y 13
#define GYR_Z 5
#define GYR_T 12

FILE *log_file;

void handle_sigint(int sig) {
  if (log_file) {
    printf("\nClosing file before exiting\n");
    fclose(log_file);
  }
  exit(0);
}

int main() {
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open("/dev/ttyACM0", O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;
  struct timespec ts;

  signal(SIGINT, handle_sigint);
  log_file = fopen("imu_log.txt", "w");

  if (log_file == NULL) {
        perror("Error opening file");
        return 1;  // Return error if file couldn't be opened
  }

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, 38400);
  cfsetospeed(&tty, 38400);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  // Write to serial port
  unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
  write(serial_port, msg, sizeof(msg));

  while(1){
  // Allocate memory for read buffer, set size according to your needs
  char read_buf [1024];

  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  memset(&read_buf, '\0', sizeof(read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
  time_t read_time = time(NULL);
  clock_gettime(CLOCK_REALTIME, &ts);
  long msec = ts.tv_nsec / 1000000;

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
      return 1;
  }

  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
  // print it to the screen like this!)
  bool msg_ok = false;
  char msg_class;
  char msg_type;
  char nmea_class[3];
  if (read_buf[0] == '$') {
    strncpy(nmea_class, read_buf + 3, 3);
    //printf("%s\n", nmea_class);
    if (strcmp(nmea_class, "GGA") == 0) {
      
      //printf("Sec: %ld, msec: %ld, %s", ts.tv_sec, msec, asctime(gmtime(&read_time)));
      //printf("%s",read_buf); // Ignore message if it is NMEA
    }
  }
  else if (read_buf[0] == 0xb5 && read_buf[1] == 0x62) msg_ok = true;
  msg_class = read_buf[2];
  msg_type = read_buf[3];
  if (msg_ok) {
    switch (msg_class) {
      case 0x10:
        switch (msg_type) {
          case 0x02:
            break;
          case 0x03:
            break;
          //default:
            //printf("Read %i bytes. Class %x, type %x\n ", num_bytes, msg_class, msg_type);
        }
        break;
      //default:
        //printf("Read %i bytes. Class %x, type %x\n ", num_bytes, msg_class, msg_type);
    }
    //printf("Read %i bytes. Class %x, type %x\n ", num_bytes, msg_class, msg_type);

    uint16_t length = (uint16_t)read_buf[4] | ((uint16_t)read_buf[5] << 8);

    if (msg_class == 0x10 && msg_type == 0x03){
      
      //printf("Num_bytes %d, class 0x%x, type 0x%x, length %d ", num_bytes, msg_class, msg_type, length);
      double acc_x = 0;
      double acc_y = 0;
      double acc_z = 0;
      double gyr_x = 0;
      double gyr_y = 0;
      double gyr_z = 0;
      uint32_t time_tag = 0;
      uint8_t ck_a = 0;
      uint8_t ck_b = 0;
      bool checksum_ok = false;
      char checksum_str[20];
      for (int i = 2; i < num_bytes-2; i++) {
        ck_a = ck_a + read_buf[i];
        ck_b = ck_b + ck_a;
      }
      if (ck_a == read_buf[num_bytes-2] && ck_b == read_buf[num_bytes-1]) {
        strcpy(checksum_str,"Checksum ok");
        checksum_ok = true;
      }
      else {
        strcpy(checksum_str,"Checksum wrong");
        checksum_ok = false;
      }
      if (checksum_ok) {

      for (int i = 6 + 4; i < num_bytes-2; i+=8) {
        int32_t data = (int32_t)read_buf[i] | ((int32_t)read_buf[i+1] << 8) | ((int32_t)read_buf[i+2] << 16);
        if (read_buf[i+2] & 0x80) data = data | 0xFF000000; // Add leading ones in case of a negative number
        double data_double = data / 1000.0;
        uint8_t data_type = (uint8_t)read_buf[i+3];
        // Time tag is a counter that gets updated every 39 usec.
        time_tag = (uint32_t)read_buf[i+4] | ((uint32_t)read_buf[i+5] << 8) | ((uint32_t)read_buf[i+6] << 16) | ((uint32_t)read_buf[i+7] << 24);
        time_tag = time_tag * 39.0 / 1000; //In msec
        switch (data_type) {
          case ACC_X:
            acc_x = data_double;
            break;
          case ACC_Y:
            acc_y = data_double;
            break;
          case ACC_Z:
            acc_z = data_double;
            break;
          case GYR_X:
            gyr_x = data_double;
            break;
          case GYR_Y:
            gyr_y = data_double;
            break;
          case GYR_Z:
            gyr_z = data_double;
            break;
        }
        //printf("%x", read_buf[i]);
        //printf("data: %d, data type: %d, time tag: %d\n", data, data_type, time_tag);
        //printf(" ");
      }
      struct tm *utc_time = gmtime(&ts.tv_sec);
      //printf("UTC Time: %04d-%02d-%02d %02d:%02d:%02d.%ld ",
      //     utc_time->tm_year + 1900, utc_time->tm_mon + 1, utc_time->tm_mday,
      //    utc_time->tm_hour, utc_time->tm_min, utc_time->tm_sec, msec);
      //printf("Sensor time: %d, \tacc: [%.2f, %.2f, %.2f], \tgyr: [%.2f, %.2f, %.2f], \t%s\n", time_tag, acc_x, acc_y, acc_z, gyr_x / 4.096, gyr_y / 4.096, gyr_z / 4.096, checksum_str);
      fprintf(log_file, "%04d,%02d,%02d,%02d,%02d,%02d.%ld\t%.3f,%.3f,%.3f\t%.3f,%.3f,%.3f\n", utc_time->tm_year + 1900, utc_time->tm_mon + 1, utc_time->tm_mday,
           utc_time->tm_hour, utc_time->tm_min, utc_time->tm_sec, msec, acc_x, acc_y, acc_z, gyr_x / 4.096, gyr_y / 4.096, gyr_z / 4.096);
      fflush(log_file);
    }
    }
    else if (msg_class == 0x10 && msg_type == 0x02){
      uint32_t timeTag = (uint32_t)read_buf[6] | ((uint32_t)read_buf[7] << 8) | ((uint32_t)read_buf[8] << 16) | ((uint32_t)read_buf[9] << 24);
      //printf("Num_bytes %d, class 0x%x, type 0x%x, length %d, time tag %d ", num_bytes, msg_class, msg_type, length, timeTag);
      
      for (int i = 10; i < num_bytes-2; i++) {
        //uint32_t data = (uint32_t)read_buf[i] | ((uint32_t)read_buf[i+1] << 8) | ((uint32_t)read_buf[i+2] << 16);
        //uint8_t data_type = (uint32_t)read_buf[i+3];
        //uint32_t time_tag = (uint32_t)read_buf[i+4] | ((uint32_t)read_buf[i+5] << 8) | ((uint32_t)read_buf[i+6] << 16) | ((uint32_t)read_buf[i+7] << 24);
        //printf("%x", read_buf[i]);
        //printf("data: %d, data type: %d, time tag: %d\n", data, data_type, time_tag);
        //printf(" ");
      }
    //printf("\n");
  }}
  }
  close(serial_port);
  return 0; // success
};
