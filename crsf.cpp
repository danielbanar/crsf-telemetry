#include <stdio.h>
#include <string.h>
#include <iostream>
#include <string>
#include <vector>
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

void ProcessPayload(std::vector<uint8_t> payload)
{
  printf("Raw: ");
  for (uint8_t byte : payload)
  {
    printf("%02x ", static_cast<unsigned char>(byte));
  }
  printf("\n");
  if (payload[0] == 0x08) // CRSF_FRAMETYPE_BATTERY_SENSOR
  {
    uint16_t voltage = (payload[1] << 8) | payload[2];
    uint16_t current = (payload[3] << 8) | payload[4];
    uint32_t capacity = (payload[5] << 16) | (payload[6] << 8) | payload[7];
    uint8_t remaining = payload[8];
    printf("BATTERY_SENSOR: %.1fV\t%.1fA\t%dmAh\t%d%%\n", ((float)voltage) / 10, ((float)current) / 10, capacity, remaining);
  }
  else if (payload[0] == 0x02) // CRSF_FRAMETYPE_GPS
  {
    int32_t latitude = (payload[1] << 24) | (payload[2] << 16) | (payload[3] << 8) | payload[4];     // degree / 10,000,000 big endian
    int32_t longitude = (payload[5] << 24) | (payload[6] << 16) | (payload[7] << 8) | payload[8];;    // degree / 10,000,000 big endian
    uint16_t groundspeed = (payload[9] << 8) | payload[10]; // km/h / 10 big endian
    uint16_t heading = (payload[11] << 8) | payload[12];     // GPS heading, degree/100 big endian
    uint16_t altitude = (payload[13] << 8) | payload[14];    // meters, +1000m big endian
    uint8_t satellites = payload[15];   // satellites
    printf("GPS: Lat: %d\tLon: %d\tGspd: %d\tHdg: %d\tAlt: %d\tSat: %d\n", latitude,longitude,groundspeed,heading,altitude,satellites);
  }
}
void CheckPayloads(std::vector<uint8_t> &buffer)
{
  while (true)
  {
    // Start Byte
    size_t start = -1;
    // Scan for Start byte
    for (int i = 0; i < buffer.size(); i++)
    {
      if (buffer[i] == 0xC8)
      {
        start = i;
        break;
      }
    }
    // if Start byte not found return and read from serial
    if (start == -1)
      return;

    // Trim to start byte
    buffer.erase(buffer.begin(), buffer.begin() + start);

    // Check for payload size - aka anti out of bounds - aka sync byte is last byte in buffer
    if (buffer.size() < 2)
      return;
    size_t payload_length = buffer[1];
    // Check if entire payload is in buffer / aka anti out of bounds
    if (buffer.size() < payload_length + 2)
      return;
    // YES we have entire payload in buffer
    // Process it and remove from buffer
    ProcessPayload((std::vector<uint8_t>(buffer.begin() + 2, buffer.begin() + payload_length + 2)));
    buffer.erase(buffer.begin(), buffer.begin() + payload_length + 2);
  }
}
int main()
{
  int serial_port = open("/dev/ttyS0", O_RDWR);

  struct termios tty;

  // Read in existing settings, and handle any error
  if (tcgetattr(serial_port, &tty) != 0)
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
  tty.c_cc[VTIME] = 10;  // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 64;

  // Crossfire baud rate
  cfsetispeed(&tty, 420000);
  cfsetospeed(&tty, 420000);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
  {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    return 1;
  }
  while (true)
  {
    uint8_t read_buf[128] = {0};
    int read_bytes = read(serial_port, &read_buf, sizeof(read_buf));
    static std::vector<uint8_t> buffer;
    buffer.insert(buffer.end(), &read_buf[0], &read_buf[read_bytes]);
    printf("Buffer size: %d\n", buffer.size());
    CheckPayloads(buffer);
  }
  close(serial_port);
  return 0; // success
};
