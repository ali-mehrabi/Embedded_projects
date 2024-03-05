#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
//#include <string.h>
#include <errno.h>
#include <sys/select.h>
//#include "socal/alt_gpio.h"
#include <socal.h>
#include <hps.h>
#include <stdint.h>
#include <include/hps_0.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <string.h>
//#include <signal.h>
//#include <assert.h>
//#include <pthread.h>
//#include <sys/mman.h>


#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define UART_DEVICE "/dev/ttyS0"
#define PACKET_SIZE 64
#define ARRAY_SIZE  10
//enum State {IDLE, RECEIVING };

int ConfigSerialPort(int fd)
{
   struct termios options;
	tcgetattr(fd, &options);
	cfsetspeed(&options, B115200);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	if (tcsetattr(fd, TCSANOW, &options) != 0)
	    {
	      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	      return 0;
	    }
	tcflush(fd, TCIOFLUSH);
	return 1;
}

char lookup_table(uint16_t a)
{
float c = 6553.6;
int k = a/c;
if(a==0) return 0;
else return (char) k+1;
}
// still not determined how to process data
void process_data(char data[PACKET_SIZE], uint16_t pd[ARRAY_SIZE])
{
    uint16_t acx[ARRAY_SIZE], acy[ARRAY_SIZE], acz[ARRAY_SIZE];

    if (data[0] == 0xff) // Check head of the buffer // for test = ff
      {
        for(int i = 0; i < ARRAY_SIZE; i++) pd[i] = 0;
      }
    else
      {
        for (int i = 0; i < ARRAY_SIZE; i++)
          {
            acx[i] = lookup_table((data[6 * i - 5] << 8) | data[6 * i - 4]);
            acy[i] = lookup_table((data[6 * i - 3] << 8) | data[6 * i - 2]);
            acz[i] = lookup_table((data[6 * i - 1] << 8) | data[6 * i ]);
            pd[i] = ((acz[i] << 8) & 0x0F00) | ((acy[i] << 4) & 0x00F0) | (0x000F & acx[i]);
          }
      }
}

uint16_t pd[ARRAY_SIZE];
int main()
{

void *virtual_base;
void *h2p_lw_led_addr;
void *h2p_lw_sw_addr;
uint32_t byte;
uint16_t pulse_delay = 100;

int axi_fd = open( "/dev/mem", ( O_RDWR | O_SYNC ));
if( axi_fd < 1 )
  {
	printf( "ERROR: could not open \"/dev/mem\"...\n" );
	return( 1 );
  }
virtual_base = mmap(NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, axi_fd, HW_REGS_BASE );
if( virtual_base == MAP_FAILED )
  {
	printf( "ERROR: mmap() failed...\n" );
	close( axi_fd );
	return( 1 );
  }

h2p_lw_led_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LED_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
h2p_lw_sw_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DIPSW_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

char    buffer[PACKET_SIZE];
//enum    State state = IDLE;
ssize_t totalBytesRead = 0;
int     uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY);
if (uart_fd < 0)
  {
	perror("Error opening UART device");
	return 1;
  }

int sr = ConfigSerialPort(uart_fd);
  if (!sr)
	  {
		printf("serial port configuration error");
		return -1;
	  }


while (1)
  {
	fd_set rfds;
	FD_ZERO(&rfds);
	FD_SET(uart_fd, &rfds);
	tcflush(uart_fd, TCIFLUSH);
	struct timeval timeout;
	timeout.tv_sec = 1; // Adjust the timeout if needed
	timeout.tv_usec = 0;

	int ready = select(uart_fd + 1, &rfds, NULL, NULL, &timeout);
	if (ready == -1)
	  {
		perror("Error in select");
		break;
	  }
	else if (ready == 0)
	  {
		// Timeout occurred, no data available
		continue;
	  }

	ssize_t bytesRead = read(uart_fd, buffer + totalBytesRead, PACKET_SIZE - totalBytesRead);

	if (bytesRead > 0)
	  {
		totalBytesRead += bytesRead;
		if (totalBytesRead >= PACKET_SIZE)
		  {
			 process_data(buffer, pd);
			 for(int i = 0; i < ARRAY_SIZE; i++)
			   {
			    byte = (uint32_t) pd[i];   ///write data to FPGA FIFO
	           *(uint32_t *)h2p_lw_led_addr =  byte; // write bye to AXI
	            usleep(pulse_delay);
	            *(uint32_t *)h2p_lw_led_addr = 0x00001000 | byte; // activate write clock
	            usleep(pulse_delay);
	            *(uint32_t *)h2p_lw_led_addr = 0x00000FFF & byte; //following lines for testing only. Remove |byte later
	            usleep(pulse_delay);
	            printf( "byte data sent to FPGA: %X, -  %X\n" , byte,*(uint32_t *)h2p_lw_led_addr);
	            usleep(100);
	            printf( "loop back the written data to FPGA: %X \n" , *(uint32_t *)h2p_lw_sw_addr);
	            usleep(1000);
			   }
			///////////////////////////////for test
			printf("Received data: ");
			for (size_t i = 0; i < PACKET_SIZE; i++)
			  {
				printf("%X ", buffer[i]);
			  }
			printf("\n");
			///////////////////////////////
			// Reset for the next packet
			totalBytesRead = 0;
			//state = IDLE;
		  }
	   else
	     {
			// Incomplete packet, continue receiving
			//state = RECEIVING;
		 }
	  }
   // else if (bytesRead == 0)
   //   {
		// Handle case when no data is available
	//	continue;
	//  }
    else if (bytesRead < 0)
      {
		if (errno == EAGAIN || errno == EWOULDBLOCK)
		  {
			// No data available, continue waiting
			continue;
	      }
	    else
	      {
			perror("Error reading from UART");
			break;
		  }
	   }
  }

close(uart_fd);
return 0;
}
