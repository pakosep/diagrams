#include <stdio.h>
#include <stdlib.h>	
#include <string.h>
#include <unistd.h>                                                          
#include <stdint.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <stdbool.h>
#include <stropts.h>
#include <poll.h>	
#include <errno.h>	

#define BAUDRATE B115200
//#define BAUDRATE B57600,B115200
#define MODEMDEVICE "/dev/ttyUSB1"
char *portname = "/dev/ttyUSB0";
//#define MODEMDEVICE "/dev/ttyACM2"

/*
Punkty na które musisz zwrócić uwagę to
1) w Linuksie urządzenie szeregowe to plik
2) funkcja poll () czeka na dane w linii szeregowej i zwraca z fds [0] .revents -> POLLRDNORM
3) res = read (fd, buf, 255); odczytuje do 255 znaków z bufora szeregowego
4) sscanf (buf, "% d \ n", & zmienna); skanuje łańcuch znaków buf i konwertuje dane na liczbę całkowitą i umieszcza je w zmiennej.
5) Funkcje set_blocking () i set_interface_attribs () zostały zebrane z sieci.

Funkcja fopen() otwiera plik, którego nazwa podana jest w pierwszym argumencie. Drugim jest łańcuch znaków zwierający litery oznaczające sposób otwarcia pliku:
"r" - otwiera plik do czytania
"r+" - otwiera plik do czytania i nadpisywania (aktualizacja)
"w" - otwiera plik do nadpisywania (zamazuje starą treść)
"w+" - otwiera plik do nadpisywania i czytania
"a" - otwiera plik do dopisywania (jeśli plik nie istnieje, to jest tworzony)
"a+" - otwiera plik do dopisywania i odczytu (jeśli plik nie istnieje, to jest tworzony)
"t" - otwiera plik w trybie tekstowym
"b" - otwiera plik w trybie binarnym
Litery można ze sobą łączyć, np. "rwb" albo "wt".
*/

int set_interface_attribs (int fd, int speed, int parity)	{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
      printf ("error %d from tcgetattr", errno);
      return -1;
    }
      cfsetospeed (&tty, speed);
      cfsetispeed (&tty, speed);
      //printf("speed=%d \n",cfgetospeed(&tty));
      tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
      // disable IGNBRK for mismatched speed tests; otherwise receive break
      // as \000 chars
      tty.c_iflag &= ~IGNBRK;         // disable break processing
      tty.c_lflag = 0;                // no signaling chars, no echo,
                                      // no canonical processing
      tty.c_oflag = 0;                // no remapping, no delays
      tty.c_cc[VMIN]  = 0;            // read doesn't block
      tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

      tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

      tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
      tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
      tty.c_cflag |= parity;
      tty.c_cflag &= ~CSTOPB;
      tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
      return -1;
    }
    return 0;
	}

void set_blocking (int fd, int should_block)	{
      struct termios tty;
      memset (&tty, 0, sizeof tty);
      if (tcgetattr (fd, &tty) != 0)
      {
        printf("error %d from tggetattr", errno);
        return;
      }
      tty.c_cc[VMIN]  = should_block ? 1 : 0;
      tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
      if (tcsetattr (fd, TCSANOW, &tty) != 0)
    printf("error %d setting term attributes", errno);
	}
	
int main(int argc, char *argv[])
  { 
    int fd;                                                             
    char buf[255];  
    struct pollfd fds[1];
    int ret, res, cnt=0;
    FILE *fp;
    cnt=cnt;
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    long db[6];
    float dbf[6];

    char fbuf[300 +1];
    char myhostname[256 +1];
	
		/* Intitalise */

    if ((argc != 2))
    {
      printf("Uzycie: %s /dev/ttyX \n",argv[0]);
      return 0;
    }
    //printf("Port %s plik %s \n",argv[1],argv[2]
    //sscanf(argv[2],"%d",&var);
    
    /* open the device */
    fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
    //fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    //fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    printf("fd = %d \n",fd);
    //printf("sp = %d \n",var);
    
    if (fd < 0)    {
      printf ("Error %d opening %s: %s \n", errno, portname, strerror (errno));
      return 0;
    }
    
    set_interface_attribs (fd, B115200, 0);  	// set speed to BAUDRATE  bps, 8n1 (no parity)
    set_blocking (fd, 0);                	// set no blocking

    /* Open STREAMS device. */
    fds[0].fd = fd;
    fds[0].events = POLLRDNORM;

		for (;;)		// forever
    {
      ret = poll(fds, 1, 1000);			// wait for response
      if (ret > 0)
			{ 
				/* An event on one of the fds has occurred. */
				if (fds[0].revents & POLLHUP)
				{
					printf("Hangup\n");
				}
				if (fds[0].revents & POLLRDNORM)
				{
					res = read(fd,buf,255);
					
					buf[res]=0;
					printf("res= %d %s %d \r\n",res,buf,B115200); 	// from serial
					//printf("res=%d \r\n",res); 	// from serial

				}
			}
    }
  }
