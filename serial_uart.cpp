#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

#include "serial_uart.h"

serial_uart::
serial_uart()
{
    reset_counters();
    set_timeout(10,0);
}

serial_uart::
~serial_uart()
{
    close_uart();
}

void serial_uart::close_uart() {
    if (ufd != -1) {
        close(ufd);
        ufd = -1;
        printf("%s closed\n", devfile);
    }
}

void serial_uart::open_uart()
{
    close_uart();

    ufd = open(devfile, O_RDWR);
    if (ufd < 0) {
        printf("Error %i opening %s: %s\n", errno, devfile, strerror(errno));
        close_uart();
        return;
    }

    if (set_interface_attribs() < 0) {
        printf("Error setting interface attributes");
        close_uart();
        return;
    }

    reset_counters();
    printf("%s opened\n", devfile);
}

void serial_uart::set_timeout(long int sec, long int usec)
{
    timeout.tv_sec  = sec;
    timeout.tv_usec = usec;
}

void serial_uart::reset_counters() {
    num_error = 0;
    num_timeouts = 0;
    num_reads = 0;
}

char* serial_uart::get_bytes() {
    fd_set readSet;
    int rv;
    memset(&read_buf,'\0',sizeof(read_buf));
    last_read_valid = false;

    FD_ZERO(&readSet);
    FD_SET(ufd, &readSet);

    // todo: figure out select!
    rv = select(ufd + 1, &readSet, NULL, NULL, &timeout);
    if(rv == -1) {
        goto bail;
    }
    else if(rv == 0) {
        num_timeouts++;
        goto bail;
    }
    else {
        num_bytes = read(ufd, &read_buf, sizeof(read_buf));
        if (num_bytes < 0) {
            printf("Error %i reading %s: %s\n", errno, devfile, strerror(errno));
            goto bail;
        }
        last_read_valid = true;
        num_reads++;
    }
bail:
    return read_buf;
}

int serial_uart::set_interface_attribs()
{

    struct termios tty;

    // Read in existing settings, and handle any error
    if (tcgetattr(ufd, &tty) < 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)baud);
    cfsetispeed(&tty, (speed_t)baud);

    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 bits per byte (most common)
    tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)

    //tty.c_lflag |= ICANON; /* Canonical mode*/
    tty.c_lflag |= ICANON | ISIG;  /* canonical input */
    tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | IEXTEN);

    tty.c_iflag &= ~IGNCR;  /* preserve carriage return */
    tty.c_iflag &= ~INPCK;
    tty.c_iflag &= ~(INLCR | ICRNL | IUCLC | IMAXBEL);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);   /* no SW flowcontrol */

    tty.c_oflag &= ~OPOST;

    tty.c_cc[VEOL] = 0;
    tty.c_cc[VEOL2] = 0;
    tty.c_cc[VEOF] = 0x04;

    if (tcsetattr(ufd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}
