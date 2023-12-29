
#ifndef __serial_uart_h_
#define __serial_uart_h_

#include <iostream>
#include <sys/select.h>
#include <termios.h>

class serial_uart
{
public:
    serial_uart();
    ~serial_uart();

    void set_baud(int _baud) { baud = _baud; }
    void set_devfile(const char * _devfile) { devfile = _devfile; }
    int set_interface_attribs();

    void open_uart();
    void close_uart();

    bool is_open() { return (ufd > 0) ? true : false; }
    int get_fd() { return ufd; }

    void set_timeout(long int sec, long int usec);
    char* get_bytes();

    void reset_counters();

public:
    bool last_read_valid;
    int num_error;
    int num_timeouts;
    int num_reads;

private:

    int ufd = -1;
    int baud;

    const char *devfile;
    char read_buf[256];
    int num_bytes;

    struct timeval timeout;
};

#endif /* serial_uart_h_ */
