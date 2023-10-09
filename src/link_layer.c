// Link layer protocol implementation

#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

volatile int stop = FALSE;
int alarmEnabled = FALSE;
int alarmCount = 0;
LinkLayerStateMachine state = START;

////////////////////////////////////////////////
// ALARM HANDLER
////////////////////////////////////////////////
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        return -1;
    }

    unsigned char byte;

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        return -1;
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        return -1;
    }

    switch (connectionParameters.role)
    {
    case LlRx:
        while(state != STOP) 
        {
            if (read(fd, &byte, 1) > 0) {
                switch (state)
                {
                case START:
                    if (byte == FLAG)
                        state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == ASR)
                        state = A_RCV;
                    else if (byte != FLAG)
                        state = START;
                    break;
                case A_RCV:
                    if (byte == CSET)
                        state = C_RCV;
                    else if (byte == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case C_RCV:
                    if (byte == (ASR ^ CSET))
                        state = BCC_OK;
                    else if (byte == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case BCC_OK:
                    if (byte == FLAG)
                        state = STOP;
                    else
                        state = START;
                    break;
                default:
                    break;
                }
            }
        }
        break;
    case LlTx:
        (void) signal(SIGALRM, alarmHandler);
        stop = FALSE;

        while(alarmCount < connectionParameters.nRetransmissions && STOP == FALSE && alarmEnabled == FALSE)
        {
        
            alarm(connectionParameters.timeout);
            if (read(fd, &byte, 1) > 0) {
                switch (state)
                {
                case START:
                    if (byte == FLAG)
                        state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == ARS)
                        state = A_RCV;
                    else if (byte != FLAG)
                        state = START;
                    break;
                case A_RCV:
                    if (byte == CUA)
                        state = C_RCV;
                    else if (byte == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case C_RCV:
                    if (byte == (ARS ^ CUA))
                        state = BCC_OK;
                    else if (byte == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case BCC_OK:
                    if (byte == FLAG)
                        state = STOP;
                    else
                        state = START;
                    break;
                default:
                    break;
                }
            }
            connectionParameters.nRetransmissions--;
        }
        if (state != STOP)
        {
            printf("Connection failed\n");
            return -1;
        }
        break;
    }

    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(int fd, const unsigned char *buf, int bufSize)
{
    unsigned char data[bufSize + 1];

    data[0] = FLAG;
    data[1] = ASR;
    data[2] = CSET;
    data[3] = (ASR ^ CSET);
    data[4] = FLAG;

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    return 1;
}
