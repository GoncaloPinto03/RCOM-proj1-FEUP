// Link layer protocol implementation

#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

volatile int stop = FALSE;
int alarmEnabled = FALSE;
int alarmCount = 0;

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
    LinkLayerStateMachine state = START;
    
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

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 1;  // Blocking read until 5 chars received

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

        unsigned char buffer[1] = {0};
        unsigned char data[5] = {0}; // +1: Save space for the final '\0' char
        
        while(state != STOP) 
        {
            if (read(fd, &buffer, 1) > 0) {
                switch (state)
                {
                case START:
                    if (buffer[0] == FLAG) 
                    {
                        state = FLAG_RCV;
                        data[0] = buffer[0];
                    }
                    break;
                case FLAG_RCV:
                    if (buffer[0] != FLAG) {
                        state = A_RCV;
                        data[1] = buffer[0];
                    }
                    else if (buffer[0] == FLAG){ 
                        state = START;
                        memset(data, 0, sizeof(data));
                    }
                    break;
                case A_RCV:
                    if (buffer[0] != FLAG) {
                        state = C_RCV;
                        data[2] = buffer[0];
                    }
                    else if (buffer[0] == FLAG) {
                        state = START;
                        memset(data, 0, sizeof(data));
                    }
                    break;
                case C_RCV:
                    if (buffer[0] != FLAG) {
                        state = BCC_OK;
                        data[3] = buffer[0];
                    }
                    else if (byte == FLAG) {
                        state = START;
                        memset(data, 0, sizeof(data));
                    }
                    break;
                case BCC_OK:
                    if (buffer[0] != FLAG) {
                        state = STOP;
                        data[4] = buffer[0];
                    }
                    else {
                        state = START;
                        memset(data, 0, sizeof(data));
                    }
                    break;
                case STOP:
                    if ((data[1] ^ data[2]) == data[3]) {
                        printf("Connection established\n");
                        stop = TRUE;
                    }
                    else {
                        state = START;
                        memset(data, 0, sizeof(data));
                    }
                    break;
                default:
                    break;
                }
            }
        }

        data[2] = CUA;
        data[3] = (data[1] ^ data[2]);

        int bytes = write(fd, data, sizeof(data));
        printf("UA MESSAGE SENT -> %d BYTES WRITTEN\n", bytes);

        break;
    case LlTx:

        unsigned char buffer[1] = {0};
        unsigned char data[5] = {0}; // +1: Save space for the final '\0' char

        buffer[0] = FLAG;
        buffer[1] = ASR;
        buffer[2] = CSET;
        buffer[3] = (buffer[1] ^ buffer[2]);
        buffer[4] = FLAG;

        while (alarmCount < connectionParameters.nRetransmissions) {
            
            if (alarmEnabled == FALSE) {

                int bytes = write(fd, buffer, sizeof(buffer));
                printf("SET message sent -> %d bytes written\n", bytes);                
                
                (void)signal(SIGALRM, alarmHandler);
                if (alarmEnabled == FALSE) {
                    alarm(connectionParameters.timeout);
                    alarmEnabled = TRUE;
                }
            }

            int bytes = read(fd, data, sizeof(data));
            if (bytes > 0 && data != 0 && data[0] == FLAG) {
                
                if (data[2] != CUA || data[3] != (data[1] ^ data[2])) {
                    printf("Connection failed -> UA Incorrect!\n");
                    printf("UA: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                    alarmEnabled = FALSE;
                    continue;;
                }
                else {
                    printf("Connection established -> UA Correct!\n");
                    printf("UA: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                    alarmEnabled = FALSE;
                    break;
                }
            }
        }

        if (alarmCount >= connectionParameters.nRetransmissions) {
            printf("Connection failed -> Alarm limit reached\n");
            return -1;
        }
    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
   
}
    

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *buf, int *packetSize)
{
    
}

// Function to send an acknowledgment
int sendAck(int fd) {
    unsigned char ack = ACK;
    int bytesSent = write(fd, &ack, 1);
    
    if (bytesSent == 1) {
        return 0;  // Successfully sent ACK
    } else {
        return -1;  // Failed to send ACK
    }
}

////////////////////////////////////////////////
// Send a DISC frame and wait for acknowledgment
////////////////////////////////////////////////
int sendDISC(int fd)
{
    unsigned char discFrame[HEADER_SIZE] = {
        0x7E,  // FLAG
        0x03,  // A (Address)
        0x0B,  // C (Control) - DISC frame
        0x0B,  // BCC (Block Check Character) - XOR of A and C
        0x7E,  // FLAG
    };

    if (write(fd, discFrame, HEADER_SIZE) == -1) {
        perror("Error sending DISC frame");
        return -1;
    }

    printf("DISC frame sent.\n");

    return 0;
}

////////////////////////////////////////////////
// Wait for a DISC frame and send acknowledgment
////////////////////////////////////////////////
int receiveDISC(int fd)
{
    unsigned char discFrame[MAX_FRAME_SIZE];
    int bytesRead = 0;

    while (1) {
        unsigned char byte;
        if (read(fd, &byte, 1) == -1) {
            perror("Error reading from the serial port");
            return -1;
        }

        discFrame[bytesRead++] = byte;

        if (bytesRead >= HEADER_SIZE && discFrame[bytesRead - 1] == 0x7E) {
            break; // End of frame
        }
    }

    printf("Received DISC frame. Sending acknowledgment.\n");

    // Define an acknowledgment (ACK) variable
    unsigned char ACK2 = 0x06;

    // Send acknowledgment (ACK)
    if (write(fd, &ACK2, 1) == -1) {
        perror("Error sending ACK");
        return -1;
    }

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

int llclose(int showStatistics) {
    return 0;
}
