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

        while (alarmCount < connectionParameters.nRetransmissions && state != STOP) 
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
    /*
    unsigned char data[bufSize + 1];

    while (TRUE) {
        if (alarmEnabled == FALSE) {
            data[0] = FLAG;
            data[1] = ASR;
            data[2] = CSET;
            data[3] = (data[1] ^ data[2]);
            data[4] = FLAG;
            
            int bytes = write(fd, buf, bufSize + 1);
            printf("Bytes written: %d\n", bytes);
            alarm(3);
            alarmEnabled = TRUE;
        }
        else {
            int bytes = read(fd, buf, bufSize + 1);
            if (((data[1] ^data[2]) ^ data[3]) == 0) {
                alarm(0);
                for (unsigned int i = 0; i < 5; i++) {
                    printf("%x ", data[i]);
                    break;
                }
            }
        }
        */
    // Create a frame based on your protocol, add proper header/footer, and calculate BCC
    // Create a frame based on your protocol, add proper header/footer, and calculate BCC
    unsigned char frame[MAX_FRAME_SIZE];
    int frameSize = createFrame(buf, bufSize, frame);
    
    // Send the frame
    int bytesSent = write(fd, frame, frameSize);
    
    if (bytesSent != frameSize) {
        // Handle transmission errors
        return -1;
    }
    
    // Wait for acknowledgment from the receiver and handle it
    unsigned char response;
    int bytesRead = read(fd, &response, 1);

    if (bytesRead == 1 && response == ACK) {
        // Frame acknowledged
        return bufSize;
    } else {
        // Handle acknowledgment error
        return -1;
    }
}
    

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(int fd, unsigned char *buf, int bufSize)
{
    unsigned char frame[MAX_FRAME_SIZE];
    int frameSize = readFrame(fd, frame, MAX_FRAME_SIZE);

    if (frameSize == -1) {
        // Handle read error
        return -1;
    }

    if (frameSize < HEADER_SIZE) {
        // Invalid frame
        return -1;
    }

    // Process and validate the received frame, check BCC, etc.

    // If the frame is valid, send an ACK
    if (sendAck(fd) == -1) {
        // Handle ACK transmission error
        return -1;
    }

    // Copy the payload to the output buffer
    int payloadSize = frameSize - HEADER_SIZE;
    if (payloadSize <= bufSize) {
        memcpy(buf, frame + HEADER_SIZE, payloadSize);
        return payloadSize;
    } else {
        // Buffer is too small
        return -1;
    }

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

int llclose(int fd, int showStatistics) {
    if (showStatistics) {
        printf("Closing the link layer.\n");
    }

    if (stop) {
        return 0;
    }

    // Assume you are the receiver, waiting for DISC frame
    if (receiveDISC(fd) == -1) {
        printf("Error receiving DISC frame.\n");
        return -1;
    }

    printf("Link layer closed successfully.\n");

    stop = TRUE;

    return 0;
}
