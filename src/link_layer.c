// Link layer protocol implementation

#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

volatile int stop = FALSE;
int alarmEnabled = FALSE;
int alarmCount = 0;
int senderNumber = 0;
int receiverNumber = 1;
int lastFrameNumber = -1;

int fd = -1;
int timeout = -1;
int nRetransmissions = -1;

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
    printf("*************************************\n");
    printf("**************** OPEN ***************\n");
    printf("*************************************\n");
    LinkLayerStateMachine state = START;
    
    nRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    
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
                    alarm(-1);
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
   printf("*************************************\n");
   printf("*************** WRITE ***************\n");
   printf("*************************************\n");

    alarmCount = 0;
    int STOP = 0;
    int index = 4;
    int control = (!senderNumber << 7) | 0x05;
    unsigned char BCC = 0x00;
    unsigned char buffer[MAX_FRAME_SIZE]={0};
    unsigned char data[5] = {0};

    //BCC destuffing
    for(int i = 0; i < bufSize; i++){
        BCC = (BCC ^ buf[i]);
    }
    buffer[0] = FLAG;
    buffer[1] = ASR;
    buffer[2] = (senderNumber << 6);
    buffer[3] = buffer[1] ^ buffer[2];

    for(int i = 0; i < bufSize; i++){
        if(buf[i] == FLAG){
            buffer[index++] = ESC;
            buffer[index++] = 0x5e;
            continue;
        }
        else if(buf[i] == ESC){
            buffer[index++] = ESC;
            buffer[index++] = 0x5D;
            continue;
        }

        buffer[index++] = buf[i];
    }

    //byte stuffing
    if(BCC == ESC){
        buffer[index++] = ESC;
        buffer[index++] = 0x5D;
    }
    else if(BCC == FLAG){
        buffer[index++] = ESC;
        buffer[index++] = 0x5E;
    }
    else{
        buffer[index++] = BCC;
    }

    buffer[index++] = FLAG;

    while(!STOP){
        if(!alarmEnabled){
            write(fd, buffer, index);
            printf("Frame sent NS=%d\n", senderNumber);

            (void)signal(SIGALRM, alarmHandler);
            if (alarmEnabled == FALSE)
            {
                alarm(timeout);
                alarmEnabled = TRUE;
            }

        }

        int result = read(fd, data, 5);
        
        if(result != -1 && data != 0){
            if(data[2] != (control) || (data[3] != (data[1]^data[2]))){
                printf("Wrong RR: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                alarmEnabled=FALSE;
                continue;
            }
            else{
                printf("Correct RR: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                alarmEnabled=FALSE;
                STOP = 1;
            }
        }

        if(alarmCount >= nRetransmissions){ //alarm limit reached
            printf("LLWRITE ERROR: Exceeded number of tries while sending frame\n");
            STOP = 1;
            close(fd);
            return -1;
        }
    }
    
    if(senderNumber){
        senderNumber = 0;
    }else{
        senderNumber = 1;
    }

    return 0;
}
    

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *buf, int *packetSize)
{
    printf("*************************************\n");
    printf("**************** READ ***************\n");
    printf("*************************************\n");

    alarmCount = 0;
    int index = 0;
    int control = (!receiverNumber << 6);
    unsigned char BCC2 = 0x00;
    unsigned char frame[MAX_FRAME_SIZE]={0};
    unsigned char data[5] = {0};
    unsigned char aux[400] = {0};
    unsigned char flag = 0;
    unsigned char STOP = FALSE;
    int infoSize = 0;

    unsigned char buffer[1] = {0};

    LinkLayerStateMachine state = START;
    unsigned char readByte = TRUE;

    while (STOP == TRUE)
    {
        if (readByte == TRUE) {
            int bytes = read(fd, buffer, 1);
            if (bytes == -1 || bytes == 0) {
                perror("Error reading from the serial port");
                continue;
            }
        }

        switch (state)
        {
        case START:
            if (buffer[0] == FLAG) {
                state = FLAG_RCV;
                frame[infoSize++] = buffer[0];
            }
            break;
        
        case FLAG_RCV:
            if (buffer[0] != FLAG) {
                state = A_RCV;
                frame[infoSize++] = buffer[0];
            }
            else if (buffer[0] == FLAG) {
                state = FLAG_RCV;
                infoSize = 0;
                frame[infoSize++] = buffer[0];
            }
            break;
        
        case A_RCV:
            if (buffer[0] != FLAG) {
                frame[infoSize++] = buffer[0];
            }
            else if (buffer[0] == FLAG) {
                STOP = TRUE;
                frame[infoSize++] = buffer[0];
                readByte = FALSE;
            }
            break;
        default:
            break;
        }   

    data[0] = FLAG;
    data[1] = ASR;
    data[4] = FLAG;
    }

    if (((frame[1] ^ frame[2]) != frame[3]) || (frame[2] != control)) {
        printf("Wrong frame received\n");
        data[2] = (receiverNumber << 7) | 0x01;
        data[3] = (data[1] ^ data[2]);
        write(fd, data, 5);
        printf("RR sent\n");
        printf("Size of REJ -> %d\n", 5);

        for(int i=0; i<5; i++) {
            printf("%02X ", data[i]);
        }
        return -1;
    }


    for(int i = 0; i < infoSize; i++){
        if(frame[i] == 0x7D && frame[i+1]==0x5e){
            buf[index++] = 0x7E;
            i++;
        }

        else if(frame[i] == 0x7D && frame[i+1]==0x5d){
            buf[index++] = 0x7D;
            i++;
        }

        else {buf[index++] = frame[i];}
    }

    int size = 0;

    if(buf[4]==0x01){
        size = 256*buf[6]+buf[7]+4 +6; //+4 para contar com os bytes de controlo, numero de seq e tamanho
        for(int i=4; i<size-2; i++){
            BCC2 = BCC2 ^ buf[i];
        }
    }
    
    else{
        size += buf[6]+ 3 + 4; //+3 para contar com os bytes de C, T1 e L1 // +4 para contar com os bytes FLAG, A, C, BCC
        size += buf[size+1] + 2 +2; //+2 para contar com T2 e L2 //+2 para contar com BCC2 e FLAG

        for(int i=4; i<size-2; i++){
            BCC2 = BCC2 ^ buf[i];
        }
    }


    if(buf[size-2] == BCC2){

        if(buf[4]==0x01){
            if(frame[5] == lastFrameNumber){
                printf("\nInfoFrame received correctly. Repeated Frame. Sending RR.\n");
                data[2] = (receiverNumber << 7) | 0x05;
                data[3] = data[1] ^ data[2];
                write(fd, data, 5);
                return -1;
            }   
            else{
                lastFrameNumber = frame[5];
            }
        }
        printf("\nInfoFrame received correctly. Sending RR.\n");
        data[2] = (receiverNumber << 7) | 0x05;
        data[3] = data[1] ^ data[2];
        write(fd, data, 5);
    }
    
    else {
        printf("\nInfoFrame not received correctly. Error in data packet. Sending REJ.\n");
        data[2] = (receiverNumber << 7) | 0x01;
        data[3] = data[1] ^ data[2];
        write(fd, data, 5);

        /*printf("\n-----REJ-----\n");
        printf("\nSize of REJ: %d\nREJ: 0x", 5);

        for(int i=0; i<5; i++){
            printf("%02X ", supFrame[i]);
        }

        printf("\n\n");*/

        return -1;
    }

    (*packetSize) = size;

    index = 0;
    
    for(int i=4; i<(*packetSize)-2; i++){
        aux[index++] = buf[i];
    }

    
    (*packetSize) = size - 6;

    memset(buf,0,sizeof(buf));

    for(int i=0; i<(*packetSize); i++){
        buf[i] = aux[i];
    }


    if(receiverNumber){
        receiverNumber = 0;
    }
    else {receiverNumber = 1;}

    return 1;

}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

int llclose(int showStatistics, LinkLayer connectionParameters, float runTime) {
    alarmCount = 0;

    printf("*************************************\n");
    printf("*************** CLOSE ***************\n");
    printf("*************************************\n");

    switch (connectionParameters.role)
    {
    case LlRx:
        unsigned char buffer[6] = {0}, data[6] = {0};
        unsigned char STOP = 0, UA = 0;

        buffer[0] = 0x7E;
        buffer[1] = 0x03;
        buffer[2] = 0x0B;
        buffer[3] = buffer[1]^buffer[2];
        buffer[4] = 0x7E;
        buffer[5] = '\0';


        while(!STOP){
            int result = read(fd, data, 5);
            
            data[5] = '\0';

            if(result==-1){
                continue;
            }


            else if(strcasecmp(buffer, data) == 0){
                printf("\nDISC message received. Responding now.\n");
                
                buffer[1] = 0x01;
                buffer[3] = buffer[1]^buffer[2];

                while(alarmCount < connectionParameters.nRetransmissions){

                    if(!alarmEnabled){
                        printf("\nDISC message sent, %d bytes written\n", 5);
                        write(fd, buffer, 5);
                        startAlarm(connectionParameters.timeout);
                    }
                    
                    int result = read(fd, data, 5);
                    if( data != 0 && result != -1 && data[0]==0x7E){
                        //se o UA estiver errado 
                        if(data[2] != 0x07 || (data[3] != (data[1]^data[2]))){
                            printf("\nUA not correct: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                            alarmEnabled = FALSE;
                            continue;
                        }
                        
                        else{   
                            printf("\nUA correctly received: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                            alarmEnabled = FALSE;
                            close(fd);
                            break;
                        }
                    }

                }

                if(alarmCount >= connectionParameters.nRetransmissions){
                    printf("\nAlarm limit reached, DISC message not sent\n");
                    return -1;
                }
                
                STOP = TRUE;
            }
        
        }
        break;

    case LlTx:
        alarmCount = 0;

        unsigned char buffer[6] = {0}, data[6] = {0};

        buffer[0] = 0x7E;
        buffer[1] = 0x03;
        buffer[2] = 0x0B;
        buffer[3] = buffer[1]^buffer[2];
        buffer[4] = 0x7E;
        buffer[5] = '\0'; //assim posso usar o strcmp

        while(alarmCount < connectionParameters.nRetransmissions){

            if(!alarmEnabled){
                
                int bytes = write(fd, buffer, 5);
                printf("\nDISC message sent, %d bytes written\n", bytes);
                startAlarm(connectionParameters.timeout);
            }

            //sleep(2);
            
            int result = read(fd, data, 5);

            buffer[1] = 0x01;
            buffer[3] = buffer[1]^buffer[2];
            data[5] = '\0';

            if(result != -1 && data != 0 && data[0]==0x7E){
                //se o DISC estiver errado 
                if(strcasecmp(buffer, data) != 0){
                    printf("\nDISC not correct: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                    alarmEnabled = FALSE;
                    continue;
                }
                
                else{   
                    printf("\nDISC correctly received: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                    alarmEnabled = FALSE;
                    
                    buffer[1] = 0x01;
                    buffer[2] = 0x07;
                    buffer[3] = buffer[1]^buffer[2];

                    int bytes = write(fd, buffer, 5);

                    close(fd);

                    printf("\nUA message sent, %d bytes written.\n\nI'm shutting off now, bye bye!\n", bytes);
                    return 1;

                }
            }

        }

        if(alarmCount >= connectionParameters.nRetransmissions){
            printf("\nAlarm limit reached, DISC message not sent\n");
            close(fd);
            return -1;
        }
    }

    if(showStatistics){
        printf("*************************************\n");
        printf("*************** STATS ***************\n");
        printf("*************************************\n");
        printf("\nNumber of packets sent: %d\nSize of data packets in information frame: %d\nTotal run time: %f\nAverage time per packet: %f\n", lastFrameNumber, 200, runTime, runTime/200.0);

    }

    return 1;
}
