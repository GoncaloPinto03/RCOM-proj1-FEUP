// Link layer protocol implementation
#include "link_layer.h"
// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

int alarmEnabled = FALSE;
int alarmCount = 0;

int numSender = 0;
int numReceiver = 1;

int numLastFrame = -1;

int fd = -1;
int timeout = -1;
int baudRate = -1;
int nRetransmissions = -1;

LinkLayerRole role;

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

// RUN ALARM 

////////////////////////////////////////////////

int alarmRun(int timeout)
{
    (void)signal(SIGALRM, alarmHandler);

    if (alarmEnabled == FALSE)
    {
        alarm(timeout);
        alarmEnabled = TRUE;
    }
    return 0;
}

////////////////////////////////////////////////

// INIT

////////////////////////////////////////////////

int llinit(LinkLayer connectionParameters) {
    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);

    alarmCount = 0;
    if (fd < 0)
    {
        //perror(connectionParameters.serialPort);
        exit(-1);
    }


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
    return fd;
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
    baudRate = connectionParameters.baudRate;
    role = connectionParameters.role;

    if (llinit(connectionParameters) == -1) {
        return -1;
    }
    printf("New termios structure set\n");
    volatile int stop = FALSE;

    if (connectionParameters.role == LlRx) {

        unsigned char buffer[1] = {0};
        unsigned char data[5] = {0}; // +1: Save space for the final '\0' char
        unsigned char readByte = TRUE;
        state = START;

        while(stop == FALSE) 
        {
            if (readByte == TRUE) {
                int bytes = read(fd, &buffer, 1);
                if (bytes == 0 || bytes == -1) {
                    continue;
                }
            }   

            switch (state)
            {
                case START:
                    if (buffer[0] == FLAG) {
                        state = FLAG_RCV;
                        data[0] = buffer[0];
                    }
                    break;
                case FLAG_RCV:
                    if (buffer[0] != FLAG) {
                        state = A_RCV;
                        data[1] = buffer[0];
                    }
                    else if (buffer[0]== FLAG){ 
                        state = START;
                        memset(data, 0, 5);
                    }
                    break;
                case A_RCV:
                    if (buffer[0] != FLAG) {
                        state = C_RCV;
                        data[2] = buffer[0];
                    }
                    else if (buffer[0] == FLAG) {
                        state = START;
                        memset(data, 0, 5);
                    }
                    break;
                case C_RCV:
                    if (buffer[0] != FLAG) {
                        state = BCC_OK;
                        data[3] = buffer[0];
                    }
                    else if (buffer[0] == FLAG) {
                        state = START;
                        memset(data, 0, 5);
                    }
                    break;
                case BCC_OK:
                    if (buffer[0] == FLAG) {
                        state = STOP;
                        data[4] = buffer[0];
                    }
                    else if (buffer[0] != FLAG){
                        state = START;
                        memset(data, 0, 5);
                    }
                    break;
                case STOP:
                    if ((data[1] ^ data[2]) == data[3]) {
                        printf("Rceived SET message!\n");
                        stop = TRUE;
                    }
                    else {
                        state = START;
                        memset(data, 0, 5);
                        readByte = TRUE;
                    }
                    break;
                default:
                    break;
                }
            }
        data[2] = CUA;
        data[3] = (data[1] ^ data[2]);

        int bytes = write(fd, data, sizeof(data));
        printf("UA MESSAGE SENT -> %d BYTES WRITTEN\n", bytes);
    }
    else if (connectionParameters.role == LlTx) {
        unsigned char buffer[5] = {FLAG, ASR, CSET, ASR^CSET, FLAG};
        unsigned char data[5] = {0}; 

	    alarmCount = 0;

        while (alarmCount < 3) {
            int bytes = write(fd, buffer, sizeof(buffer));
            printf("SET MESSAGE ---> %d BYTES WRITTEN\n", bytes); 
            
            if (alarmEnabled == FALSE) {               
                (void)signal(SIGALRM, alarmHandler);
                alarm(timeout);
		        alarmEnabled = TRUE;
            }

            int res = read(fd, data, 5);
            if (res != -1 && data != 0 && data[0] == FLAG) {
                if (data[2] == CUA && (data[3] == (data[1] ^ data[2]))) {
                    //printf("CORRECT UA\n");
                    //printf("UA: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                    alarmEnabled = FALSE;
                    break;
                }
                else {
                    //printf("WRONG UA\n");
                    //printf("UA: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                    alarmEnabled = FALSE;
                    continue;
                }
            }
        }

        if (alarmCount >= 4) {
            printf("ALARM LIMIT\n");
            return -1;
        }
    }
    alarmEnabled = FALSE;
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
    int ctrl = (!numSender << 7) | 0x05; // 7 bits menos significativos sao preenchidos com 0, e os 25 bits mais significativos sao preenchidos com o resultado do deslocamento
    unsigned char BCC = 0x00; 
    unsigned char buffer[MAX_FRAME_SIZE]={0};
    unsigned char data[5] = {0};

    // byte destuffing
    for(int i = 0; i < bufSize; i++){
        BCC = (BCC ^ buf[i]);
    }

    buffer[0] = FLAG;
    buffer[1] = ASR;
    buffer[2] = C_N(numSender);
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

    /*
    byte stuffing -> utilizado para evitar conflitos entre bytes de dados e bytes de controle
    Se BCC==ESC ele é escrito como ESC 0x5D
    Se BCC==FLAG ele é transmitido como ESC 0x5E
    Isto evita que ESC e FLAG sejam mal interpretados como inicio ou fim de trama/quadro de dados 
    */
    if(BCC == ESC){
        // ESC é usado como um indicador de que o próximo caracter não deve ser interpretado como dado, mas sim como um caracter de controlo
        buffer[index++] = ESC;
        buffer[index++] = 0x5D;
    }else if(BCC == FLAG){
        buffer[index++] = ESC;
        buffer[index++] = 0x5E;
    }else{
        buffer[index++] = BCC;
    }
    buffer[index++] = FLAG;

    while(!STOP){
        if(alarmEnabled == FALSE){
            write(fd, buffer, index);
            //printf("Frame sent NS=%d\n", numSender);

            (void)signal(SIGALRM, alarmHandler);
            if (alarmEnabled == FALSE)
            {
                alarm(timeout);
                alarmEnabled = TRUE;
            }
        }

        // Lidar com resposta do recetor
        int final_res = read(fd, data, 5);
        if(final_res != -1 && data != 0){
            //verificar variavel control da resposta com a trama enviada
            //verificar address_sender^bcc da resposta com a da trama
            if(data[2] != (ctrl) || (data[3] != (data[1]^data[2]))){
                //printf("Wrong RR: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                alarmEnabled=FALSE;
                continue;
            }
            else{
                //printf("Correct RR: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
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

    if(numSender){
        numSender = 0;
    }else{
        numSender = 1;
    }
    return 0;
}

////////////////////////////////////////////////

// LLREAD

////////////////////////////////////////////////

int llread(unsigned char *packet, int *packetSize)
{
    printf("*************************************\n");
    printf("**************** READ ***************\n");
    printf("*************************************\n");

    alarmCount = 0;
    int index = 0;
    int ctrl = (!numReceiver << 6);
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
    while (STOP == FALSE)
    {
        if (readByte == TRUE) {
            int bytes = read(fd, buffer, 1); // ler um byte de cada vez
            if (bytes == -1 || bytes == 0) {
                //perror("Error reading from the serial port");
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
            // visto que acabamos de ler uma flag não podemos ler outra, senão já estariamos na proxima trama ({FLAG,A,C,BCC,FLAG}, {FLAG,A,C,BCC,FLAG})
            if (buffer[0] != FLAG) {
                state = A_RCV;
                frame[infoSize++] = buffer[0];
            }
            // se lermos outra flag ent esta nova passa a ser o inicio da trama, logo continuamos neste estado à espera de algo diferente da flag
            else if (buffer[0] == FLAG) {
                memset(frame, 0, MAX_FRAME_SIZE);
                state = FLAG_RCV;
                infoSize = 0;
                frame[infoSize++] = buffer[0];
            }
            break;
        case A_RCV:
            // continuamos até receber uma flag
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
    }
    data[0] = FLAG;
    data[1] = ASR;
    data[4] = FLAG;

    if (((frame[1] ^ frame[2]) != frame[3]) || (frame[2] != ctrl)) {
        printf("Wrong frame received\n");
        data[2] = C_REJ(numReceiver);
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
        if(frame[i] == 0x7D && frame[i+1] == 0x5e){
            packet[index++] = 0x7E;
            i++;
        }
        else if(frame[i] == 0x7D && frame[i+1]==0x5d){
            packet[index++] = 0x7D;
            i++;
        }
        else {
            packet[index++] = frame[i];
        }
    }
    int size = 0;
    if(packet[4] == 0x01){

        size = 256 * packet[6] + packet[7] + 4 + 6; //4 ->  bytes ctrl, numero de seq e tamanho

        for(int i=4; i<size-2; i++){
            BCC2 = BCC2 ^ packet[i];
        }
    }else{
        size += packet[6] + 3 + 4; //  3 -> bytes C, T1, L1, 4 -> bytes flag, a, c, bcc
        size += packet[size+1] + 2 + 2; //2 -> bytes T2, L2, 2 -> bytes bcc2, flag

        for(int i = 4; i < size-2; i++){
            BCC2 = BCC2 ^ packet[i];
        }
    }

    if(packet[size-2] == BCC2){
        if(packet[4] == 0x01){
            if(frame[5] == numLastFrame){
                printf("\nFrame received correctly. Repeated Frame. RR sent.\n");
                data[2] = C_RR(numReceiver);
                data[3] = data[1] ^ data[2];
                write(fd, data, 5);
                return -1;
            }   
            else{
                numLastFrame = frame[5];
            }
        }

        printf("\nFrame received correctly. RR sent.\n");
        data[2] = C_RR(numReceiver);
        data[3] = data[1] ^ data[2];
        write(fd, data, 5);

    }else {
        printf("\nERROR - Not received the frame correctly. REJ sent.\n");
        data[2] = C_REJ(numReceiver);
        data[3] = data[1] ^ data[2];
        write(fd, data, 5);
        return -1;
    }

    (*packetSize) = size;
    index = 0;
    for(int i = 4; i < (*packetSize)-2; i++){
        aux[index++] = packet[i];
    }

    (*packetSize) = size - 6;
    memset(packet,0,sizeof(packet));

    for(int i=0; i<(*packetSize); i++){
        packet[i] = aux[i];
    }

    if(numReceiver){
        numReceiver = 0;
    }
    else{numReceiver = 1;}
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
    if (role == LlRx)
    {
        unsigned char buffer[6] = {0}, data[6] = {0};
        buffer[0] = 0x7E;
        buffer[1] = 0x03;
        buffer[2] = 0x0B;
        buffer[3] = buffer[1]^buffer[2];
        buffer[4] = 0x7E;
        buffer[5] = '\0'; //assim posso usar o strcmp

        unsigned char STOP = FALSE, UA = FALSE;
        while(!STOP){
            int final_res = read(fd, data, 5);
            data[5] = '\0';
            if(final_res == -1){
                continue;

            }else {
                printf("\nDISC message received. Responding now.\n");
                buffer[1] = 0x01;
                buffer[3] = buffer[1]^buffer[2];

                while(alarmCount < nRetransmissions){
                    if(!alarmEnabled){
                        printf("\nDISC message sent, %d bytes written\n", 5);
                        write(fd, buffer, 5);
                        (void)signal(SIGALRM, alarmHandler);
                        if (alarmEnabled == FALSE)
                        {
                            alarm(timeout);
                            alarmEnabled = TRUE;
                        }
                    }

                    int final_res = read(fd, data, 5);

                    if( data != 0 && final_res != -1 && data[0]==0x7E){
                        //se o UA estiver errado 
                        if(data[2] != 0x07 || (data[3] != (data[1]^data[2]))){
                            //printf("\nUA not correct: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                            alarmEnabled = FALSE;
                            continue;
                        }
                        else{   
                            //printf("\nUA correctly received: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                            alarmEnabled = FALSE;
                            close(fd);
                            break;
                        }
                    }
                }
                if(alarmCount >= nRetransmissions){
                    printf("\nAlarm limit reached, DISC message not sent\n");
                    return -1;
                }
                STOP = TRUE;
            }
        }
    }else if (role == LlTx) {
        unsigned char buffer[6] = {0}, data[6] = {0};
        buffer[0] = 0x7E;
        buffer[1] = 0x03;
        buffer[2] = 0x0B;
        buffer[3] = buffer[1]^buffer[2];
        buffer[4] = 0x7E;
        buffer[5] = '\0';

        alarmCount = 0;

        while(alarmCount < 3){
            if(!alarmEnabled){
                int bytes = write(fd, buffer, 5);
                printf("\nDISC message sent, %d bytes written\n", bytes);

                (void)signal(SIGALRM, alarmHandler);
                if (alarmEnabled == FALSE)
                {
                    alarm(timeout);
                    alarmEnabled = TRUE;
                }
            }
            int final_res = read(fd, data, 5);

            buffer[1] = 0x01;
            buffer[3] = buffer[1]^buffer[2];

            data[5] = '\0';
            if(final_res != -1 && data != 0 && data[0]==0x7E){
                //se o DISC estiver errado 
                if(strcasecmp(buffer, data) != 0){
                    //printf("\nDISC not correct: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                    alarmEnabled = FALSE;
                    continue;
                }else{   
                    //printf("\nDISC correctly received: 0x%02x%02x%02x%02x%02x\n", data[0], data[1], data[2], data[3], data[4]);
                    alarmEnabled = FALSE;
                    buffer[1] = 0x01;
                    buffer[2] = 0x07;
                    buffer[3] = buffer[1]^buffer[2];

                    int bytes = write(fd, buffer, 5);
                    close(fd);

                    //printf("\nUA message sent, %d bytes written.\n\nI'm shutting off now, bye bye!\n", bytes);
                    return 1;
                }
            }
        }

        if(alarmCount >= nRetransmissions){
            printf("\nAlarm limit reached, DISC message not sent\n");
            close(fd);
            return -1;
        }
    }

    if(showStatistics){
        printf("*************************************\n");
        printf("*************** STATS ***************\n");
        printf("*************************************\n");
        printf("\nSent %d Packets\nRun time: %f\nData Packets size in frame: %d\nAverage time per packet: %f\n", numLastFrame, runTime, 200, runTime/200.0);
    }
    return 1;
}

