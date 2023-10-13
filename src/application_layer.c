// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer lLayer;
    strcpy(lLayer.serialPort, serialPort);
    lLayer.role = role;
    lLayer.baudRate = baudRate;
    lLayer.nRetransmissions = nTries;
    lLayer.timeout = timeout;

    int fd = llopen(lLayer);

    if (fd < 0)
    {
        printf("Error opening serial port\n");
        exit(-1);
    }

    
    if (lLayer.role == LlTx) {

    }
    else if (lLayer.role == LlRx) {

    }
    else {
        printf("Invalid role\n");
        exit(-1);
    }



}
