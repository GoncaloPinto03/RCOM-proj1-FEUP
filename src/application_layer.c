// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayerRole lRole;

    if (strcmp(role, "tx") == 0)
    {
        lRole = LlTx;
    }
    else if (strcmp(role, "rx") == 0)
    {
        lRole = LlRx;
    }
    else
    {
        printf("This role is invalid\n");
        exit(-1);
    }
    LinkLayer lLayer;
    strcpy(lLayer.serialPort, serialPort);
    lLayer.role = lRole;
    lLayer.baudRate = baudRate;
    lLayer.nRetransmissions = nTries;
    lLayer.timeout = timeout;

    int fd = llopen(lLayer);

    if (fd < 0)
    {
        printf("Error opening serial port\n");
        exit(-1);
    }

    return;

}
