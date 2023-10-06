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
}
