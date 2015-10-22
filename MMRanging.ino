#include "MMRanging.h"
 
MMRanging::MMRanging(DW1000& DW) : dw(DW) {
    event_i = 0;
    counter = 0;
    dw.setCallbacks(this, &MMRanging::callbackRX, &MMRanging::callbackTX);
    dw.setCallbacks(&MMRanging::callbackRX, &MMRanging::callbackTX);
    for (int i = 0; i < 10; i++) {
        acknowledgement[i] = true;
        distances[i] = -1;
    }
    //LocalTimer.start();
    dw.startRX();
}
 
void MMRanging::callbackRX() {
    rangingframe RX;
    dw.readRegister(DW1000_RX_BUFFER, 0, (uint8_t*)&RX, dw.getFramelength());  // get data from buffer
    
    if (RX.destination == address)                                                  // only if received packet is for me
        switch (RX.type) {
            case 1:
                rangingtimingsReceiver[RX.source][0] = dw.getRXTimestamp();
                sendRangingframe(RX.source, RX.sequence_number, 2, 0);
                break;
            case 2:
                rangingtimingsSender[RX.source][1] = dw.getRXTimestamp();
                sendRangingframe(RX.source, counter, 3, 0);
                counter++;
                break;
            case 3:
                sendRangingframe(RX.source, RX.sequence_number, 4, timeDifference40Bit(rangingtimingsReceiver[RX.source][0], rangingtimingsReceiver[RX.source][1]));
                break;
            case 4:
                tofs[RX.source] = timeDifference40Bit(rangingtimingsSender[RX.source][0], rangingtimingsSender[RX.source][1]) - RX.time_difference_receiver;
                acknowledgement[RX.source] = true;
                break;
            default : break;
        }
    
    #ifdef EVENTS
        sprintf(event[event_i], "!R %d>%d / %d %d", RX.source, RX.destination, RX.sequence_number, RX.type);
        if (event_i == 8)
            event_i = 0;
        else
            event_i++;
    #endif
    
    dw.startRX();
}
 
void MMRanging::callbackTX() {
    switch (TX.type) {
        case 1:
            rangingtimingsSender[TX.destination][0] = dw.getTXTimestamp();
            break;
        case 2:
            rangingtimingsReceiver[TX.destination][1] = dw.getTXTimestamp();
            break;
        default: break;
    }
    
    #ifdef EVENTS
        sprintf(event[event_i], "!S %d>%d / %d %d", TX.source, TX.destination, TX.sequence_number, TX.type);
        if (event_i == 8)
            event_i = 0;
        else
            event_i++;
    #endif
}
 
void MMRanging::requestRanging(uint8_t destination) {
    acknowledgement[destination] = false;
    float time_before = micros();
    sendRangingframe(destination, counter, 1, 0);
    while(!acknowledgement[destination] && (micros() < time_before + 0.5f)); // wait for succeeding ranging or timeout
    roundtriptimes[destination] = micros() - time_before;
    distances[destination] = (tofs[destination] * 300 / MMRANGING_TIMEUNIT_US / 2);
}
 
void MMRanging::requestRangingAll() {
    for (int i = 1; i <= 4; i++) {  // Request ranging to all anchors
        requestRanging(i);
    }
}
 
void MMRanging::sendRangingframe(uint8_t destination, uint8_t sequence_number, uint8_t type, uint64_t time_difference_receiver) {
    TX.source = address;
    TX.destination = destination;
    TX.sequence_number = sequence_number;
    TX.type = type;
    TX.time_difference_receiver = time_difference_receiver;
    dw.sendFrame((uint8_t*)&TX, sizeof(TX));
}
 
uint64_t MMRanging::timeDifference40Bit(uint64_t early, uint64_t late) {
    int64_t difference = late - early;
    if ((difference < -MMRANGING_2POWER40+10000000000) && (difference > -MMRANGING_2POWER40-10000000000)) // if the timestamps differ a negative word length +- ~1sec that was potentially measured, correct it
        return difference + MMRANGING_2POWER40;
    if ((difference < 0) || (difference > 10000000000))
        return 10000000000;
    return (uint64_t)difference;
}
