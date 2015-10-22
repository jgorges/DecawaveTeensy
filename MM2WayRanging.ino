#include "MM2WayRanging.h"
 
MM2WayRanging::MM2WayRanging(DW1000& DW) : dw(DW) {
    isAnchor = true;
    overflow = false;
    address = 0;
    rxTimestamp = 0;
    timediffRec = 0;
    timediffSend = 0;
    for (int i = 0; i < 10; i++)
        acknowledgement[i] = true;
 
    //dw.setCallbacks(this, &MM2WayRanging::callbackRX, &MM2WayRanging::callbackTX);
 
    //LocalTimer.start();
 
    dw.startRX();
}
 
void MM2WayRanging::callbackRX() {
    dw.readRegister(DW1000_RX_BUFFER, 0, (uint8_t*)&receivedFrame, dw.getFramelength());
 
    if (receivedFrame.destination == address)
        switch (receivedFrame.type) {
            case PING:
                rxTimestamp = dw.getRXTimestamp();
                receiverTimestamps[receivedFrame.source][0] = rxTimestamp;      //Save the first timestamp on the receiving node/anchor (T_rp)
                sendDelayedAnswer(receivedFrame.source, ANCHOR_RESPONSE, rxTimestamp);
                break;
            case ANCHOR_RESPONSE:
                rxTimestamp = dw.getRXTimestamp();
                senderTimestamps[receivedFrame.source][1] = rxTimestamp;        //Save the second timestamp on the sending node/beacon (T_rr)
                sendDelayedAnswer(receivedFrame.source, 3, rxTimestamp);
                break;
            case BEACON_RESPONSE:
                rxTimestamp = dw.getRXTimestamp();
                receiverTimestamps[receivedFrame.source][2] = rxTimestamp;      //Save the third timestamp on the receiving node/anchor (T_rf)
 
                correctReceiverTimestamps(receivedFrame.source);                //Correct the timestamps for the case of a counter overflow
                //calculation of the summand on the receiving node/anchor
                timediffRec = - 2*receiverTimestamps[receivedFrame.source][1] + receiverTimestamps[receivedFrame.source][0] + receiverTimestamps[receivedFrame.source][2];
                sendTransferFrame(receivedFrame.source, timediffRec );
                break;
            case TRANSFER_FRAME:
                //calculation of the summand on the sending node/beacon
                timediffSend = 2 * senderTimestamps[receivedFrame.source][1] - senderTimestamps[receivedFrame.source][0] - senderTimestamps[receivedFrame.source][2];
                //calculation of the resulting sum of all four ToFs.
                tofs[receivedFrame.source] = receivedFrame.signedTime + timediffSend;
                acknowledgement[receivedFrame.source] = true;
                break;
            default : break;
        }
 
    dw.startRX();
}
 
void MM2WayRanging::callbackTX() {
    switch (rangingFrame.type) {
    case PING:
        senderTimestamps[rangingFrame.destination][0] = dw.getTXTimestamp();    //Save the first timestamp on the sending node/beacon (T_sp)
        break;
    case ANCHOR_RESPONSE:
        receiverTimestamps[rangingFrame.destination][1] = dw.getTXTimestamp();  //Save the second timestamp on the receiving node/anchor (T_sr)
        break;
    case BEACON_RESPONSE:
        senderTimestamps[rangingFrame.destination][2] = dw.getTXTimestamp();    //Save the third timestamp on the sending node/beacon (T_sr)
        correctSenderTimestamps(rangingFrame.destination);                      //Correct the timestamps for the case of a counter overflow
        break;
    default:
        break;
    }
 
}
 
/**
 *  Get the distance to the Anchor with address @param destination.
 *
 *   @param destination The address of the anchor
 */
void MM2WayRanging::requestRanging(uint8_t destination) {
    acknowledgement[destination] = false;
    float time_before = micros();
 
    sendPingFrame(destination);
 
    while(!acknowledgement[destination] && (micros() < time_before + 0.02f)); // wait for succeeding ranging or timeout
 
    roundtriptimes[destination] = micros() - time_before;
 
    if(acknowledgement[destination]){
    distances[destination] = calibratedDistance(destination);
    } else {
        distances[destination] = -1;
    }
}
 
inline float MM2WayRanging::calibratedDistance(uint8_t destination) {
 
    float rawDistance = (tofs[destination] * 300 * TIMEUNITS_TO_US / 4);
 
 
 
 // Calibration for Nucleo 0 (and 1)
 
 //   if (this->address == 1) rawDistance+= 10;
//    switch(destination){
//        case 2:
//            return  rawDistance * 0.9754 - 0.5004;
//        case 3:
//            return  rawDistance * 0.9759 - 0.4103;
//        case 4:
//            return  rawDistance * 0.9798 - 0.5499;
//        case 5:
//            return  rawDistance * 0.9765 - 0.5169;
//        }
 
    return rawDistance;
 
}
 
void MM2WayRanging::requestRangingAll() {
    for (int i = 1; i <= 4; i++) {  // Request ranging to all anchors
        requestRanging(i);
    }
}
 
void MM2WayRanging::sendPingFrame(uint8_t destination) {
    rangingFrame.source = address;
    rangingFrame.destination = destination;
    rangingFrame.type = PING;
    dw.sendFrame((uint8_t*)&rangingFrame, sizeof(rangingFrame));
}
 
void MM2WayRanging::sendTransferFrame(uint8_t destination, int timeDiffsReceiver) {
    transferFrame.source = address;
    transferFrame.destination = destination;
    transferFrame.type = TRANSFER_FRAME;
    transferFrame.signedTime =  timeDiffsReceiver;                      //cast the time difference
    dw.sendFrame((uint8_t*)&transferFrame, sizeof(transferFrame));
}
 
void MM2WayRanging::sendDelayedAnswer(uint8_t destination, uint8_t type, uint64_t rxTimestamp) {
 
    rangingFrame.source = address;
    rangingFrame.destination = destination;
    rangingFrame.type = type;
 
    if(rxTimestamp + ANSWER_DELAY_TIMEUNITS > MMRANGING_2POWER40)
        dw.sendDelayedFrame((uint8_t*)&rangingFrame, sizeof(rangingFrame), rxTimestamp + ANSWER_DELAY_TIMEUNITS - MMRANGING_2POWER40);
    else
        dw.sendDelayedFrame((uint8_t*)&rangingFrame, sizeof(rangingFrame), rxTimestamp + ANSWER_DELAY_TIMEUNITS);
}
 
void MM2WayRanging::correctReceiverTimestamps(uint8_t source){
 
    if(receiverTimestamps[source][0] > receiverTimestamps[source][1]){
        receiverTimestamps[source][1] += MMRANGING_2POWER40;
        receiverTimestamps[source][2] += MMRANGING_2POWER40;
    }
 
    if(receiverTimestamps[source][1] > receiverTimestamps[source][2]){
            receiverTimestamps[source][2] += MMRANGING_2POWER40;
        }
 
}
 
void MM2WayRanging::correctSenderTimestamps(uint8_t source){
 
    if (senderTimestamps[source][0] > senderTimestamps[source][1]) {
        senderTimestamps[source][1] += MMRANGING_2POWER40;
        senderTimestamps[source][2] += MMRANGING_2POWER40;
        overflow = true;
    } else if (senderTimestamps[source][1] > senderTimestamps[source][2]) {
        senderTimestamps[source][2] += MMRANGING_2POWER40;
        overflow = true;
    }else overflow = false;
 
}
 
            
