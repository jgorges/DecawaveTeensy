// by Matthias Grob & Manuel Stalder - ETH Zürich - 2015
 
#ifndef MM2WAYRANGING_H
#define MM2WAYRANGING_H
 
#include "arduino.h"
#include "DW1000.h"
 
#define TIMEUNITS_TO_US       (1/(128*499.2))               // conversion between the decawave timeunits (ca 15.65ps) to microseconds.
#define US_TO_TIMEUNITS       (128*499.2)                   // conversion between microseconds to the decawave timeunits (ca 15.65ps).
#define MMRANGING_2POWER40          1099511627776               // decimal value of 2^40 to correct timeroverflow between timestamps
 
 
 
//Predefined delay for the critical answers in the ranging algorithm
//HAS TO BE BIGGER THAN THE PROCESSING TIME OF THE FRAME ON THE NODE
#define ANSWER_DELAY_US             2500                                    //2500 works for 110kbps, 900 for 6.8Mbps
#define ANSWER_DELAY_TIMEUNITS      ANSWER_DELAY_US * (128*499.2)
 
class MM2WayRanging {
 
public:
    MM2WayRanging(DW1000& DW);
 
    void requestRanging(uint8_t destination);
    void requestRangingAll();
 
 
 
    //TODO: Better capsulation on those?
    bool isAnchor;
    uint8_t address; // Identifies the nodes as source and destination in rangingframes
 
    //TODO: Make those PRIVATE!
    float roundtriptimes[10]; // Array containing the round trip times to the anchors or the timeout which occured
    float distances[10]; // Array containing the finally calculated Distances to the anchors
 
    bool overflow;              // TRUE if counter overflows while ranging
 
private:
 
 
    DW1000& dw;
    //Timer LocalTimer;
 
    void callbackRX();
    void callbackTX();
    void sendPingFrame(uint8_t destination);
    void sendDelayedAnswer(uint8_t destination, uint8_t type, uint64_t rxTimestamp);
    void sendTransferFrame(uint8_t destination, int timestamp);
 
    inline float calibratedDistance(uint8_t destination);
 
    /**
     * These two functions correct the timestamps if the counter had an overflow between measurements
     */
    void correctReceiverTimestamps(uint8_t source);
    void correctSenderTimestamps(uint8_t source);
 
    int timediffRec;
    int timediffSend;
 
    enum FrameType{
        PING=1,
        ANCHOR_RESPONSE,
        BEACON_RESPONSE,
        TRANSFER_FRAME,
        DISTANCES_FRAME
    };
 
    //the packed attribute makes sure the types only use their respective size in memory (8 bit for uint8_t), otherwise they would always use 32 bit
    //IT IS A GCC SPECIFIC DIRECTIVE
    struct __attribute__((packed, aligned(1))) RangingFrame {
        uint8_t source;
        uint8_t destination;
        uint8_t type;
    };
 
    struct __attribute__((packed, aligned(1))) ExtendedRangingFrame : RangingFrame{
        int signedTime;
    };
 
 
    RangingFrame rangingFrame;                  // buffer in class for sending a frame (not made locally because then we can recall in the interrupt what was sent)
    ExtendedRangingFrame transferFrame;
    ExtendedRangingFrame receivedFrame;
    uint64_t rxTimestamp;
    uint64_t senderTimestamps[10][3];
    uint64_t receiverTimestamps[10][3];
    bool acknowledgement[10];                   // flag to indicate if ranging has succeeded
    uint32_t tofs[10];                          // Array containing time of flights for each node (index is address of node)
 
};
 
#endif
 
