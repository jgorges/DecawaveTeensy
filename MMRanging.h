// by Matthias Grob & Manuel Stalder - ETH Zürich - 2015
 
#ifndef MMRANGING_H
#define MMRANGING_H
 
#include "arduino.h"
#include "DW1000.h"
 
#define MMRANGING_TIMEUNIT_US       1/(128*499.2)               // conversion between LSB of TX and RX timestamps and microseconds
#define MMRANGING_TIMEUNIT_NS       1000/(128*499.2)            // conversion between LSB of TX and RX timestamps and nanoseconds
 
#define MMRANGING_2POWER40          1099511627776               // decimal value of 2^40 to correct timeroverflow between timestamps
 
//#define EVENTS                                                // to see debug output of occurring interrupt callbacks
 
class MMRanging {
    public:
        MMRanging(DW1000& DW);
        void requestRanging(uint8_t destination);
        void requestRangingAll();
    //private:
        DW1000& dw;
        //Timer LocalTimer;
        
        void callbackRX();
        void callbackTX();
        void sendRangingframe(uint8_t destination, uint8_t sequence_number, uint8_t type, uint64_t time_difference_receiver);
        uint64_t timeDifference40Bit(uint64_t early, uint64_t late); // Method to calculate the difference between two 40-Bit timestamps correcting timer overflow reset occurring between the timestamps
        
        uint8_t address;                        // Identifies the nodes as source and destination in rangingframes
        //struct __attribute__((packed, aligned(1))) rangingframe {         // TODO: avoid structure padding, 32-Bit enough for time_difference_receiver  =>  8 Byte per frame instead of 16
        struct rangingframe {
            uint8_t source;
            uint8_t destination;
            uint8_t sequence_number;
            uint8_t type;
            uint64_t time_difference_receiver;
        };
        
        rangingframe TX;                        // buffer in class for sending a frame (not made locally because then we can recall in the interrupt what was sent)
        uint64_t rangingtimingsSender[10][2];
        uint64_t rangingtimingsReceiver[10][2];
        bool acknowledgement[10];               // flag to indicate if ranging has succeeded
        uint64_t tofs[10];                      // Array containing time of flights for each node (index is address of node)
        float roundtriptimes[10];               // Array containing the round trip times to the anchors or the timeout which occured
        float distances[10];                    // Array containing the finally calculated Distances to the anchors
        
        // draft for first test
        bool receiver;
        int event_i;
        char event[10][20];
        uint8_t counter;
};
 
#endif
            
