/* IRLib_P13_MilesTag.h
 * Part of IRLib Library for Arduino receiving, decoding, and sending
 * infrared signals. See COPYRIGHT.txt and LICENSE.txt for more information.
 */
/*
 * Implements decoding for the MilesTag protocol v2 used by MilesTag based DIY laser 
 * tag systems (http://www.lasertagparts.com/mtformat-2.htm). The protocol is a modified 
 * version of the Sony protocol, meaning it uses variable length mark and fixed length spaces. 
 *
 * The protocol allows 2 basic types of packets, a "Shot" packet, consisting of a header
 * pulse and 14 bits or a "Message" packet, consisting of a header pulse and 24 bits. 
 * There is a special message packet for cloning data and other system messages that allows
 * an arbitrary number of additional bytes but support for that is not implemented here.
 *
 * The protocol allows using any of 38, 40 or 56KHz as the modulated frequency. I've chosen
 * to implement this by default 38KHz as that seems to be the most common receiver used by Arduino
 * projects. As a future endeavor, it might be worth trying to set this up to handle a runtime
 * selectable frequency.
 * 
 * The protocol specifies a 2400us header, followed by a 600us space, followed by the data bits
 * where a mark of 1200us is a 1 bit, and a mark of 600us is a 0 bite 
 */

#ifndef IRLIB_PROTOCOL_13_H
#define IRLIB_PROTOCOL_13_H
#define IR_SEND_PROTOCOL_13		case 13: IRsendMilesTag::send(data,data2,khz); break;
#define IR_DECODE_PROTOCOL_13	if(IRdecodeMilesTag::decode()) return true;
#ifdef IRLIB_HAVE_COMBO
	#define PV_IR_DECODE_PROTOCOL_13 ,public virtual IRdecodeMilesTag
	#define PV_IR_SEND_PROTOCOL_13   ,public virtual IRsendMilesTag
#else
	#define PV_IR_DECODE_PROTOCOL_13  public virtual IRdecodeMilesTag
	#define PV_IR_SEND_PROTOCOL_13    public virtual IRsendMilesTag
#endif

#ifdef IRLIBSENDBASE_H
class IRsendMilesTag: public virtual IRsendBase {
  public:
    void send(uint32_t data, uint8_t nbits, uint8_t kHz=38) {
    	//Unlike the standard Sony protocol, we only send the data once
       sendGeneric(data,nbits, 600*4, 600, 600*2, 600, 600, 600, kHz, false); 
    }
};
#endif  //IRLIBSENDBASE_H

#ifdef IRLIBDECODEBASE_H
class IRdecodeMilesTag: public virtual IRdecodeBase {
  public:
    virtual bool decode(void) {
      IRLIB_ATTEMPT_MESSAGE(F("MilesTag"));
      resetDecoder();//This used to be in the receiver getResults.
      //if(recvGlobal.decodeLength!=2*14+2 && recvGlobal.decodeLength!=2*24+2) return RAW_COUNT_ERROR;
      if(!ignoreHeader) {
        if (!MATCH(recvGlobal.decodeBuffer[1],600*4)) return HEADER_MARK_ERROR(600*4);
      }
      offset=2;//skip initial gap plus header Mark.
      while (offset < recvGlobal.decodeLength) {
        if (!MATCH(recvGlobal.decodeBuffer[offset],600)) return DATA_SPACE_ERROR(600);
        offset++;
        if (MATCH(recvGlobal.decodeBuffer[offset],600*2)) {
          value = (value << 1) | 1;
        } 
        else if (MATCH(recvGlobal.decodeBuffer[offset],600)) {
          value <<= 1;
        } 
        else return DATA_MARK_ERROR(600);
        offset++;
      }
      bits = (offset - 1) / 2;
      protocolNum = MILESTAG;
      return true;
    }
};
#endif //IRLIBDECODEBASE_H

#define IRLIB_HAVE_COMBO

#endif //IRLIB_PROTOCOL_13_H
