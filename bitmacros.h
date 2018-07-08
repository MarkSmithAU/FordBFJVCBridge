/*
(c) Mark Smith 2018
GPL v3
Not licensed for commercial use
*/

#define _setBit(port, bitnum) {(port) |= (1<<(bitnum));}

#define _setNamedBit(name) {(name##_PORT_WRITE) |= (1<<(name##_PIN));}
#define _clrNamedBit(name) {(name##_PORT_WRITE) &= ~(1<<(name##_PIN));}
#define _movNamedBit(name, value) {if ((value) != 0) {_setNamedBit(name);} else {_clrNamedBit(name);}}
#define _getNamedBit(name) (!((name##_PORT_READ) & (1<<(name##_PIN))) ? 0 : !0)

// ddr cleared (no write), out cleared (no pullup)
#define _setNamedBitNoPullUp(name) {(name##_PORT_DDR) &= ~(1<<(name##_PIN));	(name##_PORT_WRITE) &= ~(1<<(name##_PIN));}
// out to zero, ddr set (clamp to ground)
#define _clrNamedBitNoPullUp(name) {(name##_PORT_WRITE) &= ~(1<<(name##_PIN));      (name##_PORT_DDR) |= (1<<(name##_PIN));}
#define _movNamedBitNoPullUp(name, value) {if ((value) != 0) {_setNamedBitNoPullUp(name);} else {_clrNamedBitNoPullUp(name);}}
