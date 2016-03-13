/*
 * Copyright (C) 2009  ENAC, Pascal Brisset
 * Copyright (C) 2014  Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file subsystems/datalink/xbee868.h
 * Configuration for 868MHz modules
 */

/* 
 Transmit Request (API 1)

7E 00 12 10 01 00 13 A2 00 40 F3 3B 4B FF FE 00 00 54 45 53 54 43

Start delimiter: 7E
Length: 00 12 (18)
Frame type: 10 (Transmit Request)
Frame ID: 01 (1)
64-bit dest. address: 00 13 A2 00 40 F3 3B 4B
16-bit dest. address: FF FE
Broadcast radius: 00 (0)
Options: 00
RF data: 54 45 53 54
Checksum: 43


Transmit Status (API 1)

7E 00 07 8B 01 FF FE 00 00 00 76

Start delimiter: 7E
Length: 00 07 (7)
Frame type: 8B (Transmit Status)
Frame ID: 01 (1)
16-bit dest. address: FF FE
Tx. retry count: 00 (0)
Delivery status: 00 (Success)
Discovery status: 00 (No discovery overhead)
Checksum: 76


RX:
Receive Packet (API 1)

7E 00 10  90 00 13 A2 00 40 F3 3B 44 FF FE C1 54 45 53 54 0A

Start delimiter: 7E
Length: 00 10 (16)
Frame type: 90 (Receive Packet)
64-bit source address: 00 13 A2 00 40 F3 3B 44
16-bit source address: FF FE
Receive options: C1
RF data: 54 45 53 54
Checksum: 0A
*/

#ifndef XBEE868_H
#define XBEE868_H

#define XBEE_TX_ID 0x10
#define XBEE_RX_ID 0x90

#define XBEE_TX_OVERHEAD 13
#define XBEE_RFDATA_OFFSET 12
#define XBEE_TX_HEADER { \
    XBEE_TX_ID, \
    NO_FRAME_ID, \
    0x00, \
    0x13, \
    0xA2, \
    0x00, \
    0x40, \
    0xF3, \
    (GROUND_STATION_ADDR >> 8), \
    (GROUND_STATION_ADDR & 0xff), \
    0xff, \
    0xfe, \
    0x00, \
    TX_OPTIONS \
  }

#define XbeeGetRSSI(_payload) {}

#endif // XBEE868_H
