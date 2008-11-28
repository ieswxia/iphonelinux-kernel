/*
 *  arch/arm/mach-apple_iphone/usb.h
 *
 *  Copyright (C) 2008 Yiduo Wang
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef IPHONE_USB_H
#define IPHONE_USB_H

// assigned by USB Org
#define VENDOR_APPLE 0x5AC

// assigned by Apple
#define PRODUCT_IPHONE 0x1280
#define DEVICE_IPHONE 0x1103

// values we're using
#define USB_MAX_PACKETSIZE 64
#define USB_SETUP_PACKETS_AT_A_TIME 1
#define CONTROL_SEND_BUFFER_LEN 0x80
#define CONTROL_RECV_BUFFER_LEN 0x80
#define TX_QUEUE_LEN 0x80

// one packet at a time
#define USB_MULTICOUNT 1

#define OPENIBOOT_INTERFACE_CLASS 0xFF
#define OPENIBOOT_INTERFACE_SUBCLASS 0xFF
#define OPENIBOOT_INTERFACE_PROTOCOL 0x51

#define USB_LANGID_ENGLISH_US 0x0409

#define USBError 0xEEE

typedef enum USBState {
	USBStart = 0,
	USBPowered = 1,
	USBDefault = 2,
	USBAddress = 3,
	USBConfigured = 4,

	// Values higher than USBError(0xEEE) are error conditions
	USBUnknownDescriptorRequest = 0xEEE,
	USBUnknownRequest = 0xEEF
} USBState;

typedef enum USBDirection {
	USBOut = 0,
	USBIn = 1,
	USBBiDir = 2
} USBDirection;

typedef enum USBTransferType {
	USBControl = 0,
	USBIsochronous = 1,
	USBBulk = 2,
	USBInterrupt = 3
} USBTransferType;

typedef enum USBSynchronisationType {
	USBNoSynchronization = 0,
	USBAsynchronous = 1,
	USBAdaptive = 2,
	USBSynchronous = 3
} USBSynchronisationType;

typedef enum USBUsageType {
	USBDataEndpoint = 0,
	USBFeedbackEndpoint = 1,
	USBExplicitFeedbackEndpoint = 2
} USBUsageType;

enum USBDescriptorType {
	USBDeviceDescriptorType = 1,
	USBConfigurationDescriptorType = 2,
	USBStringDescriptorType = 3,
	USBInterfaceDescriptorType = 4,
	USBEndpointDescriptorType = 5,
	USBDeviceQualifierDescriptorType = 6
};

typedef void (*USBEndpointHandler)(u32 token);

typedef struct USBEndpointHandlerInfo {
	USBEndpointHandler	handler;
	u32		token;
} USBEndpointHandlerInfo;

typedef struct USBEndpointBidirHandlerInfo {
	USBEndpointHandlerInfo in;
	USBEndpointHandlerInfo out;
} USBEndpointBidirHandlerInfo;

typedef struct USBEPRegisters {
	volatile u32 control;
	volatile u32 field_4;
	volatile u32 interrupt;
	volatile u32 field_8;
	volatile u32 transferSize;
	volatile void* dmaAddress;
	volatile u32 field_18;
	volatile u32 field_1C;
} USBEPRegisters;

typedef struct USBDeviceDescriptor {
	u8 bLength;
	u8 bDescriptorType;
	u16 bcdUSB;
	u8 bDeviceClass;
	u8 bDeviceSubClass;
	u8 bDeviceProtocol;
	u8 bMaxPacketSize;
	u16 idVendor;
	u16 idProduct;
	u16 bcdDevice;
	u8 iManufacturer;
	u8 iProduct;
	u8 iSerialNumber;
	u8 bNumConfigurations;
} __attribute__ ((__packed__)) USBDeviceDescriptor;

typedef struct USBConfigurationDescriptor {
	u8 bLength;
	u8 bDescriptorType;
	u16 wTotalLength;
	u8 bNumInterfaces;
	u8 bConfigurationValue;
	u8 iConfiguration;
	u8 bmAttributes;
	u8 bMaxPower;
} __attribute__ ((__packed__)) USBConfigurationDescriptor;

typedef struct USBInterfaceDescriptor {
	u8 bLength;
	u8 bDescriptorType;
	u8 bInterfaceNumber;
	u8 bAlternateSetting;
	u8 bNumEndpoints;
	u8 bInterfaceClass;
	u8 bInterfaceSubClass;
	u8 bInterfaceProtocol;
	u8 iInterface;
} __attribute__ ((__packed__)) USBInterfaceDescriptor;

typedef struct USBEndpointDescriptor {
	u8 bLength;
	u8 bDescriptorType;
	u8 bEndpointAddress;
	u8 bmAttributes;
	u16 wMaxPacketSize;
	u8 bInterval;
} __attribute__ ((__packed__)) USBEndpointDescriptor;

typedef struct USBDeviceQualifierDescriptor {
	u8 bLength;
	u8 bDescriptorType;
	u16 bcdUSB;
	u8 bDeviceClass;
	u8 bDeviceSubClass;
	u8 bDeviceProtocol;
	u8 bMaxPacketSize;
	u8 bNumConfigurations;
	u8 bReserved;
} __attribute__ ((__packed__)) USBDeviceQualifierDescriptor;

typedef struct USBStringDescriptor {
	u8 bLength;
	u8 bDescriptorType;
	char bString[];
} __attribute__ ((__packed__)) USBStringDescriptor;

typedef struct USBFirstStringDescriptor {
	u8 bLength;
	u8 bDescriptorType;
	u16 wLANGID[];
} __attribute__ ((__packed__)) USBFirstStringDescriptor;

typedef struct USBInterface {
	USBInterfaceDescriptor descriptor;
	USBEndpointDescriptor* endpointDescriptors;
} USBInterface;

typedef struct USBConfiguration {
	USBConfigurationDescriptor descriptor;
	USBInterface* interfaces;
} USBConfiguration;

typedef struct USBSetupPacket {
	u8 bmRequestType;
	u8 bRequest;
	u16 wValue;
	u16 wIndex;
	u16 wLength;
} __attribute__ ((__packed__)) USBSetupPacket;

typedef void (*USBStartHandler)(void);
typedef void (*USBEnumerateHandler)(USBInterface* interface);

#define OPENIBOOTCMD_DUMPBUFFER 0
#define OPENIBOOTCMD_DUMPBUFFER_LEN 1
#define OPENIBOOTCMD_DUMPBUFFER_GOAHEAD 2
#define OPENIBOOTCMD_SENDCOMMAND 3
#define OPENIBOOTCMD_SENDCOMMAND_GOAHEAD 4

typedef struct OpenIBootCmd {
	u32 command;
	u32 dataLen;
}  __attribute__ ((__packed__)) OpenIBootCmd;

#define USBSetupPacketRequestTypeDirection(x) GET_BITS(x, 7, 1)
#define USBSetupPacketRequestTypeType(x) GET_BITS(x, 5, 2)
#define USBSetupPacketRequestTypeRecpient(x) GET_BITS(x, 0, 5)

#define USBSetupPacketHostToDevice 0
#define USBSetupPacketDeviceToHost 1
#define USBSetupPacketStandard 0
#define USBSetupPacketClass 1
#define USBSetupPacketVendor 2
#define USBSetupPacketRecpientDevice 0
#define USBSetupPacketRecpientInterface 1
#define USBSetupPacketRecpientEndpoint 2
#define USBSetupPacketRecpientOther 3

#define USB_CLEAR_FEATURE 1
#define USB_GET_CONFIGURATION 8
#define USB_GET_DESCRIPTOR 6
#define USB_GET_INTERFACE 10
#define USB_GET_STATUS 0
#define USB_SET_ADDRESS 5
#define USB_SET_CONFIGURATION 9
#define USB_SET_DESCRIPTOR 7
#define USB_SET_FEATURE 3
#define USB_SET_INTERFACE 11
#define USB_SYNCH_FRAME 12

#endif

