// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       SimpleRadio.ino
    Created:	28/11/2018 23.46.06
    Author:     RAHR\michael
*/

// Define User Types below here or use a .h file
//
#include "SimpleRadio.h"
#include <nrf51.h>
#include <nrf51_bitfields.h>


// Define Function Prototypes that use User Types below here or use a .h file
//

//Power levels for the RADIO
const int8_t MICROBIT_BLE_POWER_LEVEL[] = { -30, -20, -16, -12, -8, -4, 0, 4 };

// Define Functions below here or use other .ino or cpp files
//

//For the interrupt routine to work, it must be forward decleared in the header file
//This is only valid for VisualStudio, works fine in the Arduino IDE
extern "C" void RADIO_IRQHandler(void)
{
	 digitalWrite(LED, HIGH);
	 if (NRF_RADIO->EVENTS_READY)
	{
		NRF_RADIO->EVENTS_READY = 0;

		// Start listening and wait for the END event
		NRF_RADIO->TASKS_START = 1;
	}

	if (NRF_RADIO->EVENTS_END)
	{
		NRF_RADIO->EVENTS_END = 0;
		if (NRF_RADIO->CRCSTATUS == 1)
		{
			int sample = (int)NRF_RADIO->RSSISAMPLE;

			// Associate this packet's rssi value with the data just
			// transferred by DMA receive
			setRSSI(-sample);

			// Now move on to the next buffer, if possible.
			// The queued packet will get the rssi value set above.
			queueRxBuf();

			// Set the new buffer for DMA
			NRF_RADIO->PACKETPTR = (uint32_t)getRxBuf();
		}
		else
		{
			//MicroBitRadio::instance->setRSSI(0);
		}

		// Start listening and wait for the END event
		NRF_RADIO->TASKS_START = 1;
	}
	digitalWrite(LED, LOW);
}





/**
  * Retrieve a pointer to the currently allocated receive buffer. This is the area of memory
  * actively being used by the radio hardware to store incoming data.
  *
  * @return a pointer to the current receive buffer.
  */
FrameBuffer* getRxBuf()
{
	return rxBuf;
}






/**
  * Sets the RSSI for the most recent packet.
  * The value is measured in -dbm. The higher the value, the stronger the signal.
  * Typical values are in the range -42 to -128.
  *
  * @param rssi the new rssi value.
  *
  * @note should only be called from RADIO_IRQHandler...
  */
int setRSSI(int rssi)
{
	if (!(status & MICROBIT_RADIO_STATUS_INITIALISED))
		return MICROBIT_NOT_SUPPORTED;

	rssi = rssi;

	return MICROBIT_OK;
}

/**
  * Retrieves the current RSSI for the most recent packet.
  * The return value is measured in -dbm. The higher the value, the stronger the signal.
  * Typical values are in the range -42 to -128.
  *
  * @return the most recent RSSI value or MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int getRSSI()
{
	if (!(status & MICROBIT_RADIO_STATUS_INITIALISED))
		return MICROBIT_NOT_SUPPORTED;

	return rssi;
}


/**
  * Sets the radio to listen to packets sent with the given group id.
  *
  * @param group The group to join. A micro:bit can only listen to one group ID at any time.
  *
  * @return MICROBIT_OK on success, or MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int setGroup(uint8_t group)
{
	
	// Record our group id locally
	group = group;

	// Also append it to the address of this device, to allow the RADIO module to filter for us.
	NRF_RADIO->PREFIX0 = (uint32_t)group;

	return MICROBIT_OK;
}




/**
  * Attempt to queue a buffer received by the radio hardware, if sufficient space is available.
  *
  * @return MICROBIT_OK on success, or MICROBIT_NO_RESOURCES if a replacement receiver buffer
  *         could not be allocated (either by policy or memory exhaustion).
  */
int queueRxBuf()
{
	
	if (rxBuf == NULL)
		return MICROBIT_INVALID_PARAMETER;

	if (queueDepth >= MICROBIT_RADIO_MAXIMUM_RX_BUFFERS)
		return MICROBIT_NO_RESOURCES;

	// Store the received RSSI value in the frame
	rxBuf->rssi = getRSSI();

	// Ensure that a replacement buffer is available before queuing.
	FrameBuffer *newRxBuf = new FrameBuffer();

	if (newRxBuf == NULL)
		return MICROBIT_NO_RESOURCES;

	// We add to the tail of the queue to preserve causal ordering.
	rxBuf->next = NULL;

	if (rxQueue == NULL)
	{
		rxQueue = rxBuf;
	}
	else
	{
		FrameBuffer *p = rxQueue;
		while (p->next != NULL)
			p = p->next;

		p->next = rxBuf;
	}

	// Increase our received packet count
	queueDepth++;

	// Allocate a new buffer for the receiver hardware to use. the old on will be passed on to higher layer protocols/apps.
	rxBuf = newRxBuf;

	return MICROBIT_OK;
}

/**
  * Change the output power level of the transmitter to the given value.
  *
  * @param power a value in the range 0..7, where 0 is the lowest power and 7 is the highest.
  *
  * @return MICROBIT_OK on success, or MICROBIT_INVALID_PARAMETER if the value is out of range.
  */
int setTransmitPower(int power)
{
	if (power < 0 || power >= MICROBIT_BLE_POWER_LEVELS)
		return MICROBIT_INVALID_PARAMETER;

	NRF_RADIO->TXPOWER = (uint32_t)MICROBIT_BLE_POWER_LEVEL[power];

	return MICROBIT_OK;
}


/**
  * Initialises the radio for use as a multipoint sender/receiver
  *
  * @return MICROBIT_OK on success, MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int enable()
{
	// If the device is already initialised, then there's nothing to do.
	if (status & MICROBIT_RADIO_STATUS_INITIALISED)
		return MICROBIT_OK;
	Serial.println("Enable 1");
	// Only attempt to enable this radio mode if BLE is disabled.

	// If this is the first time we've been enable, allocate out receive buffers.
	if (rxBuf == NULL)
		rxBuf = new FrameBuffer();
	Serial.println("Enable 2");
	if (rxBuf == NULL)
		return MICROBIT_NO_RESOURCES;
	Serial.println("Enable 3");
	// Enable the High Frequency clock on the processor. This is a pre-requisite for
	// the RADIO module. Without this clock, no communication is possible.
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	Serial.println("Enable 4");
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
	Serial.println("Enable 5");
	// Bring up the nrf51822 RADIO module in Nordic's proprietary 1MBps packet radio mode.
	setTransmitPower(MICROBIT_RADIO_DEFAULT_TX_POWER);
	NRF_RADIO->FREQUENCY = MICROBIT_RADIO_DEFAULT_FREQUENCY;
	Serial.println("Enable 6");
	// Configure for 1Mbps throughput.
	// This may sound excessive, but running a high data rates reduces the chances of collisions...
	NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_1Mbit;
	Serial.println("Enable 7");
	// Configure the addresses we use for this protocol. We run ANONYMOUSLY at the core.
	// A 40 bit addresses is used. The first 32 bits match the ASCII character code for "uBit".
	// Statistically, this provides assurance to avoid other similar 2.4GHz protocols that may be in the vicinity.
	// We also map the assigned 8-bit GROUP id into the PREFIX field. This allows the RADIO hardware to perform
	// address matching for us, and only generate an interrupt when a packet matching our group is received.
	NRF_RADIO->BASE0 = MICROBIT_RADIO_BASE_ADDRESS;

	// Join the default group. This will configure the remaining byte in the RADIO hardware module.
	setGroup(group);
	Serial.println("Enable 8");
	// The RADIO hardware module supports the use of multiple addresses, but as we're running anonymously, we only need one.
	// Configure the RADIO module to use the default address (address 0) for both send and receive operations.
	NRF_RADIO->TXADDRESS = 0;
	NRF_RADIO->RXADDRESSES = 1;
	Serial.println("Enable 9");
	// Packet layout configuration. The nrf51822 has a highly capable and flexible RADIO module that, in addition to transmission
	// and reception of data, also contains a LENGTH field, two optional additional 1 byte fields (S0 and S1) and a CRC calculation.
	// Configure the packet format for a simple 8 bit length field and no additional fields.
	NRF_RADIO->PCNF0 = 0x00000008;
	NRF_RADIO->PCNF1 = 0x02040000 | MICROBIT_RADIO_MAX_PACKET_SIZE;
	Serial.println("Enable 10");

	// Most communication channels contain some form of checksum - a mathematical calculation taken based on all the data
	// in a packet, that is also sent as part of the packet. When received, this calculation can be repeated, and the results
	// from the sender and receiver compared. If they are different, then some corruption of the data ahas happened in transit,
	// and we know we can't trust it. The nrf51822 RADIO uses a CRC for this - a very effective checksum calculation.
	//
	// Enable automatic 16bit CRC generation and checking, and configure how the CRC is calculated.
	NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two;
	NRF_RADIO->CRCINIT = 0xFFFF;
	NRF_RADIO->CRCPOLY = 0x11021;
	Serial.println("Enable 11");

	// Set the start random value of the data whitening algorithm. This can be any non zero number.
	// Whitning xor a psudorandom noise, this should have a general positive effect on the radio signal.
	NRF_RADIO->DATAWHITEIV = 0x18;

	// Set up the RADIO module to read and write from our internal buffer.
	NRF_RADIO->PACKETPTR = (uint32_t)rxBuf;
	Serial.println("Enable 12");
	// Configure the hardware to issue an interrupt whenever a task is complete (e.g. send/receive).
	NRF_RADIO->INTENSET = 0x00000008;
	NVIC_ClearPendingIRQ(RADIO_IRQn);
	NVIC_EnableIRQ(RADIO_IRQn);
	Serial.println("Enable 13");
	NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;

	// Start listening for the next packet
	NRF_RADIO->EVENTS_READY = 0;
	NRF_RADIO->TASKS_RXEN = 1;
	while (NRF_RADIO->EVENTS_READY == 0);
	Serial.println("Enable 14");

	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_START = 1;

	Serial.println("Enable 15");

	// Done. Record that our RADIO is configured.
	status |= MICROBIT_RADIO_STATUS_INITIALISED;
	Serial.println("Enable 16");

	return MICROBIT_OK;
}

/**
  * Disables the radio for use as a multipoint sender/receiver.
  *
  * @return MICROBIT_OK on success, MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int disable()
{
	
	if (!(status & MICROBIT_RADIO_STATUS_INITIALISED))
		return MICROBIT_OK;

	// Disable interrupts and STOP any ongoing packet reception.
	NVIC_DisableIRQ(RADIO_IRQn);

	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_DISABLE = 1;
	while (NRF_RADIO->EVENTS_DISABLED == 0);

	
	// record that the radio is now disabled
	status &= ~MICROBIT_RADIO_STATUS_INITIALISED;

	return MICROBIT_OK;
}

/**
  * Determines the number of packets ready to be processed.
  *
  * @return The number of packets in the receive buffer.
  */
int dataReady()
{
	return queueDepth;
}

/**
  * Retrieves the next packet from the receive buffer.
  * If a data packet is available, then it will be returned immediately to
  * the caller. This call will also dequeue the buffer.
  *
  * @return The buffer containing the the packet. If no data is available, NULL is returned.
  *
  * @note Once recv() has been called, it is the callers responsibility to
  *       delete the buffer when appropriate.
  */
FrameBuffer* recv()
{
	FrameBuffer *p = rxQueue;

	if (p)
	{
		// Protect shared resource from ISR activity
		NVIC_DisableIRQ(RADIO_IRQn);

		rxQueue = rxQueue->next;
		queueDepth--;

		// Allow ISR access to shared resource
		NVIC_EnableIRQ(RADIO_IRQn);
	}

	return p;
}

/**
  * Transmits the given buffer onto the broadcast radio.
  * The call will wait until the transmission of the packet has completed before returning.
  *
  * @param data The packet contents to transmit.
  *
  * @return MICROBIT_OK on success, or MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int send(FrameBuffer *buffer)
{


	if (buffer == NULL)
		return MICROBIT_INVALID_PARAMETER;

	if (buffer->length > MICROBIT_RADIO_MAX_PACKET_SIZE + MICROBIT_RADIO_HEADER_SIZE - 1)
		return MICROBIT_INVALID_PARAMETER;

	// Firstly, disable the Radio interrupt. We want to wait until the trasmission completes.
	NVIC_DisableIRQ(RADIO_IRQn);

	// Turn off the transceiver.
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_DISABLE = 1;
	while (NRF_RADIO->EVENTS_DISABLED == 0);

	// Configure the radio to send the buffer provided.
	NRF_RADIO->PACKETPTR = (uint32_t)buffer;

	// Turn on the transmitter, and wait for it to signal that it's ready to use.
	NRF_RADIO->EVENTS_READY = 0;
	NRF_RADIO->TASKS_TXEN = 1;
	while (NRF_RADIO->EVENTS_READY == 0);

	// Start transmission and wait for end of packet.
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_START = 1;
	while (NRF_RADIO->EVENTS_END == 0);

	// Return the radio to using the default receive buffer
	NRF_RADIO->PACKETPTR = (uint32_t)rxBuf;

	// Turn off the transmitter.
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_DISABLE = 1;
	while (NRF_RADIO->EVENTS_DISABLED == 0);

	// Start listening for the next packet
	NRF_RADIO->EVENTS_READY = 0;
	NRF_RADIO->TASKS_RXEN = 1;
	while (NRF_RADIO->EVENTS_READY == 0);

	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_START = 1;

	// Re-enable the Radio interrupt.
	NVIC_ClearPendingIRQ(RADIO_IRQn);
	NVIC_EnableIRQ(RADIO_IRQn);

	return MICROBIT_OK;
}


// The setup() function runs once each time the micro-controller starts
void setup()
{
	id = 0;
	status = 0;
	group = MICROBIT_RADIO_DEFAULT_GROUP;
	queueDepth = 0;
	rssi = 0;
	rxQueue = NULL;
	rxBuf = NULL;
	Serial.begin(115200);

	pinMode(COL1, OUTPUT);
	digitalWrite(COL1, LOW);

	pinMode(LED, OUTPUT);

	// put your setup code here, to run once:
	//NVIC_DisableIRQ(RADIO_IRQn);
	//NRF_RADIO->EVENTS_DISABLED = 0;
	//NRF_RADIO->TASKS_DISABLE = 1;
	enable();
	myDataSendData = new FrameBuffer();
}




// Add the main program code into the continuous loop() function
void loop()
{
	int buffer = dataReady();
	static long currentMillis;
	

	delay(10);
	FrameBuffer* myData = recv();
	if (myData != NULL) {
		Serial.print(myData->length);
		Serial.print("    ");
		Serial.print(myData->version);
		Serial.print("    ");
		Serial.print(myData->group);
		Serial.print("    ");
		Serial.print(myData->protocol);
		Serial.print("    ");
		Serial.print(myData->payload[10]);
		Serial.print("    ");
		Serial.println(dataReady());
		delete myData;

		myDataSendData->length = 3;
		myDataSendData->group =   10;
		myDataSendData->version = 12;
		myDataSendData->protocol = 14;

		
		if (millis() - currentMillis >= interval)
		{
			
			Serial.println(currentMillis);
			send(myDataSendData);
			currentMillis = millis();
		}



	}
}
