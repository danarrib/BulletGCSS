/*
  msp_library.cpp

  This is a MSPv2 Library, made for Bullet Ground Control Station System
  https://github.com/danarrib/BulletGCSS
  
  Written by Daniel Ribeiro - https://github.com/danarrib
  
  With the valuable help of:
    Michel Pastor - https://github.com/shellixyz
    Paweł Spychalski - https://github.com/DzikuVx
    Olivier C. - https://github.com/OlivierC-FR

* This program is free software: you can redistribute it and/or modify  
* it under the terms of the GNU General Public License as published by  
* the Free Software Foundation, version 3.
*
* This program is distributed in the hope that it will be useful, but 
* WITHOUT ANY WARRANTY; without even the implied warranty of 
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License 
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Arduino.h"
#include "msp_library.h"

void MSPLibrary::begin(Stream &stream, uint32_t timeout)
{
  _stream = &stream;
  _timeout = timeout;
}

void MSPLibrary::reset()
{
  _stream->flush();
  while (_stream->available() > 0)
    _stream->read();
}

uint8_t MSPLibrary::crc8_dvb_s2(uint8_t crc, byte a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

void MSPLibrary::send(uint16_t messageID, void * payload, uint16_t size)
{
  uint8_t _crc = 0;
  uint8_t message[size + 9];
  message[0] = '$';
  message[1] = 'X';
  message[2] = '<';
  message[3] = 0; //flag
  message[4] = messageID; //function
  message[5] = messageID >> 8;
  message[6] = size; //payload size
  message[7] = size >> 8;
  for(uint8_t i = 3; i < 8; i++) {
    _crc = crc8_dvb_s2(_crc, message[i]);
  }
  //Start of Payload
  uint8_t * payloadPtr = (uint8_t*)payload;
  for (uint16_t i = 0; i < size; ++i) {
    message[i + 8] = *(payloadPtr++);
    _crc = crc8_dvb_s2(_crc, message[i + 8]);
  }
  message[size + 8] = _crc;
  _stream->write(message, sizeof(message));
}

bool MSPLibrary::recv(uint16_t * messageID, void * payload, uint16_t maxSize, uint16_t * recvSize)
{
  uint32_t t0 = millis();
  uint8_t _crc = 0;
  
  while (1) {
    // read header
    while (_stream->available() < 3)
      if (millis() - t0 >= _timeout)
        return false;
    char header[3];
    _stream->readBytes((char*)header, 3);

    // check header
    if (header[0] == '$' && header[1] == 'X' && header[2] == '>') {
      while (_stream->available() < 5)
        if (millis() - t0 >= _timeout)
          return false;

      uint8_t flag = _stream->read();
      _stream->readBytes((char*)messageID, 2);
      _stream->readBytes((char*)recvSize, 2);

      // Anything after the header should be accounted on the checksum.
      _crc = crc8_dvb_s2(_crc, flag);
      _crc = crc8_dvb_s2(_crc, *messageID);
      _crc = crc8_dvb_s2(_crc, *messageID >> 8);
      _crc = crc8_dvb_s2(_crc, *recvSize);
      _crc = crc8_dvb_s2(_crc, *recvSize >> 8);
      
      // read payload
      uint8_t * payloadPtr = (uint8_t*)payload;
      uint16_t idx = 0;
      while (idx < *recvSize) {
        if (millis() - t0 >= _timeout)
          return false;

        if (_stream->available() > 0) {
          uint8_t b = _stream->read();
          _crc = crc8_dvb_s2(_crc, b);
          if (idx < maxSize)
            *(payloadPtr++) = b;
          ++idx;
        }
      }

      // zero remaining bytes if *size < maxSize
      for (; idx < maxSize; ++idx)
        *(payloadPtr++) = 0;

      // Wait until there's nothing left to be get from stream
      while (_stream->available() == 0)
        if (millis() - t0 >= _timeout)
          return false;

      // Now get the checksum
      uint8_t checksum = _stream->read();

      if(_crc != checksum)
      {
        Serial.printf("Checksum failed. Calculated: %d - Retrieved: %d.\n", _crc, checksum);
        Serial.printf("MessageID: %d - MaxSize: %d - Retrieved Size %d.\n", *messageID, maxSize, *recvSize);
        //return false;
      }
      
      return true;
    }
  }
}

bool MSPLibrary::waitFor(uint16_t messageID, void * payload, uint16_t maxSize, uint16_t * recvSize)
{
  uint16_t recvMessageID;
  uint16_t recvSizeValue;
  uint32_t t0 = millis();
  while (millis() - t0 < _timeout)
    if (recv(&recvMessageID, payload, maxSize, (recvSize ? recvSize : &recvSizeValue)) && messageID == recvMessageID)
      return true;

  return false;
}

bool MSPLibrary::command(uint16_t messageID, void * payload, uint16_t size, bool waitACK)
{
  send(messageID, payload, size);

  // ack required
  if (waitACK)
    return waitFor(messageID, NULL, 0);

  return true;
}

bool MSPLibrary::request(uint16_t messageID, void *payload, uint16_t maxSize, uint16_t *recvSize)
{
  send(messageID, NULL, 0);
  return waitFor(messageID, payload, maxSize, recvSize);
}

bool MSPLibrary::requestWithPayload(uint16_t messageID, void *payload, uint16_t maxSize, uint16_t *recvSize)
{
  send(messageID, payload, maxSize);
  return waitFor(messageID, payload, maxSize, recvSize);
}

bool MSPLibrary::recvText(uint16_t * messageID, void * dst, uint16_t *recvSize)
{
  uint32_t t0 = millis();
  
  while (1) {
    // read header
    while (_stream->available() < 3)
      if (millis() - t0 >= _timeout)
        return false;
    char header[3];
    _stream->readBytes((char*)header, 3);

    // check header
    if (header[0] == '$' && header[1] == 'X' && header[2] == '>') {
      while (_stream->available() < 5)
        if (millis() - t0 >= _timeout)
          return false;

      uint8_t flag = _stream->read();
      _stream->readBytes((char*)messageID, 2);
      _stream->readBytes((char*)recvSize, 2);

      // read payload
      uint16_t idx = 0;
      char payload[*recvSize];
      uint8_t * payloadPtr = (uint8_t*)payload;
      while (idx < *recvSize) {
        if (millis() - t0 >= _timeout)
          return false;

        if (_stream->available() > 0) {
          uint8_t b = _stream->read();
          if (idx < *recvSize)
            payload[idx] = b;
          ++idx;
        }
      }

      // Copy the text to the dst so it can be accessed outside
      memcpy(dst, payload, *recvSize);
      
      // Wait until there's nothing left to be get from stream
      while (_stream->available() == 0)
        if (millis() - t0 >= _timeout)
          return false;

      // The last byte of the message is a checksum byte. We need to read it so we can discart from buffer.
      uint8_t checksum = _stream->read();

      return true;
    }
  }
}

bool MSPLibrary::waitForText(uint16_t messageID, void * payload, uint16_t *recvSize)
{
  uint16_t recvMessageID;
  uint16_t recvSizeValue;
  uint32_t t0 = millis();
  while (millis() - t0 < _timeout)
    if (recvText(&recvMessageID, payload, recvSize) && messageID == recvMessageID)
      return true;

  return false;
}

bool MSPLibrary::requestText(uint16_t messageID, void * payload, uint16_t *recvSize)
{
  send(messageID, NULL, 0);
  return waitForText(messageID, payload, recvSize);
}
