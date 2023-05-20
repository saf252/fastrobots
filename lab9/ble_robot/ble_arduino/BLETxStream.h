#ifndef _BLE_TX_STREAM_H_
#define _BLE_TX_STREAM_H_

#include <ArduinoBLE.h>

static constexpr size_t MAX_MSG_LEN = 150;

class BLETxStream
{
  public:
    BLETxStream(BLECharacteristic* txCharacteristic)
      : _txCharacteristic(txCharacteristic)
      , _msgLength(1)
      , _maxMsgLength(min(txCharacteristic->valueSize(), MAX_MSG_LEN))
    {
      _msgBuffer[0] = 0;
    }

    void flush()
    {
      _msgBuffer[0] = 0;
      _txCharacteristic->writeValue(_msgBuffer, _msgLength);
      _msgLength = 1;
    }

    void write(const uint8_t* data, size_t length)
    {
      for (size_t i = 0; i < length; ++i)
      {
        if (_msgLength == _maxMsgLength)
        {
          _msgBuffer[0] += 1;
          _txCharacteristic->writeValue(_msgBuffer, _msgLength);
          _msgLength = 1;
        }
        _msgBuffer[_msgLength++] = data[i];
      }
    }

    void write(const char* str)
    {
      return write(str, strlen(str));
    }

    template<typename T>
    void write(const T& data)
    {
      return write((const uint8_t*) &data, sizeof(T));
    }

    template<typename T>
    void write(const T* data, size_t length)
    {
      return write((const uint8_t*) data, sizeof(T) * length);
    }

  private:
    BLECharacteristic* _txCharacteristic;
    uint8_t _msgBuffer[MAX_MSG_LEN];
    size_t _msgLength;
    size_t _maxMsgLength;
};

#endif // _BLE_TX_STREAM_H_