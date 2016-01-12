typedef struct
{
  char cmd;
  uint32_t id = 0;
  uint8_t request[8];
  uint8_t requestLength = 0;
  uint8_t reply[8]; 
  uint8_t replyLength = 0;
} COMMAND;

typedef struct
{
  // the from-address of the responding device
  uint16_t id = -1;       
  // the to-address of the device to send the request to
  uint16_t requestId = -1;
  uint16_t length = 0;
  uint16_t index = 0;
  uint8_t next = 1;
  uint8_t request[8];
  uint8_t requestLength = 0;
  uint8_t reply[8]; 
  uint8_t replyLength = 0;
  uint8_t* data = 0; 
} ISO_MESSAGE;

