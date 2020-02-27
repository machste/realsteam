#ifndef LOC_MESSAGE_H
#define LOC_MESSAGE_H

typedef enum {
  SERVO_MESSAGE,
  INFO_REQUEST,
  INFO_RESPONSE
} MessageType;

struct ServoMessage
{
  ServoMessage() : type(SERVO_MESSAGE) {};
  MessageType type;
  int index;
  int value;
};

struct InfoRequest
{
  InfoRequest() : type(INFO_REQUEST) {};
  MessageType type;
};

struct InfoResponse
{
  InfoResponse() : type(INFO_RESPONSE) {};
  MessageType type;
  int ambient_temp;
  int pressure;
};

#endif /* LOC_MESSAGE_H */
