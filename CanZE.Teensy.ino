
// load the CAN library
#include <FlexCAN.h>
// load some predefined structures
#include "structs.h"

// define things
#define Output Serial
#define Bluetooth Serial1
#define BT_SPEED 1311600
#define CAN_SPEED 500000
#define CAN_STRUCT CAN_message_t

#define SIM 1SIM

// create the CAN bus object
FlexCAN CANbus(CAN_SPEED);

// define the data array for the free frames
CAN_STRUCT* dataArray = 0;
int dataArraySize = 0;
CAN_STRUCT EMPTY;

// things we need for the incoming requests
String readBuffer = "";
String filter = "";
ISO_MESSAGE isoMessage;

// variables needed to count the number of messages per second
long unsigned count = 0;          // number of received messages
double totalRate = 0;             // reception rate based on millis()
double filteredRate = 0;          // reception rate based on millis()


boolean outputReceivedFrames = 0;

void setup(void)
{
  // init EMPTY frame
  EMPTY.len=8;
  for(int i=0; i<EMPTY.len; i++)
    EMPTY.buf[i]=0;
  
  // initialise the serial connection
  Output.begin(19200);

  // initialise the Bluetooth connection
  Bluetooth.begin(BT_SPEED);

  // initialise the CAN bus 
  CANbus.begin();
}


void loop(void)
{
  CAN_STRUCT incoming;
  
  // did we receive a frame?
  if(CANbus.read(incoming)) 
  {
      // process it
      processFrame(incoming);
  }

  // process any input on the BT line
  readIncoming();
}

void addFrame(CAN_STRUCT &frame)
{
  // create new array
  dataArraySize++;
  CAN_STRUCT *temp = new CAN_STRUCT [dataArraySize];
  // copy data
  for(int i=0; i<dataArraySize-1; i++)
    temp[i]=dataArray[i];
  // delete old array
  if (dataArray != 0) {
      delete [] dataArray;
  }
  // switch reference    
  dataArray = temp;
  // store new frame
  dataArray[dataArraySize-1]=frame;
}

int findFrameIndexById(uint32_t id)
{
  for(int i=0; i<dataArraySize; i++)
  {
    if(dataArray[i].id==id) return i;
  }
  return -1;
}

CAN_STRUCT getFrameById(int id)
{
  int index = findFrameIndexById(id);
  if (index==-1) return EMPTY;
  else return dataArray[index];
}

void storeFreeFrame(CAN_STRUCT &frame)
{
  int index = findFrameIndexById(frame.id);
  if(index==-1) addFrame(frame);
  else dataArray[index]=frame;  
}

void processFrame(CAN_STRUCT &frame)
{
  count++;
  totalRate = (count/(millis()/1000.));

  /*if((frame.id >= 0x700))
  {
    String dataString = frameToBOB(frame);
    Output.print(dataString);
  }*/
    
  // only process frame if not filtered out or no active filter
  String idHex = String(frame.id,HEX);
  if(filter.length()==0 || filter.indexOf(idHex)!=-1)
  {
    filteredRate = (count/(millis()/1000.));
  
    // free data stream is < 0x700
    if (frame.id < 0x700)
    {
      storeFreeFrame(frame);
    }
    // if there is content and this is the frame we are waiting for
    else if (frame.len > 0 && frame.id == isoMessage.id)
    {
      Output.print(frameToBOB(frame));
      
      // id = first nibble
      uint8_t type = frame.buf[0] >> 4;
      // single frame answer
      if (type == 0x0) {
        String dataString = frameToBOB(frame);
        dataString.trim();
        dataString += ",";
        for (int i = 0; i < isoMessage.replyLength; i++)
        {
          dataString += getHex(isoMessage.reply[i]);
        }
        dataString += "\r\n";
  
        Output.println(dataString);
        Bluetooth.print(dataString);
      }
      // first frame of a multi-framed message
      else if (type == 0x1) {
        // length = second nibble + second byte
        uint16_t messageLength = frame.buf[0] & 0x0f;
        messageLength <<= 8;
        messageLength |= frame.buf[1];

        // reply ID = third & fourth byte
        uint32_t frameReply = 0;
        uint32_t reply = 0;
        for (int i = 0; i < isoMessage.replyLength; i++)
        {
          frameReply <<= 16;
          frameReply |=  frame.buf[i + 2];
          reply <<= 16;
          reply |= isoMessage.reply[i];
        }
  
        // is this the message we are waiting for?
        if (frameReply == reply)
        {
          // request the remaining 0x2 frames
          CAN_STRUCT flow;
          flow.id = isoMessage.requestId;
          flow.len = 8;
          flow.buf[0] = 0x30;  // type 3, clear to send
          flow.buf[1] = 0x00;  // no flow control
          flow.buf[2] = 0x10;  // no delay between frames
          flow.buf[3] = 0; // fill-up
          flow.buf[4] = 0; // fill-up
          flow.buf[5] = 0; // fill-up
          flow.buf[6] = 0; // fill-up
          flow.buf[7] = 0; // fill-up
          CANbus.write(flow);
  
          // build new iso message
          // set id
          isoMessage.id = frame.id;
          // set final length
          isoMessage.length = messageLength;
          // init data
          uint8_t* data = new uint8_t[messageLength];
          for (int i = 0; i < messageLength; i++) data[i] = 0;
            isoMessage.data = data;
          // init sequence
          isoMessage.next = 1;
          // fill up data
          isoMessage.index = 0;
          for (int i = 2; i < frame.len; i++)
          {
            if (isoMessage.index < isoMessage.length)
            {
              isoMessage.data[isoMessage.index] = frame.buf[i];
              isoMessage.index++;
            }
          }
        }
      }
      // consecutive frames
      else if (type == 0x2)
      {
  
        uint16_t sequence = frame.buf[0] & 0x0f;
        if (isoMessage.id == frame.id &&
            isoMessage.next == sequence)
        {
          for (int i = 1; i < frame.len; i++)
          {
            if (isoMessage.index < isoMessage.length)
            {
              isoMessage.data[isoMessage.index] = frame.buf[i];
              isoMessage.index++;
            }
          }
  
          // wait for next message
          isoMessage.next = (isoMessage.next + 1) % 16;
  
          // is this the last part?
          if (isoMessage.index >= isoMessage.length) // for some frames one byte is missing :-?
          {
            String dataString = messageToBOB(isoMessage);
            Output.print(dataString);
            Bluetooth.print(dataString);
  
            // reset filters
            CAN_filter_t filter;
            CANbus.setFilter(filter,0);
  
            // cancel this message
            isoMessage.id = -1;
          }
        }
      } // 0x2 frame
    } // ISO frame
  } // filter
}

String frameToBOB(CAN_STRUCT &frame)
{
  String dataString = String(frame.id, HEX) + ",";
  for (int i = 0; i < frame.len; i++)
  {
    dataString += getHex(frame.buf[i]);
  }
  dataString += "\n";
  return dataString;
}

String messageToBOB(ISO_MESSAGE &message)
{
  String dataString = String(message.id, HEX) + ",";
  for (int i = 0; i < message.length; i++)
  {
    dataString += getHex(message.data[i]);
  }
  dataString += ",";
  for (int i = 0; i < message.replyLength; i++)
  {
    dataString += getHex(message.reply[i]);
  }
  dataString += "\n";
  return dataString;
}

void readIncoming()
{
  // Output
  if(Output.available())
  {
    char ch = Output.read();
    if(ch=='\n' || ch=='\r')
    {
      if(readBuffer!="")
      {
        processCommand(readBuffer.trim());
        readBuffer="";        
      }
    }
    else
    {
      readBuffer+=ch;
    }
  }
  
  // Bluetooth
  if(Bluetooth.available())
  {
    char ch = Bluetooth.read();
    if(ch=='\n' || ch=='\r')
    {
      if(readBuffer!="")
      {
        processCommand(readBuffer.trim());
        readBuffer="";        
      }
    }
    else
    {
      readBuffer+=ch;
    }
    
  }  
}

/* cmd    description                 example
 * ------------------------------------------
 * f      add filter                  f<ID>
 * r      remove filter               r<ID>
 * c      clear filters               c
 * a      output all captured frames  a
 * g      get free frame              g<ID>
 * i      send an ISO request         i<ID>,<request><reply>
 * t      test a frame                t<ID>,<data>
 * o      display set filters         o
 * s      get statistics              s
 */
void processCommand(String &line)
{
  Output.println(line);
  //if(line.trim()!="")
  {
    COMMAND command = decodeCommand(line);
    
    if(command.cmd=='f')
    {
      String id = getHexSimple(command.id);
      int pos = filter.indexOf(id);
      // only add if not yet there
      if(pos==-1)
        filter+=id+",";
    }
    // remove filter
    else if(command.cmd=='r')
    {
      String id = getHexSimple(command.id);
      int pos = filter.indexOf(id);
      // only remove if present
      if(pos!=-1)
        filter.remove(pos,id.length()+1);
    }
    // clear filter
    else if(command.cmd=='c')
    {
      filter="";
    }
    // all frames
    else if(command.cmd=='a')
    {
      Output.println("Framecount = "+String(dataArraySize));
      for(int i=0; i<dataArraySize; i++)
      {
        CAN_STRUCT frame = dataArray[i];
        String sendData = frameToOutput(frame);
        Output.print(sendData);
        Bluetooth.print(sendData);
      }
    }
    // get frame
    else if(command.cmd=='g')
    {
      CAN_STRUCT frame = getFrameById(command.id);
      
      #ifdef SIM
      // simulate
        frame.id=command.id;
        for(int i=0; i<8; i++)
          frame.buf[i]=random(0,255);
          /*//if(i%2==0)
          { 
            frame.buf[i]=(frame.buf[i]+random(0,5))%255;
            if(frame.buf[i]>0x0F)
              frame.buf[i]=(frame.buf[i]+random(-2,2))%255;
            //if(frame.buf[i]==0)
            //  frame.buf[i+1]=(frame.buf[i+1]+1)%255;
          }*/
        storeFreeFrame(frame);
      #endif
      
      String sendData = frameToOutput(frame);
      Output.print(sendData);
      Bluetooth.print(sendData);
    }
    // send a request and wait for the response
    else if(command.cmd=='i')
    {
       #ifdef SIM
        Output.print("nothing");
       #endif
       // only accept this command if the requested ID belongs to an ISO-TP frame
       if(command.id>0x700)
       {
          // store ID
          isoMessage.id=command.id;
          isoMessage.requestId = getRequestId(command.id);
          Serial.println("Request for "+getHex(command.id)+" -> "+getHex(isoMessage.requestId));
          // store reply
          isoMessage.replyLength=command.replyLength;
          for(int i=0; i<command.replyLength; i++)
            isoMessage.reply[i]=command.reply[i];
          // store request
          isoMessage.requestLength=command.requestLength;
          for(int i=0; i<command.requestLength; i++)
            isoMessage.request[i]=command.request[i];
  
          if(isoMessage.requestId>0)
          {
            // buidl the CAN frame
            CAN_STRUCT frame;
            // set the ID
            frame.id=isoMessage.requestId;
            // set the length
            frame.len=command.requestLength+1;
            frame.len=8;
            // zero out frame
            for(int i=0; i<8; i++)
              frame.buf[i]=0;
            // fill up the first byte 0x00 & requestLength
            frame.buf[0]=command.requestLength;
            // fill up the other bytes
            for(int i=0; i<command.requestLength; i++)
              frame.buf[i+1]=command.request[i];
            // activate filter
            //CAN_filter_t filter;
            //filter.id=command.id;
            //CANbus.setFilter(filter,0);
            // send the frame
            CANbus.write(frame);
            Output.print(frameToBOB(frame));
            // --> any incoming frames with the given id will be handeled by "processFrame" and send off if complete
          }
       }
    }
    // test a frame
    else if(command.cmd=='t')
    {
       CAN_STRUCT frame;
       frame.id=command.id;
       frame.len=command.requestLength;
       for(int i=0; i<command.requestLength; i++)
          frame.buf[i]=command.request[i];
       processFrame(frame);
    }
    // debug filters to output
    else if(command.cmd=='o')
    {
        Output.println("Filters: "+filter);
        Bluetooth.println("Filters: "+filter);
    }
    // get frame rate
    else if(command.cmd=='s')
    {
        Output.println("Frame Rate: "+String(totalRate));
        Bluetooth.println("Frame Rate: "+String(totalRate));
  
        Output.println("Filtered Frame Rate: "+String(filteredRate));
        Bluetooth.println("Filtered Frame Rate: "+String(filteredRate));
    }
    // toggle output
    else if(command.cmd=='l')
    {
        outputReceivedFrames=!outputReceivedFrames;
        Output.print("outputReceivedFrames is now ");
        if(outputReceivedFrames)
          Output.println("ON");
        else
          Output.println("OFF");
    }
  }
}

COMMAND decodeCommand(String &input)
{
  COMMAND result;

  // trim whitespaces
  input.trim();

  // stop if input is empty
  if(input.length()==0) return result;
  
  // the first letter is the command
  result.cmd = input.charAt(0);
  input.remove(0,1); 

  // if there is something more,
  if(input.length()!=0)
  {
    // get the ID
    char ch;
    String id = "";
    do
    {
      ch=input.charAt(0);
      if(ch!=',') id+=ch;
      input.remove(0,1); 
    }
    while(input.length()!=0 && ch!=',');
    result.id=hexToDec(id);
  }
  
  // if there is something more,
  if(input.length()!=0)
  {
      // get the REQUEST
      char ch;
      String request = "";
      do
      {
        ch=input.charAt(0);
        if(ch!=',') request+=ch;
        input.remove(0,1); 
      }
      while(input.length()!=0 && ch!=',');
      for(uint32_t i=0; i<request.length(); i+=2)
      {
        result.request[result.requestLength]=hexToDec(request.substring(i,i+2));
        result.requestLength++;
      }
  }

  // if there is something more,
  if(input.length()!=0)
  {
      // get the REPLY
      char ch;
      String reply = "";
      do
      {
        ch=input.charAt(0);
        if(ch!=',') reply+=ch;
        input.remove(0,1); 
      }
      while(input.length()!=0 && ch!=',');
      for(uint32_t i=0; i<reply.length(); i+=2)
      {
        result.reply[result.replyLength]=hexToDec(reply.substring(i,i+2));
        result.replyLength++;
      }
  }
  return result;
}

String frameToOutput(CAN_STRUCT &frame)
{
  String dataString = String(frame.id,HEX)+",";
  for(int i = 0; i<frame.len; i++) 
  {
    dataString+=getHex(frame.buf[i]);
  }
  dataString+="\r\n";
  return dataString;
}

String getHex(int num)
{
  String stringOne =  String(num, HEX); 
  if(stringOne.length()<2) stringOne="0"+stringOne;
  return stringOne;
}

String getHexSimple(int num)
{
  String stringOne =  String(num, HEX); 
  return stringOne;
}

unsigned int hexToDec(String hexString) {
  
  unsigned int decValue = 0;
  int nextInt;
  
  for (uint32_t i = 0; i < hexString.length(); i++) {
    
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    
    decValue = (decValue * 16) + nextInt;
  }
  
  return decValue;
}

uint16_t getRequestId(uint16_t responseId)
{                     //from        // to
    if     (responseId==0x7ec) return 0x7e4;  // EVC / SCH
    else if(responseId==0x7cd) return 0x7ca;  // TCU
    else if(responseId==0x7bb) return 0x79b;  // LBC
    else if(responseId==0x77e) return 0x75a;  // PEB
    else if(responseId==0x772) return 0x752;  // Airbag
    else if(responseId==0x76d) return 0x74d;  // USM / UDP
    else if(responseId==0x763) return 0x743;  // CLUSTER / instrument panel
    else if(responseId==0x762) return 0x742;  // PAS
    else if(responseId==0x760) return 0x740;  // ABS
    else if(responseId==0x7bc) return 0x79c;  // UBP
    else if(responseId==0x765) return 0x745;  // BCM
    else if(responseId==0x764) return 0x744;  // CLIM
    else if(responseId==0x76e) return 0x74e;  // UPA
    else if(responseId==0x793) return 0x792;  // BCB
    else if(responseId==0x7b6) return 0x796;  // LBC2
    else if(responseId==0x722) return 0x702;  // LINSCH
    else return -1;
}
