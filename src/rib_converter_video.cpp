/*=========================================================================

  Program:   Converter Class for Video
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include "rib_converter_video.h"
#include "rib_converter_manager.h"
#include "ros/ros.h"
#include "igtlImageMessage.h"

#if defined(OpenIGTLink_USE_VP9)
  #include "VP9Encoder.h"
  #include "VP9Decoder.h"
#endif

RIBConverterVideo::RIBConverterVideo()
  : RIBConverter<sensor_msgs::Image>()
{
  InitCodec();
}

RIBConverterVideo::RIBConverterVideo(ros::NodeHandle *nh)
  : RIBConverter<sensor_msgs::Image>(nh)
{
  InitCodec();
}

RIBConverterVideo::RIBConverterVideo(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh)
  : RIBConverter<sensor_msgs::Image>(topicPublish, topicSubscribe, nh)
{
  InitCodec();
}

void RIBConverterVideo::InitCodec()
{
  VP9Encoder* VP9StreamEncoder = new VP9Encoder();
  VP9StreamEncoder->SetPicWidthAndHeight(320,240); // default size
  VP9StreamEncoder->InitializeEncoder();
  //VP9StreamEncoder->SetLosslessLink(true);
  this->VideoEncoder = VP9StreamEncoder;

  this->ImageBuffer = new unsigned char[320*240*3/2]; // Buffer for 320 x 240 image in YUV420 
}


void ConvertBGRToYUV420(unsigned char *bgr, unsigned char *destination, unsigned int width, unsigned int height)
{
  size_t image_size = width * height;
  size_t upos = image_size;
  size_t vpos = upos + upos / 4;
  size_t i = 0;

  unsigned char* pbgr = bgr;
  unsigned char* pdest = destination;
  unsigned char* pdest_u = destination + upos;
  unsigned char* pdest_v = destination + vpos;
  unsigned char* pdest_end = destination + image_size;
    
  while (pdest < pdest_end)
    {
    unsigned char* pdest_endl = pdest + width;
    while (pdest < pdest_endl)
      {
      int b = *(pbgr++);
      int g = *(pbgr++);
      int r = *(pbgr++);
      
      *(pdest++) =   (unsigned char)((66 * r + 129 * g + 25 * b) >> 8) + 16;
      *(pdest_u++) = (unsigned char)((-38 * r - 74 * g + 112 * b) >> 8) + 128;
      *(pdest_v++) = (unsigned char)((112 * r - 94 * g - 18 * b) >> 8) + 128;

      b = *(pbgr++);
      g = *(pbgr++);
      r = *(pbgr++);
      *(pdest++) = ((66 * r + 129 * g + 25 * b) >> 8) + 16;
      }
    
    pdest_endl = pdest + width;
    while (pdest < pdest_endl)
      {
      int r = *(pbgr++);
      int g = *(pbgr++);
      int b = *(pbgr++);
      
      *(pdest++) = (unsigned char) ((66 * r + 129 * g + 25 * b) >> 8) + 16;
      }
    }
}

int RIBConverterVideo::onIGTLMessage(igtl::MessageHeader * header)
{
}


void RIBConverterVideo::onROSMessage(const sensor_msgs::Image::ConstPtr & msg)
{
  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return;
    }

  unsigned int width = msg->width;
  unsigned int height = msg->height;
  unsigned int imageSize = width*height;

  unsigned int origWidth = this->VideoEncoder->GetPicWidth();
  unsigned int origHeight = this->VideoEncoder->GetPicHeight();
  
  if (origWidth != width || origHeight != height)
    {
    this->VideoEncoder->SetPicWidthAndHeight(width, height);
    this->VideoEncoder->InitializeEncoder();
    if (this->ImageBuffer)
      {
      delete [] this->ImageBuffer;
      this->ImageBuffer = new unsigned char[imageSize*3/2]; // Buffer for width x height in YUV420
      }
    }

  // TODO: Check endian
  // BGRToYUV converter is implemented above, because the converter in OpenIGTLink is not well optimized
  // and it only support RGB (not BGR).
  ConvertBGRToYUV420((igtlUint8*)msg->data.data(),(igtlUint8*) this->ImageBuffer, width, height);
  //this->VideoEncoder->ConvertRGBToYUV((igtlUint8*)msg->data.data(),(igtlUint8*) this->ImageBuffer, width, height);

  SourcePicture* pSrcPic = new SourcePicture();
  pSrcPic->colorFormat = FormatI420;
  pSrcPic->picWidth = width; // check the test image
  pSrcPic->picHeight = height;
  pSrcPic->data[0] = this->ImageBuffer;
  pSrcPic->data[1] = pSrcPic->data[0] + imageSize;
  pSrcPic->data[2] = pSrcPic->data[1] + imageSize/4;
  pSrcPic->stride[0] = pSrcPic->picWidth;
  pSrcPic->stride[1] = pSrcPic->stride[2] = pSrcPic->stride[0] >> 1;
  
  igtl::VideoMessage::Pointer videoMessage = igtl::VideoMessage::New();
  videoMessage->SetDeviceName("ROS-IGTL-Bridge");
  videoMessage->SetHeaderVersion(IGTL_HEADER_VERSION_2);
  int iEncFrames = this->VideoEncoder->EncodeSingleFrameIntoVideoMSG(pSrcPic, videoMessage, false);

  if(iEncFrames == 0)
    {
    socket->Send(videoMessage->GetPackPointer(), videoMessage->GetPackSize());
    }
  
}





