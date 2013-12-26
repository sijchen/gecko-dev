/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <memory>

#include "logging.h"
#include "nspr.h"

#include "codec_def.h"
#include "codec_app_def.h"
#include "codec_api.h"

#include <iostream>
#include <sys/time.h>

#include <mozilla/Scoped.h>
#include "VideoConduit.h"
#include "AudioConduit.h"

#include "video_engine/include/vie_external_codec.h"

#include "codec_def.h"
#include "codec_app_def.h"
#include "codec_api.h"
#include "param_svc.h"


#include "runnable_utils.h"

#include "WebrtcOpenH264VideoCodec.h"

namespace mozilla {

//#define OUTPUT_BITSTREAM
//#define GET_TIMING
    
    
#define GET_TIMING_MZLOG
//Usage:
//    export NSPR_LOG_MODULES=openh264:X //X=5
//    export NSPR_LOG_FILE=~/Desktop/log.txt
    
MOZ_MTLOG_MODULE("openh264");

struct EncodedFrame {
 public:
  EncodedFrame(uint8_t *buffer, uint32_t size, uint32_t length,
               uint32_t width, uint32_t height, uint32_t timestamp,
               webrtc::VideoFrameType frame_type) :
      image_(buffer, size, length),
      buffer_(buffer) {
    image_._encodedWidth = width;
    image_._encodedHeight = height;
    image_._timeStamp = timestamp;
    image_._frameType = frame_type;
    image_._completeFrame = true;
  }
#ifdef OUTPUT_BITSTREAM
  static EncodedFrame* Create(const SFrameBSInfo& frame,
                              uint32_t width, uint32_t height,
                              uint32_t timestamp, webrtc::VideoFrameType frame_type,
                              int iEncoderIdx)
    //FILE* pEncStrmFile)
#else
    static EncodedFrame* Create(const SFrameBSInfo& frame,
                                uint32_t width, uint32_t height,
                                uint32_t timestamp, webrtc::VideoFrameType frame_type)
#endif
    
    {
    // Buffer up the data.
    uint32_t length = 0;
    std::vector<uint32_t> lengths;

    for (int i=0; i<frame.iLayerNum; ++i) {
      lengths.push_back(0);
      for (int j=0; j<frame.sLayerInfo[i].iNalCount; ++j) {
        lengths[i] += frame.sLayerInfo[i].iNalLengthInByte[j];
        length += frame.sLayerInfo[i].iNalLengthInByte[j];
      }
    }

    ScopedDeleteArray<uint8_t> buffer(new uint8_t[length]);
    uint8_t *tmp = buffer;

    for (int i=0; i<frame.iLayerNum; ++i) {
      // TODO(ekr@rtfm.com): This seems screwy, but I copied it from Cisco.
      memcpy(tmp, frame.sLayerInfo[i].pBsBuf, lengths[i]);
      tmp += lengths[i];
    }
      
#ifdef OUTPUT_BITSTREAM
    char cFileName[100];
    std::snprintf(cFileName, sizeof(cFileName), "h264encstrm_enc%d.264", iEncoderIdx);
    FILE* pEncStrmFile = fopen(cFileName,"ab+");
        
    //if not open the file again,
    //the following code cannot write out bit stream correctly
    fwrite(((unsigned char *)buffer), 1, length, pEncStrmFile);
    fclose(pEncStrmFile);
        
        //the following code can write out bit stream correctly
        //FILE* ftmp = fopen("encstream.264","ab+");
        //     fwrite(((unsigned char *)buffer), 1, length, ftmp);
        //fclose(ftmp);
#endif
      
    return new EncodedFrame(buffer.forget(), length, length,
                            width, height, timestamp, frame_type);
  }

  webrtc::EncodedImage& image() { return image_; }

 private:
  webrtc::EncodedImage image_;
  ScopedDeleteArray<uint8_t> buffer_;
};

// Encoder.
WebrtcOpenH264VideoEncoder::WebrtcOpenH264VideoEncoder()
    : callback_(nullptr),
      mutex_("WebrtcOpenH264VideoEncoder") {
  nsIThread* thread;

  nsresult rv = NS_NewNamedThread("encoder-thread", &thread);
  MOZ_ASSERT(NS_SUCCEEDED(rv));

  thread_ = thread;
}

WebrtcOpenH264VideoEncoder::~WebrtcOpenH264VideoEncoder() {
  if (encoder_) {
    DestroySVCEncoder(encoder_);
  }
}

int32_t WebrtcOpenH264VideoEncoder::InitEncode(
    const webrtc::VideoCodec* codecSettings,
    int32_t numberOfCores,
    uint32_t maxPayloadSize) {
  max_payload_size_ = maxPayloadSize;

  int rv = CreateSVCEncoder(&encoder_);
  if (rv) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  SVCEncodingParam param;
  memset(&param, 0, sizeof(param));

  MOZ_MTLOG(ML_INFO, "Initializing encoder at "
	    << codecSettings->width
	    << "x"
	    << codecSettings->height
	    << "@"
	    << static_cast<int>(codecSettings->maxFramerate));

  // Translate parameters.
  param.iPicWidth = codecSettings->width;
  param.iPicHeight = codecSettings->height;
  param.iTargetBitrate = codecSettings->startBitrate * 1000;
  param.iTemporalLayerNum = 1;
  param.iSpatialLayerNum = 1;
  //param.iIntraPeriod = 30*(static_cast<int>(codecSettings->maxFramerate));
  // TODO(ekr@rtfm.com). Scary conversion from unsigned char to float below.
  param.fFrameRate = codecSettings->maxFramerate;
  param.iInputCsp = videoFormatI420;

  // Set up layers. Currently we have one layer.
  auto layer = &param.sSpatialLayers[0];

  layer->iVideoWidth = codecSettings->width;
  layer->iVideoHeight = codecSettings->height;
  layer->iQualityLayerNum = 1;
  layer->iSpatialBitrate = param.iTargetBitrate;
  layer->fFrameRate = param.fFrameRate;

  // Based on guidance from Cisco.
  layer->sSliceCfg.sSliceArgument.uiSliceMbNum[0] = 1000;
  layer->sSliceCfg.sSliceArgument.uiSliceNum = 1;
  layer->sSliceCfg.sSliceArgument.uiSliceSizeConstraint = 1000;

  rv = encoder_->Initialize(&param, INIT_TYPE_PARAMETER_BASED);
  if (rv)
    return WEBRTC_VIDEO_CODEC_MEMORY;
    
//#ifdef CODEC_TRACE
  static int iEncoderIdx = 0;
  char cFileName[100];
    
#ifdef OUTPUT_BITSTREAM
  //std::snprintf(cFileName, sizeof(cFileName), "h264encstrm_enc%d.264", iEncoderIdx);
  //m_pEncStrmFile = fopen(cFileName,"wb+");

#endif
#ifdef GET_TIMING
  //std::snprintf(cFileName, sizeof(cFileName), "h264enctrace_enc%d.txt", iEncoderIdx);
  //m_pEncTraceFile = fopen(cFileName,"w+");
#endif
    MOZ_MTLOG(ML_INFO, "Encoder:\t"<< iEncoderIdx
              << "InputParam:\t" << codecSettings->maxFramerate << "fps ("
              << codecSettings->startBitrate << "," << codecSettings->minBitrate << "," << codecSettings->maxBitrate << ")kbps \t"
              << "Encoding" << param.fFrameRate << "fps@" << param.iTargetBitrate <<" bps \t"
              << "EncodingLayer" << layer->iVideoWidth << "x" << layer->iVideoHeight
              << "@" << layer->fFrameRate << "fps@" << layer->iSpatialBitrate<<"bps \t");
    
//#endif
  m_iEncoderIdx=iEncoderIdx;
  iEncoderIdx++;
   
    m_tLastCallTime.tv_sec = 0;
    m_tLastCallTime.tv_usec = 0;
    
  return WEBRTC_VIDEO_CODEC_OK;
}


int32_t WebrtcOpenH264VideoEncoder::Encode(
    const webrtc::I420VideoFrame& inputImage,
    const webrtc::CodecSpecificInfo* codecSpecificInfo,
    const std::vector<webrtc::VideoFrameType>* frame_types) {
  MOZ_MTLOG(ML_DEBUG, "Frame added");

  MOZ_ASSERT(!frame_types->empty());
  if (frame_types->empty())
    return WEBRTC_VIDEO_CODEC_ERROR;
  // TODO(ekr@rtfm.com): Actually handle frame type.

  webrtc::I420VideoFrame* imageCopy = new webrtc::I420VideoFrame();
  imageCopy->CopyFrame(inputImage);

  RUN_ON_THREAD(thread_,
      WrapRunnable(nsRefPtr<WebrtcOpenH264VideoEncoder>(this),
		   &WebrtcOpenH264VideoEncoder::Encode_w,
		   imageCopy, (*frame_types)[0]),
		NS_DISPATCH_NORMAL);

  return WEBRTC_VIDEO_CODEC_OK;
}

void WebrtcOpenH264VideoEncoder::Encode_w(
    webrtc::I420VideoFrame* inputImage,
    webrtc::VideoFrameType frame_type) {
  SFrameBSInfo encoded;
  SSourcePicture src;

  src.iColorFormat = videoFormatI420;
  src.iStride[0] = inputImage->stride(webrtc::kYPlane);
  src.pData[0] = reinterpret_cast<unsigned char*>(
      const_cast<uint8_t *>(inputImage->buffer(webrtc::kYPlane)));
  src.iStride[1] = inputImage->stride(webrtc::kUPlane);
  src.pData[1] = reinterpret_cast<unsigned char*>(
      const_cast<uint8_t *>(inputImage->buffer(webrtc::kUPlane)));
  src.iStride[2] = inputImage->stride(webrtc::kVPlane);
  src.pData[2] = reinterpret_cast<unsigned char*>(
      const_cast<uint8_t *>(inputImage->buffer(webrtc::kVPlane)));
  src.iStride[3] = 0;
  src.pData[3] = nullptr;
  src.iPicWidth = inputImage->width();
  src.iPicHeight = inputImage->height();

  const SSourcePicture* pics = &src;

#ifdef GET_TIMING
    char cFileName[100];
    std::snprintf(cFileName, sizeof(cFileName), "h264encoder_enc%d.txt", m_iEncoderIdx);
    FILE* fenctrace = fopen(cFileName,"a+");
    struct timeval tv;
    
    time_t curtime1, curtime2;
    gettimeofday(&tv, NULL);
    curtime1=tv.tv_usec;
    fprintf(fenctrace, "Encoder\t%d\t:BeforeEncoder(tv_sec,tv_usec):\t%ld\t%ld\t", m_iEncoderIdx, tv.tv_sec, tv.tv_usec);
#endif
#ifdef GET_TIMING_MZLOG
    struct timeval tv2;
    struct timeval BeginTime;
    
    timeval curtimeB;
    gettimeofday(&tv2, NULL);
    
    BeginTime = tv2;
    //MOZ_MTLOG(ML_INFO, "Encoder\t"<< m_iEncoderIdx << "\tBeforeEncoder\t"<< tv2.tv_sec << "\t" << tv2.tv_usec);
#endif
    
  PRIntervalTime t0 = PR_IntervalNow();
  int type = encoder_->EncodeFrame(&pics, 1, &encoded);
  PRIntervalTime t1 = PR_IntervalNow();
  MOZ_MTLOG(ML_DEBUG, "Encoding time: " << PR_IntervalToMilliseconds(t1 - t0) << "ms");
    
#ifdef GET_TIMING
    gettimeofday(&tv, NULL);
    curtime2=tv.tv_usec;
    fprintf(fenctrace, "AfterEncoder(tv_sec,tv_usec): \t%ld\t%ld\t", tv.tv_sec, tv.tv_usec);
#endif
#ifdef GET_TIMING_MZLOG
    gettimeofday(&tv2, NULL);
    curtimeB=tv2;
    //MOZ_MTLOG(ML_INFO, "Encoder\t"<< m_iEncoderIdx << "\tAfterEncoder\t"<< tv2.tv_sec << "\t" << tv2.tv_usec);
    int timestamp = inputImage->timestamp();
#endif



  // Translate int to enum
  switch (type) {
    case videoFrameTypeIDR:
    case videoFrameTypeI:
    case videoFrameTypeP:
    case videoFrameTypeSkip:
    case videoFrameTypeIPMixed:
      break;
    case videoFrameTypeInvalid:
      MOZ_MTLOG(ML_ERROR, "Couldn't encode frame. Error = " << type);
      return;
      break;
    default:
      // The API is defined as returning a type.
      MOZ_CRASH();
      break;
  }
#ifdef OUTPUT_BITSTREAM
  ScopedDeletePtr<EncodedFrame> encoded_frame(
      EncodedFrame::Create(encoded,
                           inputImage->width(),
                           inputImage->height(),
                           inputImage->timestamp(),
                           frame_type,
                           m_iEncoderIdx));
                           // m_pEncStrmFile));
#else
    ScopedDeletePtr<EncodedFrame> encoded_frame(
                                                EncodedFrame::Create(encoded,
                                                                     inputImage->width(),
                                                                     inputImage->height(),
                                                                     inputImage->timestamp(),
                                                                     frame_type));
#endif
    if (0==encoded_frame->image()._length)
    {
        MOZ_MTLOG(ML_INFO, "Encoder\t"<< m_iEncoderIdx
                  << "\tInputWidth\t"<< inputImage->width() << "\tInputHeight\t"<< inputImage->height() << "\tInputTimestamp\t"<< inputImage->timestamp()
                  << "\tEncoderReturn\t"<< type
                  << "\tEncodedFrameTimestamp\t"<< encoded_frame->image()._timeStamp << "\tEncodedFrameLength\t"<< encoded_frame->image()._length
                  << "\tEncodedFrameLayerNum\t"<< encoded.iLayerNum );
        
    }
    
    delete inputImage;
    callback_->Encoded(encoded_frame->image(), NULL, NULL);
    
#ifdef GET_TIMING
    gettimeofday(&tv, NULL);
    fprintf(fenctrace, "AfterEmit(tv_sec, tv_usec): \t%ld\t%ld\t EncoderDelta(ms): \t%ld\n", tv.tv_sec, tv.tv_usec, (curtime2-curtime1)/1000);
    fclose(fenctrace);
#endif
#ifdef GET_TIMING_MZLOG
    if (encoded_frame->image()._length)
    {
    gettimeofday(&tv2, NULL);
    int calldiff = (m_tLastCallTime.tv_usec)?(((BeginTime.tv_sec - m_tLastCallTime.tv_sec) * 1000000 + (BeginTime.tv_usec - m_tLastCallTime.tv_usec))/1000):0;
    MOZ_MTLOG(ML_INFO, "Encoder\t"<< m_iEncoderIdx
              << "\tCurFrameTimestamp\t"<< encoded_frame->image()._timeStamp << "\tCurFrameLength\t"<< encoded_frame->image()._length
              << "\tBeginEncoder\t"<< BeginTime.tv_sec << "\t" << BeginTime.tv_usec
              << "\tAfterEmit\t"<< tv2.tv_sec << "\t" << tv2.tv_usec
              << "\tEncoderDelta\t" << ((curtimeB.tv_sec - BeginTime.tv_sec) * 1000000 + (curtimeB.tv_usec - BeginTime.tv_usec))/1000
              << "\tBetweenCallEncoder\t" << calldiff );
    m_tLastCallTime=BeginTime;
    }
#endif


  return;
}

void WebrtcOpenH264VideoEncoder::EmitFrames() {
  MutexAutoLock lock(mutex_);

  while(!frames_.empty()) {
    ScopedDeletePtr<EncodedFrame> frame(frames_.front());
    MOZ_MTLOG(ML_DEBUG, "Emitting frame length=" << frame->image()._length);
    callback_->Encoded(frame->image(), NULL, NULL);
    frames_.pop();
  }
}

int32_t WebrtcOpenH264VideoEncoder::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback) {
  callback_ = callback;

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t WebrtcOpenH264VideoEncoder::Release() {
#ifdef OUTPUT_BITSTREAM
  //if (m_pEncStrmFile)
  //  fclose(m_pEncStrmFile);
#endif
#ifdef GET_TIMING
  //  if (m_pEncTraceFile)
  //      fclose(m_pEncTraceFile);
#endif
    return WEBRTC_VIDEO_CODEC_OK;
}

int32_t WebrtcOpenH264VideoEncoder::SetChannelParameters(uint32_t packetLoss,
						     int rtt) {
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t WebrtcOpenH264VideoEncoder::SetRates(uint32_t newBitRate,
                                             uint32_t frameRate) {
  return WEBRTC_VIDEO_CODEC_OK;
}



// Decoder.
WebrtcOpenH264VideoDecoder::WebrtcOpenH264VideoDecoder()
    : decoder_(nullptr),
      callback_(nullptr),
      mutex_("WebrtcOpenH264VideoDecoder") {
  nsIThread* thread;

  nsresult rv = NS_NewNamedThread("encoder-thread", &thread);
  MOZ_ASSERT(NS_SUCCEEDED(rv));

  thread_ = thread;
}

WebrtcOpenH264VideoDecoder::~WebrtcOpenH264VideoDecoder() {
  if (decoder_) {
    DestroyDecoder(decoder_);
  }
}

int32_t WebrtcOpenH264VideoDecoder::InitDecode(
    const webrtc::VideoCodec* codecSettings,
    int32_t numberOfCores) {
  long rv = CreateDecoder (&decoder_);
  if (rv) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  SDecodingParam param;
  memset(&param, 0, sizeof(param));
  param.iOutputColorFormat = videoFormatI420;
  param.uiTargetDqLayer = UCHAR_MAX;  // TODO(ekr@rtfm.com): correct?
  param.uiEcActiveFlag = 1; // Error concealment on.
  param.sVideoProperty.eVideoBsType = VIDEO_BITSTREAM_DEFAULT;

  long lrv = decoder_->Initialize(&param, INIT_TYPE_PARAMETER_BASED);
  if (lrv) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
    
//#ifdef CODEC_TRACE
    static int iDecoderIdx = 0;
    char cFileName[100];
    
#ifdef GET_TIMING
    
    //std::snprintf(cFileName, sizeof(cFileName), "h264dectrace_dec%d.txt", iDecoderIdx);
    //m_pDecTraceFile = fopen(cFileName,"wb+");
#endif
//#endif
    m_iDecoderIdx=iDecoderIdx;
    iDecoderIdx++;
    

    m_tLastCallTime.tv_sec = 0;
    m_tLastCallTime.tv_usec = 0;
    
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t WebrtcOpenH264VideoDecoder::Decode(
    const webrtc::EncodedImage& inputImage,
    bool missingFrames,
    const webrtc::RTPFragmentationHeader* fragmentation,
    const webrtc::CodecSpecificInfo*
    codecSpecificInfo,
    int64_t renderTimeMs) {
  SBufferInfo decoded;
  memset(&decoded, 0, sizeof(decoded));
  void *data[3] = {nullptr, nullptr, nullptr};
  MOZ_MTLOG(ML_DEBUG, "Decoding frame input length=" << inputImage._length);

#ifdef GET_TIMING
    char cFileName[100];
    std::snprintf(cFileName, sizeof(cFileName), "h264decoder_dec%d.txt", m_iDecoderIdx);
    FILE* fdectrace = fopen(cFileName,"a+");
    struct timeval tv;
    
    time_t curtime1, curtime2,curtime3;
    gettimeofday(&tv, NULL);
    curtime1=tv.tv_usec;
    fprintf(fdectrace, "Decoder\t%d\t:BeforeDecoder(tv_sec,tv_usec):\t%ld\t%ld\t", m_iDecoderIdx, tv.tv_sec, tv.tv_usec);
#endif
#ifdef GET_TIMING_MZLOG
    struct timeval tv2;
    struct timeval BeginTime;
    
    timeval curtimeB;
    gettimeofday(&tv2, NULL);
    
    BeginTime = tv2;
    //MOZ_MTLOG(ML_INFO, "Decoder\t"<< m_iDecoderIdx << "\tBeforeDecoder\t"<< tv2.tv_sec << "\t" << tv2.tv_usec);
#endif
    
  int rv = decoder_->DecodeFrame(inputImage._buffer,
                                 inputImage._length,
                                 data,
                                 &decoded);
    
#ifdef GET_TIMING
    gettimeofday(&tv, NULL);
    curtime2=tv.tv_usec;
    fprintf(fdectrace, "AfterDecoder(tv_sec,tv_usec): \t%ld\t%ld\t", tv.tv_sec, tv.tv_usec);
#endif
#ifdef GET_TIMING_MZLOG
    gettimeofday(&tv2, NULL);
    curtimeB=tv2;
    //MOZ_MTLOG(ML_INFO, "Decoder\t"<< m_iDecoderIdx << "\tAfterDecoder\t"<< tv2.tv_sec << "\t" << tv2.tv_usec);
#endif

    
  if (rv) {
    MOZ_MTLOG(ML_ERROR, "Decoding error rv=" << rv);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }


  MutexAutoLock lock(mutex_);
  int width;
  int height;
  int ystride;
  int uvstride;

  if (decoded.eBufferProperty == BUFFER_HOST) {
    width = decoded.UsrData.sSystemBuffer.iWidth;
    height = decoded.UsrData.sSystemBuffer.iHeight;
    ystride = decoded.UsrData.sSystemBuffer.iStride[0];
    uvstride = decoded.UsrData.sSystemBuffer.iStride[1];
  } else {
    // TODO(ekr@rtfm.com): How can this happen
    MOZ_CRASH();
    width = decoded.UsrData.sVideoBuffer.iSurfaceWidth;
    height = decoded.UsrData.sVideoBuffer.iSurfaceHeight;
  }
  int len = ystride * height;

  if (len) {
    if (decoded_image_.CreateFrame(ystride * height, static_cast<uint8_t *>(data[0]),
                                   uvstride * height/2, static_cast<uint8_t *>(data[1]),
                                   uvstride * height/2, static_cast<uint8_t *>(data[2]),
                                   width, height,
                                   ystride, uvstride, uvstride
                                   ))
      return WEBRTC_VIDEO_CODEC_ERROR;
    decoded_image_.set_timestamp(inputImage._timeStamp);

    RUN_ON_THREAD(thread_,
                  // Shared pointer keeps the object live.
                  WrapRunnable(nsRefPtr<WebrtcOpenH264VideoDecoder>(this),
                               &WebrtcOpenH264VideoDecoder::RunCallback),
                  NS_DISPATCH_NORMAL);
  }

#ifdef GET_TIMING
    gettimeofday(&tv, NULL);
    curtime3=tv.tv_usec;
    fprintf(fdectrace, "AfterEmit(tv_sec,tv_usec)): \t%ld\t%ld\t DecoderDelta(ms): \t%ld\n", tv.tv_sec, tv.tv_usec, (curtime2-curtime1)/1000);
    fclose(fdectrace);
#endif
#ifdef GET_TIMING_MZLOG
    gettimeofday(&tv2, NULL);
    int calldiff = (m_tLastCallTime.tv_usec)?(((BeginTime.tv_sec - m_tLastCallTime.tv_sec) * 1000000 + (BeginTime.tv_usec - m_tLastCallTime.tv_usec))/1000):0;
    MOZ_MTLOG(ML_INFO, "Decoder\t"<< m_iDecoderIdx
              << "\tCurFrameTimestamp\t"<< inputImage._timeStamp << "\tCurFrameLength\t"<< inputImage._length
              << "\tBeginDecoder\t"<< BeginTime.tv_sec << "\t" << BeginTime.tv_usec
              << "\tAfterEmit\t"<< tv2.tv_sec << "\t" << tv2.tv_usec
              << "\tDecoderDelta\t" << ((curtimeB.tv_sec - BeginTime.tv_sec) * 1000000 + (curtimeB.tv_usec - BeginTime.tv_usec))/1000
              << "\tBetweenCallDecoder\t" << calldiff );
    m_tLastCallTime=BeginTime;
#endif
#ifdef OUTPUT_YUV
    FILE* fdecstrm = fopen("h264dec.yuv","ab+");
    fwrite(((unsigned char *)data[0]), 1, ystride*height, fdecstrm);
    fwrite(((unsigned char *)data[1]), 1, uvstride*height/2, fdecstrm);
    fwrite(((unsigned char *)data[2]), 1, uvstride*height/2, fdecstrm);
    fclose(fdecstrm);
    
    FILE* fdectrace = fopen("h264dec.txt","a+");
    fprintf(fdectrace, "Decoder: %dx%d, Ystride:%d\n", width,height,ystride);
    fclose(fdectrace);
#endif
    
  return WEBRTC_VIDEO_CODEC_OK;
}

void WebrtcOpenH264VideoDecoder::RunCallback() {
  MutexAutoLock lock(mutex_);

  callback_->Decoded(decoded_image_);
}

int32_t WebrtcOpenH264VideoDecoder::RegisterDecodeCompleteCallback(
    webrtc::DecodedImageCallback* callback) {
  callback_ = callback;

  return WEBRTC_VIDEO_CODEC_OK;
}


int32_t WebrtcOpenH264VideoDecoder::Release() {
#ifdef GET_TIMING
  //  if (m_pDecTraceFile)
  //      fclose(m_pDecTraceFile);
#endif
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t WebrtcOpenH264VideoDecoder::Reset() {
  return WEBRTC_VIDEO_CODEC_OK;
}

}
