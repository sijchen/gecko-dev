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

  static EncodedFrame* Create(const SFrameBSInfo& frame,
                              uint32_t width, uint32_t height,
                              uint32_t timestamp, webrtc::VideoFrameType frame_type) {
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
  param.iTargetBitrate = codecSettings->maxBitrate * 1000; // kbps -> bps
  param.iTemporalLayerNum = 1;
  param.iSpatialLayerNum = 1;
  // TODO(ekr@rtfm.com). Scary conversion from unsigned char to float below.
  param.fFrameRate = codecSettings->maxFramerate;
  param.iInputCsp = videoFormatI420;

  // Set up layers. Currently we have one layer.
  auto layer = &param.sSpatialLayers[0];

  layer->iVideoWidth = codecSettings->width;
  layer->iVideoHeight = codecSettings->height;
  layer->iQualityLayerNum = 1;
  layer->iSpatialBitrate = param.iTargetBitrate;

  // Based on guidance from Cisco.
  layer->sSliceCfg.sSliceArgument.uiSliceMbNum[0] = 1000;
  layer->sSliceCfg.sSliceArgument.uiSliceNum = 1;
  layer->sSliceCfg.sSliceArgument.uiSliceSizeConstraint = 1000;

  rv = encoder_->Initialize(&param, INIT_TYPE_PARAMETER_BASED);
  if (rv)
    return WEBRTC_VIDEO_CODEC_MEMORY;

  return WEBRTC_VIDEO_CODEC_OK;
}


int32_t WebrtcOpenH264VideoEncoder::Encode(
    const webrtc::I420VideoFrame& inputImage,
    const webrtc::CodecSpecificInfo* codecSpecificInfo,
    const std::vector<webrtc::VideoFrameType>* frame_types) {
  MOZ_MTLOG(ML_DEBUG, "Encoding frame");

  MOZ_ASSERT(!frame_types->empty());
  if (frame_types->empty())
    return WEBRTC_VIDEO_CODEC_ERROR;
  // TODO(ekr@rtfm.com): Actually handle frame type.

  SFrameBSInfo encoded;
  SSourcePicture src;

  src.iColorFormat = videoFormatI420;
  src.iStride[0] = inputImage.stride(webrtc::kYPlane);
  src.pData[0] = reinterpret_cast<unsigned char*>(
      const_cast<uint8_t *>(inputImage.buffer(webrtc::kYPlane)));
  src.iStride[1] = inputImage.stride(webrtc::kUPlane);
  src.pData[1] = reinterpret_cast<unsigned char*>(
      const_cast<uint8_t *>(inputImage.buffer(webrtc::kUPlane)));
  src.iStride[2] = inputImage.stride(webrtc::kVPlane);
  src.pData[2] = reinterpret_cast<unsigned char*>(
      const_cast<uint8_t *>(inputImage.buffer(webrtc::kVPlane)));
  src.iStride[3] = 0;
  src.pData[3] = nullptr;
  src.iPicWidth = inputImage.width();
  src.iPicHeight = inputImage.height();

  const SSourcePicture* pics = &src;

  int type = encoder_->EncodeFrame(&pics, 1, &encoded);

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
      return WEBRTC_VIDEO_CODEC_ERROR;
      break;
    default:
      // The API is defined as returning a type.
      MOZ_CRASH();
      break;
  }

  ScopedDeletePtr<EncodedFrame> encoded_frame(
      EncodedFrame::Create(encoded,
                           inputImage.width(),
                           inputImage.height(),
                           inputImage.timestamp(),
                           (*frame_types)[0]));

  MutexAutoLock lock(mutex_);
  frames_.push(encoded_frame.forget());

  RUN_ON_THREAD(thread_,
                WrapRunnable(
                    // RefPtr keeps object alive.
                    nsRefPtr<WebrtcOpenH264VideoEncoder>(this),
                    &WebrtcOpenH264VideoEncoder::EmitFrames),
                NS_DISPATCH_NORMAL);

  return WEBRTC_VIDEO_CODEC_OK;
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
  int rv = decoder_->DecodeFrame(inputImage._buffer,
                                 inputImage._length,
                                 data,
                                 &decoded);
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
    if (decoded_image_.CreateFrame(len, static_cast<uint8_t *>(data[0]),
                                   uvstride*height/2, static_cast<uint8_t *>(data[1]),
                                   uvstride*height/2, static_cast<uint8_t *>(data[2]),
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
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t WebrtcOpenH264VideoDecoder::Reset() {
  return WEBRTC_VIDEO_CODEC_OK;
}

}
