/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

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

// Encoder.
WebrtcOpenH264VideoEncoder::WebrtcOpenH264VideoEncoder()
  : timestamp_(0),
    callback_(nullptr),
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

  return WEBRTC_VIDEO_CODEC_OK;
}

void WebrtcOpenH264VideoEncoder::EmitFrames() {
  MutexAutoLock lock(mutex_);

  while(!frames_.empty()) {
    EncodedFrame *frame = &frames_.front();
    EmitFrame(frame);
    frames_.pop();
  }
}

void WebrtcOpenH264VideoEncoder::EmitFrame(EncodedFrame *frame) {
  webrtc::EncodedImage encoded_image;
  encoded_image._encodedWidth = frame->width_;
  encoded_image._encodedHeight = frame->height_;
  encoded_image._timeStamp = frame->timestamp_;  // TODO(ekr@rtfm.com): Fix times
  encoded_image.capture_time_ms_ = 0;
  encoded_image._frameType = webrtc::kKeyFrame;
  encoded_image._buffer=reinterpret_cast<uint8_t *>(frame);
  encoded_image._length = sizeof(EncodedFrame);
  encoded_image._size = sizeof(EncodedFrame);
  encoded_image._completeFrame = true;

  callback_->Encoded(encoded_image, NULL, NULL);
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
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t WebrtcOpenH264VideoDecoder::Decode(
    const webrtc::EncodedImage& inputImage,
    bool missingFrames,
    const webrtc::RTPFragmentationHeader* fragmentation,
    const webrtc::CodecSpecificInfo*
    codecSpecificInfo,
    int64_t renderTimeMs) {
  if (sizeof(EncodedFrame) != inputImage._length)
    return WEBRTC_VIDEO_CODEC_ERROR;

  EncodedFrame* frame = reinterpret_cast<EncodedFrame*>(
      inputImage._buffer);
  size_t len = frame->width_ * frame->height_;
  ScopedDeleteArray<uint8_t> data(new uint8_t[len]);
  memset(data.get(), frame->value_, len);

  MutexAutoLock lock(mutex_);
  if (decoded_image_.CreateFrame(len, data,
                                 len/4, data,
                                 len/4, data,
                                 frame->width_, frame->height_,
                                 frame->width_,
                                 frame->width_/2,
                                 frame->width_/2))
    return WEBRTC_VIDEO_CODEC_ERROR;
  decoded_image_.set_timestamp(inputImage._timeStamp);

  RUN_ON_THREAD(thread_,
                // Shared pointer keeps the object live.
                WrapRunnable(nsRefPtr<WebrtcOpenH264VideoDecoder>(this),
                             &WebrtcOpenH264VideoDecoder::RunCallback),
                NS_DISPATCH_NORMAL);

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
