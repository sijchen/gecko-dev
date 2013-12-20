/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "CSFLog.h"
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

  return WEBRTC_VIDEO_CODEC_OK;
}


int32_t WebrtcOpenH264VideoEncoder::Encode(
    const webrtc::I420VideoFrame& inputImage,
    const webrtc::CodecSpecificInfo* codecSpecificInfo,
    const std::vector<webrtc::VideoFrameType>* frame_types) {
  EncodedFrame encoded;

  const uint8_t* buffer = inputImage.buffer(webrtc::kYPlane);
  encoded.width_ = inputImage.width();
  encoded.height_ = inputImage.height();
  encoded.value_ = *buffer;
  encoded.timestamp_ = timestamp_;
  timestamp_ += 90000 / 30;

  MutexAutoLock lock(mutex_);
  frames_.push(encoded);

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
