// #include <stdexcept>
// #include <libavcodec/avcodec.h>
// #include <libswscale/swscale.h>
// #include <libavutil/imgutils.h>

#include "h264decoder.hpp"

// // Détecte API moderne (FFmpeg 5/6 => libavcodec >= 59)
// #if !defined(LIBAVCODEC_VERSION_MAJOR)
//   #error "LIBAVCODEC_VERSION_MAJOR not defined"
// #endif
// #define USE_OLD_FFMPEG (LIBAVCODEC_VERSION_MAJOR < 59)

typedef unsigned char ubyte;

// class H264Decoder {
// public:
//   H264Decoder();
//   ~H264Decoder();

//   // Remplis 'pkt' avant d’appeler decode_frame(); retourne une frame décodée
//   const AVFrame& decode_frame();

//   AVCodecContext *context = nullptr;
//   const AVCodec *codec = nullptr;
//   AVPacket *pkt = nullptr;
//   AVFrame *frame = nullptr;

// #if USE_OLD_FFMPEG
//   // Ancienne API : rien de plus
// #else
//   // API moderne : on garde la possibilité d’utiliser un parser si besoin
//   AVCodecParserContext *parser = nullptr;
// #endif
// };

H264Decoder::H264Decoder() {
#if USE_OLD_FFMPEG
  // Ancienne API (Ubuntu 20.04 / Foxy)
  avcodec_register_all();
#endif

  codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec) throw std::runtime_error("H264 decoder not found");

  context = avcodec_alloc_context3(codec);
  if (!context) throw std::runtime_error("alloc context failed");

#if USE_OLD_FFMPEG
  // Compat flags historiques
  #ifndef CODEC_CAP_TRUNCATED
    #define CODEC_CAP_TRUNCATED AV_CODEC_CAP_TRUNCATED
  #endif
  #ifndef CODEC_FLAG_TRUNCATED
    #define CODEC_FLAG_TRUNCATED AV_CODEC_FLAG_TRUNCATED
  #endif
  if (codec->capabilities & CODEC_CAP_TRUNCATED) {
    context->flags |= CODEC_FLAG_TRUNCATED;
  }
#else
  // Moderne : pas besoin d’avcodec_register_all, ni de flags TRUNCATED
  // (Optionnel) parser si tu reçois des fragments Annex B
  // parser = av_parser_init(codec->id);
  // if (!parser) throw std::runtime_error("parser init failed");
#endif

  frame = av_frame_alloc();
  pkt   = av_packet_alloc();
  if (!frame || !pkt) throw std::runtime_error("alloc frame/packet failed");

  if (avcodec_open2(context, codec, nullptr) < 0)
    throw std::runtime_error("avcodec_open2 failed");
}

H264Decoder::~H264Decoder() {
#if !USE_OLD_FFMPEG
  if (parser) av_parser_close(parser);
#endif
  if (pkt)    av_packet_free(&pkt);
  if (frame)  av_frame_free(&frame);
  if (context) avcodec_free_context(&context);
}

const AVFrame& H264Decoder::decode_frame() {
#if USE_OLD_FFMPEG
  int got_picture = 0;
  int nread = avcodec_decode_video2(context, frame, &got_picture, pkt);
  if (nread < 0 || !got_picture) {
    throw std::runtime_error("decode failed or no frame");
  }
  return *frame;
#else
  if (int s = avcodec_send_packet(context, pkt); s < 0) {
    throw std::runtime_error("avcodec_send_packet failed");
  }
  int r = avcodec_receive_frame(context, frame);
  if (r == AVERROR(EAGAIN) || r == AVERROR_EOF) {
    throw std::runtime_error("no frame available yet");
  } else if (r < 0) {
    throw std::runtime_error("avcodec_receive_frame failed");
  }
  return *frame;
#endif
}

// // Conversion en BGR24 via swscale (compatible ancien/nouveau)
// class ConverterRGB24 {
// public:
//   ConverterRGB24() = default;
//   ~ConverterRGB24();

//   const AVFrame& convert(const AVFrame& in, ubyte* out_rgb);
//   int predict_size(int w, int h) { return w * h * 3; } // BGR24

// private:
//   SwsContext* sws = nullptr;
//   AVFrame* framergb = nullptr;
//   int last_w = 0, last_h = 0;
// };

// ConverterRGB24::~ConverterRGB24() {
//   if (framergb) av_frame_free(&framergb);
//   if (sws) sws_freeContext(sws);
// }

const AVFrame& ConverterRGB24::convert(const AVFrame& in, ubyte* out_rgb) {
  const int w = in.width, h = in.height;
  if (!framergb) framergb = av_frame_alloc();

  if (!sws || w != last_w || h != last_h) {
    if (sws) { sws_freeContext(sws); sws = nullptr; }
    sws = sws_getContext(
      w, h, static_cast<AVPixelFormat>(in.format),
      w, h, AV_PIX_FMT_BGR24,
      SWS_BILINEAR, nullptr, nullptr, nullptr
    );
    last_w = w; last_h = h;
  }

  // Associe 'out_rgb' aux buffers de 'framergb'
  if (av_image_fill_arrays(
        framergb->data, framergb->linesize,
        out_rgb, AV_PIX_FMT_BGR24, w, h, 1) < 0) {
    throw std::runtime_error("av_image_fill_arrays failed");
  }

  sws_scale(
    sws,
    in.data, in.linesize, 0, h,
    framergb->data, framergb->linesize
  );

  framergb->width  = w;
  framergb->height = h;
  framergb->format = AV_PIX_FMT_BGR24;
  return *framergb;
}

ssize_t H264Decoder::parse(const unsigned char* in_data, ssize_t in_size) {
  if (!pkt) return -1;
  pkt->data = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(in_data));
  pkt->size = static_cast<int>(in_size);
  return in_size;  // ou le nombre d’octets réellement consommés si tu parsers
}


bool H264Decoder::is_frame_available() const {
  return true;
}