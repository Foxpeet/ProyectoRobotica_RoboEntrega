// Copyright (c) 2024, The Robot Web Tools Contributors
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "robo_entrega_web_video_server/vp9_streamer.hpp"

namespace robo_entrega_web_video_server
{

Vp9Streamer::Vp9Streamer(
  const robo_entrega_async_web_server_cpp::HttpRequest & request,
  robo_entrega_async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr node)
: LibavStreamer(request, connection, node, "webm", "libvpx-vp9", "video/webm")
{
}
Vp9Streamer::~Vp9Streamer()
{
}

void Vp9Streamer::initializeEncoder()
{
  // codec options set up to provide somehow reasonable performance in cost of poor quality
  // should be updated as soon as VP9 encoding matures
  av_opt_set_int(codec_context_->priv_data, "pass", 1, 0);
  av_opt_set_int(codec_context_->priv_data, "speed", 8, 0);
  av_opt_set_int(codec_context_->priv_data, "cpu-used", 4, 0);  // 8 is max
  av_opt_set_int(codec_context_->priv_data, "crf", 20, 0);      // 0..63 (higher is lower quality)
}

Vp9StreamerType::Vp9StreamerType()
: LibavStreamerType("webm", "libvpx-vp9", "video/webm")
{
}

std::shared_ptr<ImageStreamer> Vp9StreamerType::create_streamer(
  const robo_entrega_async_web_server_cpp::HttpRequest & request,
  robo_entrega_async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::SharedPtr node)
{
  return std::make_shared<Vp9Streamer>(request, connection, node);
}

}  // namespace robo_entrega_web_video_server
