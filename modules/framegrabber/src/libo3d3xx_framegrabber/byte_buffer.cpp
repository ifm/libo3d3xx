/*
 * Copyright (C) 2016 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "o3d3xx_framegrabber/byte_buffer.hpp"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
#include <glog/logging.h>

const std::size_t o3d3xx::IMG_TICKET_SZ = 16;
const std::size_t o3d3xx::IMG_CHUNK_HEADER_SZ = 36;

bool
o3d3xx::verify_ticket_buffer(const std::vector<std::uint8_t>& buff)
{
  return buff.size() == o3d3xx::IMG_TICKET_SZ &&
         buff.at(4) == 'L' &&
         buff.at(14) == '\r' &&
         buff.at(15) == '\n';
}

bool
o3d3xx::verify_image_buffer(const std::vector<std::uint8_t>& buff)
{
  std::size_t buff_sz = buff.size();

  return buff_sz > 8 &&
         std::string(buff.begin()+4, buff.begin()+8) == "star" &&
         std::string(buff.end()-6, buff.end()-2) == "stop" &&
         buff.at(buff_sz - 2) == '\r' &&
         buff.at(buff_sz - 1) == '\n';
}

std::size_t
o3d3xx::get_image_buffer_size(const std::vector<std::uint8_t>& buff)
{
  return std::stoi(std::string(buff.begin()+5, buff.end()));
}

std::size_t
o3d3xx::get_chunk_index(const std::vector<std::uint8_t>& buff,
                        o3d3xx::image_chunk chunk_type)
{
  std::size_t idx = 8; // start of first chunk

  while (buff.begin()+idx < buff.end())
    {
      if (static_cast<std::uint32_t>(chunk_type) ==
          o3d3xx::mkval<std::uint32_t>(buff.data()+idx))
        {
          return idx;
        }

      // move to the beginning of the next chunk
      idx += o3d3xx::mkval<std::uint32_t>(buff.data()+idx+4);
    }

  //throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
  return std::numeric_limits<std::size_t>::max();
}

std::size_t
o3d3xx::get_num_bytes_in_pixel_format(o3d3xx::pixel_format f)
{
  switch (f)
    {
    case o3d3xx::pixel_format::FORMAT_8U:
      return 1;

    case o3d3xx::pixel_format::FORMAT_8S:
      return 1;

    case o3d3xx::pixel_format::FORMAT_16U:
      return 2;

    case o3d3xx::pixel_format::FORMAT_16S:
      return 2;

    case o3d3xx::pixel_format::FORMAT_32U:
      return 4;

    case o3d3xx::pixel_format::FORMAT_32S:
      return 4;

    case o3d3xx::pixel_format::FORMAT_32F:
      return 4;

    case o3d3xx::pixel_format::FORMAT_64U:
      return 8;

    case o3d3xx::pixel_format::FORMAT_64F:
      return 8;

    case o3d3xx::pixel_format::FORMAT_16U2:
      return 2 * 2;

    case o3d3xx::pixel_format::FORMAT_32F3:
      return 4 * 3;

    default:
      return 0;
    }
}

o3d3xx::ByteBuffer::ByteBuffer()
  : dirty_(false)
{ }

o3d3xx::ByteBuffer::ByteBuffer(const o3d3xx::ByteBuffer& src_buff)
  : o3d3xx::ByteBuffer()
{
  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
}

o3d3xx::ByteBuffer&
o3d3xx::ByteBuffer::operator= (const o3d3xx::ByteBuffer& src_buff)
{
  if (this == &src_buff)
    {
      return *this;
    }

  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);

  return *this;
}

o3d3xx::ByteBuffer::~ByteBuffer()
{
  DLOG(INFO) << "Byte buffer dtor";
}

void
o3d3xx::ByteBuffer::SetBytes(std::vector<std::uint8_t>& buff,
                             bool copy)
{
  if (copy)
    {
      std::size_t sz = buff.size();
      this->bytes_.resize(sz);

      std::copy(buff.begin(),
                buff.begin() + sz,
                this->bytes_.begin());
    }
  else
    {
      buff.swap(this->bytes_);
    }

  this->_SetDirty(true);
}

void
o3d3xx::ByteBuffer::_SetDirty(bool flg) noexcept
{
  this->dirty_ = flg;
}

bool
o3d3xx::ByteBuffer::Dirty() const noexcept
{
  return this->dirty_;
}

std::vector<std::uint8_t>
o3d3xx::ByteBuffer::Bytes()
{
  return this->bytes_;
}

void
o3d3xx::ByteBuffer::Organize()
{
  // meant to act as a stub for subclassers
  if (! this->Dirty())
    {
      return;
    }

  // Theoretically, you would structure your data here
  // then when you are done, flag the buffer is "not dirty".

  this->_SetDirty(false);
}
