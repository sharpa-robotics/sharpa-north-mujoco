#include "sharpa_mujoco/plugins/wave_hand/ltc_protocol.h"

#include <limits>
#include <utility>

#include "sharpa_mujoco/plugins/wave_hand/wave_protocol.h"

namespace sharpa::mujoco::plugins::wave_hand
{
namespace
{

constexpr std::uint8_t kIdentifier0 = 0xBB;
constexpr std::uint8_t kIdentifier1 = 0xEE;
constexpr std::uint8_t kLtcVersion = 0;

void write_u16_be(std::uint8_t* output, std::uint16_t value)
{
    output[0] = static_cast<std::uint8_t>(value >> 8U);
    output[1] = static_cast<std::uint8_t>(value);
}

void write_u32_be(std::uint8_t* output, std::uint32_t value)
{
    output[0] = static_cast<std::uint8_t>(value >> 24U);
    output[1] = static_cast<std::uint8_t>(value >> 16U);
    output[2] = static_cast<std::uint8_t>(value >> 8U);
    output[3] = static_cast<std::uint8_t>(value);
}

std::uint16_t read_u16_be(const std::uint8_t* input)
{
    return (static_cast<std::uint16_t>(input[0]) << 8U) |
           static_cast<std::uint16_t>(input[1]);
}

std::uint32_t read_u32_be(const std::uint8_t* input)
{
    return (static_cast<std::uint32_t>(input[0]) << 24U) |
           (static_cast<std::uint32_t>(input[1]) << 16U) |
           (static_cast<std::uint32_t>(input[2]) << 8U) |
           static_cast<std::uint32_t>(input[3]);
}

bool valid_header(const std::uint8_t* data,
                  std::size_t max_payload_size,
                  std::size_t& payload_size,
                  std::size_t& frame_size)
{
    if (data[0] != kIdentifier0 || data[1] != kIdentifier1 ||
        data[2] != kLtcVersion || data[3] != 0 || data[7] != 0)
    {
        return false;
    }

    const std::uint32_t length = read_u32_be(data + 8);
    if (length < kLtcCrcSize)
    {
        return false;
    }
    payload_size = static_cast<std::size_t>(length - kLtcCrcSize);
    if (payload_size > max_payload_size ||
        length > std::numeric_limits<std::size_t>::max() - kLtcHeaderSize)
    {
        return false;
    }
    frame_size = kLtcHeaderSize + static_cast<std::size_t>(length);
    return true;
}

}  // namespace

bool encode_ltc_frame(const LtcFrame& frame,
                      std::vector<std::uint8_t>& output,
                      std::size_t max_payload_size)
{
    if (frame.payload.size() > max_payload_size ||
        frame.payload.size() >
            std::numeric_limits<std::uint32_t>::max() - kLtcCrcSize)
    {
        return false;
    }

    output.assign(kLtcHeaderSize + frame.payload.size() + kLtcCrcSize, 0);
    output[0] = kIdentifier0;
    output[1] = kIdentifier1;
    output[2] = kLtcVersion;
    output[3] = 0;
    write_u16_be(output.data() + 4, frame.command);
    output[6] = frame.response_code;
    output[7] = 0;
    write_u32_be(output.data() + 8,
                 static_cast<std::uint32_t>(frame.payload.size() + kLtcCrcSize));
    std::copy(frame.payload.begin(), frame.payload.end(),
              output.begin() + static_cast<std::ptrdiff_t>(kLtcHeaderSize));
    write_u32_be(output.data() + output.size() - kLtcCrcSize,
                 ieee_crc32(output.data(), output.size() - kLtcCrcSize));
    return true;
}

bool decode_ltc_frame(const std::uint8_t* data,
                      std::size_t size,
                      LtcFrame& frame,
                      std::size_t max_payload_size)
{
    if (data == nullptr || size < kLtcHeaderSize + kLtcCrcSize)
    {
        return false;
    }
    std::size_t payload_size = 0;
    std::size_t frame_size = 0;
    if (!valid_header(data, max_payload_size, payload_size, frame_size) ||
        size != frame_size ||
        read_u32_be(data + size - kLtcCrcSize) !=
            ieee_crc32(data, size - kLtcCrcSize))
    {
        return false;
    }

    LtcFrame decoded;
    decoded.command = read_u16_be(data + 4);
    decoded.response_code = data[6];
    decoded.payload.assign(data + kLtcHeaderSize,
                           data + kLtcHeaderSize + payload_size);
    frame = std::move(decoded);
    return true;
}

LtcStreamParser::LtcStreamParser(std::size_t max_payload_size)
    : max_payload_size_(max_payload_size)
{
}

std::vector<LtcFrame> LtcStreamParser::push(const std::uint8_t* data,
                                            std::size_t size)
{
    std::vector<LtcFrame> frames;
    if (data == nullptr)
    {
        return frames;
    }
    if (size != 0)
    {
        buffer_.insert(buffer_.end(), data, data + size);
    }

    std::size_t consumed = 0;
    while (buffer_.size() - consumed >= 2)
    {
        std::size_t marker = consumed;
        while (marker + 1 < buffer_.size() &&
               (buffer_[marker] != kIdentifier0 ||
                buffer_[marker + 1] != kIdentifier1))
        {
            ++marker;
        }
        if (marker + 1 >= buffer_.size())
        {
            consumed = buffer_.size() -
                       (buffer_.back() == kIdentifier0 ? 1U : 0U);
            break;
        }
        consumed = marker;
        const std::size_t available = buffer_.size() - consumed;
        if (available < kLtcHeaderSize)
        {
            break;
        }

        std::size_t payload_size = 0;
        std::size_t frame_size = 0;
        if (!valid_header(buffer_.data() + consumed, max_payload_size_,
                          payload_size, frame_size))
        {
            ++consumed;
            continue;
        }
        if (available < frame_size)
        {
            break;
        }

        LtcFrame frame;
        if (decode_ltc_frame(buffer_.data() + consumed, frame_size, frame,
                             max_payload_size_))
        {
            frames.push_back(std::move(frame));
            consumed += frame_size;
        }
        else
        {
            // Advance one byte so a BB EE sequence inside a corrupt frame can
            // become the next synchronization candidate.
            ++consumed;
        }
    }
    if (consumed != 0)
    {
        buffer_.erase(buffer_.begin(),
                      buffer_.begin() + static_cast<std::ptrdiff_t>(consumed));
    }
    return frames;
}

std::vector<LtcFrame> LtcStreamParser::push(
    const std::vector<std::uint8_t>& data)
{
    return push(data.data(), data.size());
}

void LtcStreamParser::reset()
{
    buffer_.clear();
}

std::size_t LtcStreamParser::buffered_size() const noexcept
{
    return buffer_.size();
}

}  // namespace sharpa::mujoco::plugins::wave_hand
