#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sharpa::mujoco::plugins::wave_hand
{

constexpr std::size_t kLtcHeaderSize = 12;
constexpr std::size_t kLtcCrcSize = 4;
constexpr std::size_t kDefaultLtcMaxPayloadSize = 64U * 1024U;

struct LtcFrame
{
    std::uint16_t command = 0;
    std::uint8_t response_code = 0;
    std::vector<std::uint8_t> payload{};
};

bool encode_ltc_frame(const LtcFrame& frame,
                      std::vector<std::uint8_t>& output,
                      std::size_t max_payload_size = kDefaultLtcMaxPayloadSize);
bool decode_ltc_frame(const std::uint8_t* data,
                      std::size_t size,
                      LtcFrame& frame,
                      std::size_t max_payload_size = kDefaultLtcMaxPayloadSize);

class LtcStreamParser
{
   public:
    explicit LtcStreamParser(std::size_t max_payload_size = kDefaultLtcMaxPayloadSize);

    // Appends arbitrary TCP bytes and returns every complete, valid frame found.
    // Invalid headers, impossible lengths, and bad-CRC frames are skipped while
    // retaining enough trailing bytes to resynchronize on the next BB EE header.
    std::vector<LtcFrame> push(const std::uint8_t* data, std::size_t size);
    std::vector<LtcFrame> push(const std::vector<std::uint8_t>& data);

    void reset();
    std::size_t buffered_size() const noexcept;

   private:
    std::size_t max_payload_size_;
    std::vector<std::uint8_t> buffer_;
};

}  // namespace sharpa::mujoco::plugins::wave_hand
