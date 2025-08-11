#ifndef MVEC_LIB__MVEC_STATUS_MESSAGES_HPP_
#define MVEC_LIB__MVEC_STATUS_MESSAGES_HPP_

#include <stdint.h>
#include <array>
#include <chrono>
#include "socketcan_adapter/can_frame.hpp"
#include "mvec_lib/j1939_id.hpp"
#include "mvec_lib/can_bitwork.hpp"
#include "mvec_lib/mvec_constants.hpp"

namespace polymath::sygnal
{

class MvecFuseStatusMessage
{
public:
    MvecFuseStatusMessage();
    
    bool parse(const socketcan::CanFrame & frame);
    
    MvecFuseStatus get_fuse_status(uint8_t fuse_id) const;
    
    bool is_valid() const { return is_valid_; }
    
    std::chrono::steady_clock::time_point get_timestamp() const { return timestamp_; }

private:
    std::array<MvecFuseStatus, MvecHardware::MAX_FUSES> fuse_statuses_;
    std::chrono::steady_clock::time_point timestamp_;
    bool is_valid_;
};

class MvecRelayStatusMessage
{
public:
    MvecRelayStatusMessage();
    
    bool parse(const socketcan::CanFrame & frame);
    
    MvecRelayStatus get_relay_status(uint8_t relay_id) const;
    
    bool is_valid() const { return is_valid_; }
    
    std::chrono::steady_clock::time_point get_timestamp() const { return timestamp_; }

private:
    std::array<MvecRelayStatus, MvecHardware::MAX_RELAYS> relay_statuses_;
    std::chrono::steady_clock::time_point timestamp_;
    bool is_valid_;
};

class MvecErrorStatusMessage
{
public:
    MvecErrorStatusMessage();
    
    bool parse(const socketcan::CanFrame & frame);
    
    uint16_t get_error_bits() const { return error_bits_; }
    uint8_t get_grid_address() const { return grid_address_; }
    
    bool has_error(MvecErrorType error_type) const;
    
    bool is_valid() const { return is_valid_; }
    
    std::chrono::steady_clock::time_point get_timestamp() const { return timestamp_; }

private:
    uint8_t grid_address_;
    uint16_t error_bits_;
    std::chrono::steady_clock::time_point timestamp_;
    bool is_valid_;
};

} // polymath::sygnal

#endif // MVEC_LIB__MVEC_STATUS_MESSAGES_HPP_