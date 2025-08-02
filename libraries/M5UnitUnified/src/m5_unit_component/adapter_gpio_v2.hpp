/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file adapter_gpio_v2.hpp
  @brief Adapters to treat M5HAL and GPIO in the same way using RMT v2
  @note  Currently handles GPIO directly, but will handle via M5HAL in the future
*/
#ifndef M5_UNIT_COMPONENT_ADAPTER_GPIO_V2_HPP
#define M5_UNIT_COMPONENT_ADAPTER_GPIO_V2_HPP

#include "identify_functions.hpp"
#include "types.hpp"
#include "adapter_gpio.hpp"

#if defined(M5_UNIT_UNIFIED_USING_RMT_V2)
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_encoder.h>
#include <driver/gpio.h>

namespace m5 {
namespace unit {

/*!
  @class m5::unit::AdapterGPIO
  @brief GPIO access adapter
 */
class AdapterGPIO : public AdapterGPIOBase {
public:
    AdapterGPIO(const int8_t rx_pin, const int8_t tx_pin);
};

}  // namespace unit
}  // namespace m5
#endif
#endif
