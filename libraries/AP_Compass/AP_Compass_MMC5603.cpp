/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Compass_MMC5603.h"

#if AP_COMPASS_MMC5603_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <utility>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

#define MMC5603_DEBUG 0

#define REG_XOUT0          0x00
#define REG_STATUS          0x18
#define REG_ODR             0x1A
#define REG_CONTROL0        0x1B
#define REG_CONTROL1        0x1C
#define REG_CONTROL2        0x1D
#define REG_PRODUCT_ID      0x39

// bits in REG_CONTROL0
#define REG_CONTROL0_CMM_FREQ_EN (1 << 7)
#define REG_CONTROL0_AUTO_SR (1 << 5)
#define REG_CONTROL0_TMM     (1 << 0)
#define REG_CONTROL1_SWRST  (1 << 7)

#define REG_CONTROL2_CMM_EN (1 << 4)
#define REG_CONTROL2_EN_PRD_SET (1 << 3)
#define REG_CONTROL2_PRD_SET_SHIFT 0

#define REG_STATUS_MM_DONE (1 << 6)
#define REG_STATUS_SAT_SENSOR (1 << 5)

#define MMC5603_PRODUCT_ID (1 << 4)

AP_Compass_Backend *AP_Compass_MMC5603::probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_MMC5603 *sensor = new AP_Compass_MMC5603(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_MMC5603::AP_Compass_MMC5603(AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                       bool _force_external,
                                       enum Rotation _rotation)
    : dev(std::move(_dev))
    , force_external(_force_external)
    , rotation(_rotation)
{
}

static Vector3f field_vector(const uint8_t *magdata){
    const uint16_t zero_offset = 32768; // 16 bit mode (top 16 bits of 20-bit output)
    Vector3f ret = {
        float(((uint16_t)magdata[0] << 8) | magdata[1]) - zero_offset,
        float(((uint16_t)magdata[2] << 8) | magdata[3]) - zero_offset,
        float(((uint16_t)magdata[4] << 8) | magdata[5]) - zero_offset
    };
    return ret;
}

static float median3(float a, float b, float c)
{
    if (a > b) {
        const float t = a; a = b; b = t;
    }
    if (b > c) {
        const float t = b; b = c; c = t;
    }
    if (a > b) {
        const float t = a; a = b; b = t;
    }
    return b;
}

static bool raw_data_valid(const uint8_t *magdata)
{
    // Reject clearly invalid reads (bus glitches often yield 0xFF/0x00).
    for (uint8_t i = 0; i < 6; i++) {
        if (magdata[i] == 0xFF || magdata[i] == 0x00) {
            return false;
        }
    }
    return true;
}

bool AP_Compass_MMC5603::get_measurement(Vector3f &ret){
    uint8_t data[6];
    uint8_t status;
    if (!dev->read_registers(REG_STATUS, &status, 1)) {
        state = MMCState::STATE_MEASURE;
        return false;
    }

    // In continuous mode, read the latest sample even if not-ready is set.
    if (!dev->read_registers(REG_XOUT0, data, sizeof(data))) {
        state = MMCState::STATE_MEASURE;
        return false;
    }

    if (!raw_data_valid(data)) {
        // One retry if we saw a clearly invalid burst.
        if (!dev->read_registers(REG_XOUT0, data, sizeof(data)) || !raw_data_valid(data)) {
            return false;
        }
    }
#if MMC5603_DEBUG
    if (now_ms - last_debug_ms > 1000U) {
        last_debug_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "MMC5603 st=%02x d=%02x %02x %02x %02x %02x %02x",
                      status,
                      data[0], data[1], data[2], data[3], data[4], data[5]);
    }
#endif
    ret = field_vector(data);
    return true;
}

bool AP_Compass_MMC5603::init()
{
    dev->get_semaphore()->take_blocking();

    dev->set_retries(10);
    
    uint8_t whoami = 0;
    for (uint8_t i = 0; i < 5 && whoami != MMC5603_PRODUCT_ID; i++) {
        dev->read_registers(REG_PRODUCT_ID, &whoami, 1);
        if (whoami != MMC5603_PRODUCT_ID) {
            hal.scheduler->delay(5);
        }
    }
    if (whoami != MMC5603_PRODUCT_ID) {
        // not a MMC5603
        dev->get_semaphore()->give();
        return false;
    }

    // reset sensor
    dev->write_register(REG_CONTROL1, REG_CONTROL1_SWRST);
    hal.scheduler->delay(50);
    
    dev->write_register(REG_CONTROL0, 0x00);
    dev->write_register(REG_CONTROL1, 0x00);
    dev->write_register(REG_CONTROL2, 0x00);

    // configure continuous mode at 75Hz with periodic set/reset
    const uint8_t odr_hz = 75;
    const uint8_t prd_set_1000 = 6;
    dev->write_register(REG_ODR, odr_hz);
    dev->write_register(REG_CONTROL0, REG_CONTROL0_CMM_FREQ_EN | REG_CONTROL0_AUTO_SR);
    dev->write_register(REG_CONTROL2,
                        REG_CONTROL2_CMM_EN |
                        REG_CONTROL2_EN_PRD_SET |
                        (prd_set_1000 << REG_CONTROL2_PRD_SET_SHIFT));
    
    dev->get_semaphore()->give();

    /* register the compass instance in the frontend */
    dev->set_device_type(DEVTYPE_MMC5603);
    if (!register_compass(dev->get_bus_id(), compass_instance)) {
        return false;
    }
    
    set_dev_id(compass_instance, dev->get_bus_id());

    printf("Found a MMC5603 on 0x%x as compass %u\n", unsigned(dev->get_bus_id()), compass_instance);
    
    set_rotation(compass_instance, rotation);

    if (force_external) {
        set_external(compass_instance, true);
    }
    
    dev->set_retries(1);
    
    // call timer() at 75Hz
    dev->register_periodic_callback(13333,
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_MMC5603::timer, void));

    // wait 250ms for the compass to make it's initial readings
    hal.scheduler->delay(250);
    
    return true;
}

void AP_Compass_MMC5603::timer()
{
    
    const uint16_t sensitivity = 1024U; // counts per Gauss, 16 bit mode
    constexpr float counts_to_milliGauss = -1.0e3f / sensitivity;
    constexpr float filter_alpha = 0.1f;
    constexpr float max_delta_mg = 50.0f;
    constexpr uint8_t max_spike_rejects = 2;

    switch (state) {
    case MMCState::STATE_MEASURE: {
        Vector3f field;
        if (!get_measurement(field)) {
            break;
        }
        field *= counts_to_milliGauss;
        const bool calibrating = _compass.is_calibrating();
        if (raw_hist_count < 2) {
            raw_hist[raw_hist_count++] = field;
        } else {
            raw_hist[0] = raw_hist[1];
            raw_hist[1] = field;
        }
        if (raw_hist_count >= 2) {
            const Vector3f a = raw_hist[0];
            const Vector3f b = raw_hist[1];
            const Vector3f c = field;
            field.x = median3(a.x, b.x, c.x);
            field.y = median3(a.y, b.y, c.y);
            field.z = median3(a.z, b.z, c.z);
        }
        if (!calibrating) {
            if (have_last_field) {
                if ((field - last_field).length() > max_delta_mg) {
                    // Allow large changes only if they persist for a few samples.
                    if (spike_rejects++ < max_spike_rejects) {
                        return;
                    }
                    spike_rejects = 0;
                } else {
                    spike_rejects = 0;
                }
                field = last_field * (1.0f - filter_alpha) + field * filter_alpha;
            } else {
                have_last_field = true;
            }
            last_field = field;
        }
        accumulate_sample(field, compass_instance);
        break;
    }
    }
}

void AP_Compass_MMC5603::read()
{
    drain_accumulated_samples(compass_instance);
}

#endif  // AP_COMPASS_MMC5603_ENABLED
