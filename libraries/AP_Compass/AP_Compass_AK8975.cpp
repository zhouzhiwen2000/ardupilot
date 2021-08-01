/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <assert.h>
#include <utility>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_Compass_AK8975.h"
#include <AP_InertialSensor/AP_InertialSensor_Invensense.h>

#define AK8975_WIA                                      0x00
#        define AK8975_Device_ID                        0x48

#define AK8975_HXL                                      0x03

/* bit definitions for AK8975 CNTL1 */
#define AK8975_CNTL1                                    0x0A
#        define    AK8975_SELFTEST_MODE                 0x08
#        define    AK8975_POWERDOWN_MODE                0x00
#        define    AK8975_FUSE_MODE                     0x0f
#        define    AK8975_SINGLE_MEASUREMENT_MODE       0x01


#define AK8975_ASAX                                     0x10

#define AK8975_MILLIGAUSS_SCALE 10.0f

struct PACKED sample_regs {
    int16_t val[3];
    uint8_t st2;
};

extern const AP_HAL::HAL &hal;

AP_Compass_AK8975::AP_Compass_AK8975(AP_AK8975_BusDriver *bus,
                                     bool force_external,
                                     enum Rotation rotation)
    : _bus(bus)
    , _rotation(rotation)
    , _force_external(force_external)
{
}

AP_Compass_AK8975::~AP_Compass_AK8975()
{
    delete _bus;
}

AP_Compass_Backend *AP_Compass_AK8975::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                             bool force_external,
                                             enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        dev->set_read_flag(0x80);
    }

    AP_AK8975_BusDriver *bus = new AP_AK8975_BusDriver_HALDevice(std::move(dev));
    if (!bus) {
        return nullptr;
    }

    AP_Compass_AK8975 *sensor = new AP_Compass_AK8975(bus, force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}


bool AP_Compass_AK8975::init()
{
    AP_HAL::Semaphore *bus_sem = _bus->get_semaphore();

    if (!bus_sem) {
        return false;
    }
    _bus->get_semaphore()->take_blocking();

    if (!_bus->configure()) {
        hal.console->printf("AK8975: Could not configure the bus\n");
        goto fail;
    }

    if (!_check_id()) {
        hal.console->printf("AK8975: Wrong id\n");
        goto fail;
    }

    if (!_calibrate()) {
        hal.console->printf("AK8975: Could not read calibration data\n");
        goto fail;
    }

    if (!_setup_mode()) {
        hal.console->printf("AK8975: Could not setup mode\n");
        goto fail;
    }

    if (!_bus->start_measurements()) {
        hal.console->printf("AK8975: Could not start measurements\n");
        goto fail;
    }

    _initialized = true;

    /* register the compass instance in the frontend */
    _bus->set_device_type(DEVTYPE_AK8975);
    if (!register_compass(_bus->get_bus_id(), _compass_instance)) {
        goto fail;
    }
    set_dev_id(_compass_instance, _bus->get_bus_id());

    set_rotation(_compass_instance, _rotation);
    if (_force_external) {
        set_external(_compass_instance, true);
    }
    bus_sem->give();
    hal.scheduler->delay(30);//wait for first result
    _bus->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Compass_AK8975::_update, void));

    return true;

fail:
    bus_sem->give();
    return false;
}

void AP_Compass_AK8975::read()
{
    if (!_initialized) {
        return;
    }

    drain_accumulated_samples(_compass_instance);
}

void AP_Compass_AK8975::_make_adc_sensitivity_adjustment(Vector3f& field) const
{
    static const float ADC_16BIT_RESOLUTION = 0.3f;

    field *= ADC_16BIT_RESOLUTION;
}

void AP_Compass_AK8975::_make_factory_sensitivity_adjustment(Vector3f& field) const
{
    field.x *= _magnetometer_ASA[0];
    field.y *= _magnetometer_ASA[1];
    field.z *= _magnetometer_ASA[2];
}

void AP_Compass_AK8975::_update()
{
    struct sample_regs regs;
    Vector3f raw_field;

    if (!_bus->block_read(AK8975_HXL, (uint8_t *) &regs, sizeof(regs))) {
        return;
    }

    /* Check for overflow. See AK8975's datasheet, section
     * 6.4.3.6 - Magnetic Sensor Overflow. */
    if ((regs.st2 & 0x08)) {
        return;
    }

    raw_field = Vector3f(regs.val[0], regs.val[1], regs.val[2]);

    if (is_zero(raw_field.x) && is_zero(raw_field.y) && is_zero(raw_field.z)) {
        return;
    }

    _make_factory_sensitivity_adjustment(raw_field);
    _make_adc_sensitivity_adjustment(raw_field);
    raw_field *= AK8975_MILLIGAUSS_SCALE;

    accumulate_sample(raw_field, _compass_instance, 10);

    if (!_bus->start_measurements()) {
        return;
    }
}

bool AP_Compass_AK8975::_check_id()
{
    for (int i = 0; i < 5; i++) {
        uint8_t deviceid = 0;

        /* Read AK8975's id */
        if (_bus->register_read(AK8975_WIA, &deviceid) &&
            deviceid == AK8975_Device_ID) {
            return true;
        }
    }

    return false;
}

bool AP_Compass_AK8975::_setup_mode() {
    return _bus->register_write(AK8975_CNTL1, AK8975_POWERDOWN_MODE);
}


bool AP_Compass_AK8975::_calibrate()
{
    /* Enable FUSE-mode in order to be able to read calibration data */
    _bus->register_write(AK8975_CNTL1, AK8975_FUSE_MODE);

    uint8_t response[3];

    _bus->block_read(AK8975_ASAX, response, 3);

    for (int i = 0; i < 3; i++) {
        float data = response[i];
        _magnetometer_ASA[i] = ((data - 128) / 256 + 1);
    }

    return true;
}

/* AP_HAL::Device implementation of the AK8975 */
AP_AK8975_BusDriver_HALDevice::AP_AK8975_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : _dev(std::move(dev))
{
}

bool AP_AK8975_BusDriver_HALDevice::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

bool AP_AK8975_BusDriver_HALDevice::register_read(uint8_t reg, uint8_t *val)
{
    return _dev->read_registers(reg, val, 1);
}

bool AP_AK8975_BusDriver_HALDevice::register_write(uint8_t reg, uint8_t val)
{
    return _dev->write_register(reg, val);
}
bool AP_AK8975_BusDriver_HALDevice::start_measurements()
{
    return _dev->write_register(AK8975_CNTL1, AK8975_SINGLE_MEASUREMENT_MODE);
}

AP_HAL::Semaphore *AP_AK8975_BusDriver_HALDevice::get_semaphore()
{
    return _dev->get_semaphore();
}

AP_HAL::Device::PeriodicHandle AP_AK8975_BusDriver_HALDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _dev->register_periodic_callback(period_usec, cb);
}