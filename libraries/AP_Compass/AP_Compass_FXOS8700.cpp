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

#include "AP_Compass_FXOS8700.h"
#include <AP_InertialSensor/AP_InertialSensor_Invensense.h>

#define FXOS8700_WIA                                      0x0D
#        define FXOS8700_Device_ID                        0xC7

#define CTRL_REG1                                         0x2A
#        define ODR_800HZ                                 (0x00<<3)
#        define ODR_400HZ                                 (0x01<<3)
#        define ODR_200HZ                                 (0x02<<3)
#        define ODR_100HZ                                 (0x03<<3)
#        define ODR_50HZ                                  (0x04<<3)
#        define ODR_12_5HZ                                (0x05<<3)
#        define ACTIVE                                    (0x01<<0)
#define M_CTRL_REG1                                       0x5B
#define M_OUT                                             0x33
#define FXOS8700_MILLIGAUSS_SCALE 1.0f

struct PACKED sample_regs {
    be16_t  val[3];
};

extern const AP_HAL::HAL &hal;

AP_Compass_FXOS8700::AP_Compass_FXOS8700(AP_FXOS8700_BusDriver *bus,
                                     bool force_external,
                                     enum Rotation rotation)
    : _bus(bus)
    , _rotation(rotation)
    , _force_external(force_external)
{
}

AP_Compass_FXOS8700::~AP_Compass_FXOS8700()
{
    delete _bus;
}

AP_Compass_Backend *AP_Compass_FXOS8700::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                             bool force_external,
                                             enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        dev->set_read_flag(0x80);
    }

    AP_FXOS8700_BusDriver *bus = new AP_FXOS8700_BusDriver_HALDevice(std::move(dev));
    if (!bus) {
        return nullptr;
    }

    AP_Compass_FXOS8700 *sensor = new AP_Compass_FXOS8700(bus, force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}


bool AP_Compass_FXOS8700::init()
{
    AP_HAL::Semaphore *bus_sem = _bus->get_semaphore();

    if (!bus_sem) {
        return false;
    }
    _bus->get_semaphore()->take_blocking();

    if (!_bus->configure()) {
        hal.console->printf("FXOS8700: Could not configure the bus\n");
        goto fail;
    }

    if (!_check_id()) {
        hal.console->printf("FXOS8700: Wrong id\n");
        goto fail;
    }

    if (!_setup_mode()) {
        hal.console->printf("FXOS8700: Could not setup mode\n");
        goto fail;
    }

    _initialized = true;

    /* register the compass instance in the frontend */
    _bus->set_device_type(DEVTYPE_FXOS8700);
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
    _bus->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Compass_FXOS8700::_update, void));//100hz

    return true;

fail:
    bus_sem->give();
    return false;
}

void AP_Compass_FXOS8700::read()
{
    if (!_initialized) {
        return;
    }

    drain_accumulated_samples(_compass_instance);
}

void AP_Compass_FXOS8700::_update()
{
    struct sample_regs regs;
    Vector3f raw_field;

    if (!_bus->block_read(M_OUT, (uint8_t *) &regs, sizeof(regs))) {
        return;
    }

    raw_field = Vector3f(be16toh(regs.val[0]), be16toh(regs.val[1]), be16toh(regs.val[2]));

    if (is_zero(raw_field.x) && is_zero(raw_field.y) && is_zero(raw_field.z)) {
        return;
    }
    raw_field *= FXOS8700_MILLIGAUSS_SCALE;

    accumulate_sample(raw_field, _compass_instance, 10);
}

bool AP_Compass_FXOS8700::_check_id()
{
    for (int i = 0; i < 5; i++) {
        uint8_t deviceid = 0;

        /* Read FXOS8700's id */
        if (_bus->register_read(FXOS8700_WIA, &deviceid) &&
            deviceid == FXOS8700_Device_ID) {
            return true;
        }
    }

    return false;
}

bool AP_Compass_FXOS8700::_setup_mode() {
    bool success_setup=_bus->register_write(CTRL_REG1, ODR_100HZ|ACTIVE);
    bool success_init=_bus->register_write(M_CTRL_REG1, (7<<2)|(1<<0));
    return success_init&&success_setup;

}

/* AP_HAL::Device implementation of the FXOS8700 */
AP_FXOS8700_BusDriver_HALDevice::AP_FXOS8700_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : _dev(std::move(dev))
{
}

bool AP_FXOS8700_BusDriver_HALDevice::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

bool AP_FXOS8700_BusDriver_HALDevice::register_read(uint8_t reg, uint8_t *val)
{
    return _dev->read_registers(reg, val, 1);
}

bool AP_FXOS8700_BusDriver_HALDevice::register_write(uint8_t reg, uint8_t val)
{
    return _dev->write_register(reg, val);
}

AP_HAL::Semaphore *AP_FXOS8700_BusDriver_HALDevice::get_semaphore()
{
    return _dev->get_semaphore();
}

AP_HAL::Device::PeriodicHandle AP_FXOS8700_BusDriver_HALDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _dev->register_periodic_callback(period_usec, cb);
}