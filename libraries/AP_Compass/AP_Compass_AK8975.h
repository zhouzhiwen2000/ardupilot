#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AuxiliaryBus;
class AuxiliaryBusSlave;
class AP_InertialSensor;
class AP_AK8975_BusDriver;

class AP_Compass_AK8975 : public AP_Compass_Backend
{
public:
    /* Probe for AK8975 standalone on I2C/SPI bus */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    /* Probe for AK8975 on auxiliary bus of MPU9250, connected through I2C */
    static AP_Compass_Backend *probe_mpu9250(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                             enum Rotation rotation);

    /* Probe for AK8975 on auxiliary bus of MPU9250, connected through SPI */
    static AP_Compass_Backend *probe_mpu9250(uint8_t mpu9250_instance,
                                             enum Rotation rotation);

    static constexpr const char *name = "AK8975";

    virtual ~AP_Compass_AK8975();

    void read() override;

private:
    AP_Compass_AK8975(AP_AK8975_BusDriver *bus,
                      bool force_external,
                      enum Rotation rotation);

    bool init();
    void _make_factory_sensitivity_adjustment(Vector3f &field) const;
    void _make_adc_sensitivity_adjustment(Vector3f &field) const;

    bool _setup_mode();
    bool _check_id();
    bool _calibrate();

    void _update();

    AP_AK8975_BusDriver *_bus;

    float _magnetometer_ASA[3] {0, 0, 0};

    uint8_t _compass_instance;
    bool _initialized;
    enum Rotation _rotation;
    bool _force_external;
};

class AP_AK8975_BusDriver
{
public:
    virtual ~AP_AK8975_BusDriver() { }

    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) = 0;
    virtual bool register_read(uint8_t reg, uint8_t *val) = 0;
    virtual bool register_write(uint8_t reg, uint8_t val) = 0;

    virtual AP_HAL::Semaphore  *get_semaphore() = 0;

    virtual bool configure() { return true; }
    virtual bool start_measurements() { return true; }
    virtual AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t, AP_HAL::Device::PeriodicCb) = 0;

    // set device type within a device class
    virtual void set_device_type(uint8_t devtype) = 0;

    // return 24 bit bus identifier
    virtual uint32_t get_bus_id(void) const = 0;
};

class AP_AK8975_BusDriver_HALDevice: public AP_AK8975_BusDriver
{
public:
    AP_AK8975_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::Device> dev);

    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    virtual bool register_read(uint8_t reg, uint8_t *val) override;
    virtual bool register_write(uint8_t reg, uint8_t val) override;
    virtual bool start_measurements() override;
    virtual AP_HAL::Semaphore  *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    // set device type within a device class
    void set_device_type(uint8_t devtype) override {
        _dev->set_device_type(devtype);
    }

    // return 24 bit bus identifier
    uint32_t get_bus_id(void) const override {
        return _dev->get_bus_id();
    }
    
private:
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
};
