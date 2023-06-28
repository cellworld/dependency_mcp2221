#include <mcp2221.h>
#include <libmcp2221.h>
#include <hidapi.h>
#include <iostream>


using namespace std;

namespace mcp2221 {
    size_t open_devices = 0;

    std::vector<Device_info> Device::get_devices() {
        vector<Device_info> device_info;
        struct hid_device_info* allDevices = hid_enumerate(MCP2221_DEFAULT_VID, MCP2221_DEFAULT_PID);
        struct hid_device_info* currentDevice;
        currentDevice = allDevices;

        while (currentDevice)
        {
            auto &di = device_info.emplace_back();
            di.interface_number = currentDevice->interface_number;
            di.release_number = currentDevice->release_number;
            di.product_id = currentDevice->product_id;
            di.product_string = (char *)currentDevice->product_string;
            di.manufacturer_string = (char *)currentDevice->manufacturer_string;
            di.path = currentDevice->path;
            di.serial_number = (char *)currentDevice->serial_number;
            di.vendor_id = currentDevice->vendor_id;
            di.usage = currentDevice->usage;
            di.usage_page = currentDevice->usage_page;
            currentDevice = currentDevice->next;
        }
        hid_free_enumeration(allDevices);
        return device_info;
    }
    Device::Device(string &path) :
            pin_values(MCP2221_GPIO_COUNT),
            path(path),
            pin_mode(MCP2221_GPIO_COUNT, NONE),
            adc_pins(MCP2221_GPIO_COUNT, 0),
            need_pooling(false),
            interrupt(NULL){
        for (auto &pv:pin_values) pv = false;
    }

    Device::~Device() {
        open_devices --;
        if (open_devices==0){
            busy.lock();
            libmcp2221::mcp2221_exit();
            busy.unlock();
        }
        need_pooling = false;
        if (pooling_thread.joinable()) pooling_thread.join();
    }

    void Device::set_direction(GPIO pin, MODE direction) {
        if (direction == INTERRUPT && pin != GPIO::pin1){
            cerr << "Interrupt can only be configured to GPIO pin 1 " << endl;
            exit(1);
        }
        pin_mode[pin] = direction;
    }

    void Device::open() {
        if (open_devices==0){
            libmcp2221::mcp2221_init();
        }
        open_devices ++;
        busy.lock();
        device = libmcp2221::mcp2221_open_device(path.c_str());
        busy.unlock();
        if (device==NULL){
            cerr << "Failed to open device at " << path << endl;
            exit(1);
        }
//        busy.lock();
//        auto res = mcp2221_reset(device);
//        busy.unlock();
//        if (res != libmcp2221::MCP2221_SUCCESS){
//            cerr << "Failed to reset the device" << endl;
//        }
        auto configuration = _config();
        busy.lock();
        libmcp2221::mcp2221_setGPIOConf(device, &configuration);
        libmcp2221::mcp2221_saveGPIOConf(device, &configuration);
        busy.unlock();
        if (need_pooling){
            pooling_thread = thread(&Device::_pooling_process, this);
        }
    }

    void Device::_pooling_process(Device *device) {
        vector<bool> pin_values(MCP2221_GPIO_COUNT);
        device->get_pins();
        for (size_t i=0;i<MCP2221_GPIO_COUNT;i++) pin_values[i] = device->pin_values[i];

        while (device->need_pooling) {
            int interrupt;
            device->busy.lock();
            auto res = mcp2221_readInterrupt(device->device, &interrupt);
            device->busy.unlock();
            if(res ==libmcp2221::MCP2221_SUCCESS) {
                if (interrupt && device->interrupt) {
                    device->busy.lock();
                    res = mcp2221_clearInterrupt(device->device);
                    device->busy.unlock();
                    if (device->interrupt) device->interrupt();
                    if (res != libmcp2221::MCP2221_SUCCESS) {
                        cerr << "failed to clear interrupt flag" << endl;
                    }
                }
            } else {
                cerr << "failed to retrieve interrupt state" << endl;
            }
            this_thread::sleep_for(500us);
        }
    }

    libmcp2221::mcp2221_gpioconfset_t Device::_config() {
        int gpio_in = 0;
        int gpio_out = 0;
        int gpio_adc = 0;
        int gpio_pwm = 0;
        int gpio_int = 0;
        int adc_pin_count = 0;
        for (size_t pin = 0; pin < MCP2221_GPIO_COUNT; pin++) {
            int gpio = get_gpio(pin);
            switch (pin_mode[pin]) {
                case INPUT:
                    gpio_in |= gpio;
                    break;
                case OUTPUT:
                    gpio_out |= gpio;
                    break;
                case PWM:
                    gpio_pwm |= gpio;
                    break;
                case ADC:
                    adc_pins[pin] = adc_pin_count++;
                    gpio_adc |= gpio;
                    break;
                case INTERRUPT:
                    need_pooling = true;
                    gpio_int |= gpio;
                    break;
                case NONE:
                    // No configuration needed
                    break;
            }
        }
        libmcp2221::mcp2221_gpioconfset_t gpioConf = libmcp2221::mcp2221_GPIOConfInit();
        int curr_config = 0;
        if (gpio_out) {
            gpioConf.conf[curr_config].gpios = gpio_out;
            gpioConf.conf[curr_config].mode = libmcp2221::MCP2221_GPIO_MODE_GPIO;
            gpioConf.conf[curr_config].direction = libmcp2221::MCP2221_GPIO_DIR_OUTPUT;
            gpioConf.conf[curr_config].value = libmcp2221::MCP2221_GPIO_VALUE_HIGH;
            curr_config ++;
        }
        if (gpio_in) {
            gpioConf.conf[curr_config].gpios = gpio_in;
            gpioConf.conf[curr_config].mode = libmcp2221::MCP2221_GPIO_MODE_GPIO;
            gpioConf.conf[curr_config].direction = libmcp2221::MCP2221_GPIO_DIR_INPUT;
            curr_config ++;
        }
        if (gpio_adc) {
            gpioConf.conf[curr_config].gpios = gpio_adc;
            gpioConf.conf[curr_config].mode  = libmcp2221::MCP2221_GPIO_MODE_ALT1;
            curr_config ++;
        }
        if (gpio_pwm) {
            gpioConf.conf[curr_config].gpios = gpio_pwm;
            gpioConf.conf[curr_config].mode  = libmcp2221::MCP2221_GPIO_MODE_ALT2;
            curr_config ++;
        }
        if (gpio_int) {
            gpioConf.conf[curr_config].gpios = gpio_int;
            gpioConf.conf[curr_config].mode  = libmcp2221::MCP2221_GPIO_MODE_ALT3;
        }
        return gpioConf;
    }

    void Device::set_pin(GPIO pin, bool value) {
        if (pin_mode[pin]!=OUTPUT) {
            cerr << "pin " << pin << " mode is not set to OUTPUT" << endl;
            return;
        }
        pin_values[pin] = value;
        busy.lock();
        auto res = libmcp2221::mcp2221_setGPIO(device, (libmcp2221::mcp2221_gpio_t) (get_gpio(pin)), (libmcp2221::mcp2221_gpio_value_t)value);
        busy.unlock();
        if (res != libmcp2221::MCP2221_SUCCESS){
            cerr << "failed to set pin " << pin << " to " << (value?"HI":"LO") << endl;
        }
    }

    bool Device::get_pin(GPIO pin) {
        get_pins();
        return pin_values[pin];
    }

    void Device::set_interrupt(TRIGGER trigger, void (*p_interrupt)()) {
        libmcp2221::mcp2221_int_trig_t t;
        if (trigger == RISING){
            t = libmcp2221::MCP2221_INT_TRIG_RISING;
        } else {
            t = libmcp2221::MCP2221_INT_TRIG_FALLING;
        }
        busy.lock();
        mcp2221_setInterrupt(device, t, 1);
        busy.unlock();
        interrupt = p_interrupt;
    }

    int Device::get_adc(GPIO pin) {
        if (pin_mode[pin] != ADC){
            cerr << "pin " << pin << " mode is not set to ADC" << endl;
            return 0;
        }
        int adc[MCP2221_ADC_COUNT];
        busy.lock();
        auto res = libmcp2221::mcp2221_readADC(device, adc);
        busy.unlock();
        if(res != libmcp2221::MCP2221_SUCCESS) {
            cerr << "unable to retrieve ADC value from pin " << pin << endl;
            return 0;
        }
        return adc[adc_pins[pin]];
    }

    void Device::set_adc_reference(int) {
        busy.lock();
        mcp2221_setADC(device, libmcp2221::MCP2221_ADC_REF_VDD);
        busy.unlock();
    }

    void Device::get_pins() {
        libmcp2221::mcp2221_gpio_value_t values[MCP2221_GPIO_COUNT];
        busy.lock();
        auto res = libmcp2221::mcp2221_readGPIO(device, values);
        busy.unlock();
        if (res != libmcp2221::MCP2221_SUCCESS){
            cerr << "failed to get pin values" << endl;
        }
        for (size_t i = 0; i < MCP2221_GPIO_COUNT; i++) {
            auto v = values[i] == libmcp2221::MCP2221_GPIO_VALUE_HIGH;
            if (pin_mode[i] == INPUT && pin_values[i] != v) {
                pin_values[i] = v;
            }
        }
    }
}