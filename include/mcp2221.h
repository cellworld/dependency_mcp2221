#include "libmcp2221.h"
#include "hidapi.h"
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#define  get_gpio(pin) (1 << (int)pin)

namespace mcp2221 {

    enum GPIO {
        pin0,
        pin1,
        pin2,
        pin3
    };
    enum MODE {
        NONE,
        INPUT,
        OUTPUT,
        ADC,
        PWM,
        INTERRUPT
    };

    enum TRIGGER {
        RISING,
        FALLING
    };

    struct Device_info {
        std::string path;
        unsigned short vendor_id;
        unsigned short product_id;
        std::string serial_number;
        unsigned short release_number;
        std::string manufacturer_string;
        std::string product_string;
        unsigned short usage_page;
        unsigned short usage;
        int interface_number;
    };

    struct Device {
        Device(std::string &path);
        ~Device();
        void open();
        static std::vector<Device_info> get_devices();
        void set_direction(GPIO, MODE);
        void set_interrupt(TRIGGER, void (*)());
        void set_pin(GPIO, bool);
        void get_pins();
        bool get_pin(GPIO);
        int get_adc(GPIO);
        void set_adc_reference(int);
        std::vector<std::atomic<bool>> pin_values;

    private:
        static void _pooling_process(Device *device);
        libmcp2221::mcp2221_gpioconfset_t _config();
        std::string path;
        libmcp2221::mcp2221_t *device;
        std::vector<MODE> pin_mode;
        std::vector<size_t> adc_pins;
        std::thread pooling_thread;
        bool need_pooling;
        void (*interrupt)();
        std::mutex busy;
    };
}
