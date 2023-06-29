#include <libmcp2221.h>
#include <hidapi.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <functional>
#include <iostream>
#include <chrono>

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
        FALLING,
        RISING_AND_FALLING
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
        void set_pin(GPIO, bool);
        void get_pins();
        bool get_pin(GPIO);
        int get_adc(GPIO);
        void set_adc_reference(int);
        std::vector<std::atomic<bool>> pin_values;

        template<int N>
        struct my_placeholder {
            static my_placeholder ph;
        };

        template<int... indices>
        static auto bind_all(auto f, auto val, std::integer_sequence<int, indices...>) {
            return std::bind(f, val, my_placeholder<indices+1>::ph...);
        }

        template<typename Callable, typename... Args,
                typename = std::_Require< std::__not_<std::is_same<std::__remove_cvref_t<Callable>, std::thread>>>>
        void set_interrupt(TRIGGER trigger, Callable f, Args... args) {

            libmcp2221::mcp2221_int_trig_t t;
            switch (trigger) {
                case(RISING):
                    t = libmcp2221::MCP2221_INT_TRIG_RISING;
                    break;
                case(FALLING):
                    t = libmcp2221::MCP2221_INT_TRIG_FALLING;
                    break;
                case (RISING_AND_FALLING):
                    t = libmcp2221::MCP2221_INT_TRIG_BOTH;
                    break;
            }
            busy.lock();
            mcp2221_setInterrupt(device, t, 1);
            busy.unlock();

            interrupt_thread = std::thread([this, f](auto o, auto ...args) {
                constexpr std::size_t n = sizeof...(args);
                auto bf = Device::bind_all(f, o,  std::make_integer_sequence<int, sizeof...(args)>());

                while (manage_interrupt) {
                    int interrupt;
                    busy.lock();
                    auto res = mcp2221_readInterrupt(device, &interrupt);
                    busy.unlock();
                    if(res ==libmcp2221::MCP2221_SUCCESS) {
                        if (interrupt) {
                            busy.lock();
                            res = mcp2221_clearInterrupt(device);
                            busy.unlock();
                            if (interrupt) {
                                bf(args...);
                            }
                            if (res != libmcp2221::MCP2221_SUCCESS) {
                                std::cerr << "failed to clear interrupt flag" << std::endl;
                            }
                        }
                    } else {
                        std::cerr << "failed to retrieve interrupt state" << std::endl;
                    }
                    std::this_thread::sleep_for(std::chrono::microseconds(500));
                }
            }, args...);
        }
    private:
        libmcp2221::mcp2221_gpioconfset_t _config();
        std::string path;
        libmcp2221::mcp2221_t *device;
        std::vector<MODE> pin_mode;
        std::vector<size_t> adc_pins;
        std::thread pooling_thread;
        std::thread interrupt_thread;
        std::atomic<bool> manage_interrupt;
        std::mutex busy;
    };
}
