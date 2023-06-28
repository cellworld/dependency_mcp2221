#include<thread>
#include<catch.h>
#include<iostream>
#include<mcp2221.h>
#include<libmcp2221.h>
#include<hidapi.h>

using namespace std;
using namespace libmcp2221;

int int_count = 0;
int change_count = 0;

void int_code(){
    int_count++;
    cout << "PIN1 INTERRUPT COUNT: " << int_count << endl;
}

TEST_CASE("mcp2221") {
    auto device_info = mcp2221::Device::get_devices();
    if (device_info.empty()){
        cout << "device not found" << endl;
    } else {
        auto d = device_info[0];
        cout << "device foun at " << d.path << endl;
        cout << "opening..." << endl;
        mcp2221::Device device  (d.path);
        device.set_direction(mcp2221::pin0, mcp2221::OUTPUT);
        device.set_direction(mcp2221::pin1, mcp2221::INTERRUPT);
        device.open();
        device.set_interrupt(mcp2221::RISING, int_code);
        bool p = false;
        while (true){
            this_thread::sleep_for(250ms);
            device.set_pin(mcp2221::pin0, p);
            this_thread::sleep_for(250ms);
            p = !p;
        }
    }
}
