#include<thread>
#include<catch.h>
#include<iostream>
#include<libmcp2221.h>
#include<hidapi.h>

using namespace std;
using namespace libmcp2221;

static mcp2221_error readGPIOs(mcp2221_t* myDev)
{
puts("~~~~~~~~~~~~");

mcp2221_gpio_value_t values[MCP2221_GPIO_COUNT];
mcp2221_error res = mcp2221_readGPIO(myDev, values);
if(res != MCP2221_SUCCESS)
return res;

cout << "GPIO 1:" << values[1] << endl;

return res;
}

TEST_CASE("server") {

    puts("Starting!");

    mcp2221_init();

    // Get list of MCP2221s
    printf("Looking for devices... ");
    int count = mcp2221_find(MCP2221_DEFAULT_VID, MCP2221_DEFAULT_PID, NULL, NULL, NULL);
    printf("found %d devices\n", count);

    // Open whatever device was found first
    printf("Opening device... ");
    mcp2221_t* myDev = mcp2221_open();

    if(!myDev)
    {
        mcp2221_exit();
        puts("No MCP2221s found");
        getchar();
        FAIL();
        exit;
    }
    puts("done");

    // Configure GPIOs
    printf("Configuring GPIOs... ");
    mcp2221_gpioconfset_t gpioConf = mcp2221_GPIOConfInit();

    // Check GP DESIGNATION TABLE in the datasheet for what the dedicated and alternate functions are for each GPIO pin

    // Configure GPIO 0 and 2 as OUTPUT HIGH
    gpioConf.conf[0].gpios		= MCP2221_GPIO0 ;
    gpioConf.conf[0].mode		= MCP2221_GPIO_MODE_GPIO;
    gpioConf.conf[0].direction	= MCP2221_GPIO_DIR_OUTPUT;
    gpioConf.conf[0].value		= MCP2221_GPIO_VALUE_HIGH;

    // Configure GPIO 1 and GPIO 3 as INPUT
    gpioConf.conf[1].gpios		= MCP2221_GPIO1 ;
    gpioConf.conf[1].mode		= MCP2221_GPIO_MODE_GPIO;
    gpioConf.conf[1].direction	= MCP2221_GPIO_DIR_INPUT;

    // Apply config
    mcp2221_setGPIOConf(myDev, &gpioConf);

    // Also save config to flash
    mcp2221_saveGPIOConf(myDev, &gpioConf);

    puts("done");

    mcp2221_error res;
    bool v = false;
    while(1)
    {
        // Get GPIO 0 and 2 to LOW
        res = mcp2221_setGPIO(myDev, static_cast<mcp2221_gpio_t>(MCP2221_GPIO0), v?MCP2221_GPIO_VALUE_HIGH:MCP2221_GPIO_VALUE_LOW);
        if(res != MCP2221_SUCCESS)
            break;

        this_thread::sleep_for(250ms);

        res = readGPIOs(myDev);
        if(res != MCP2221_SUCCESS)
            break;


        this_thread::sleep_for(250ms);
        v = !v;

    }
    mcp2221_exit();
}

