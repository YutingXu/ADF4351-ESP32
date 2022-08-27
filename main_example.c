#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"

#include "adf4351.h"

static char TAG[] = "main";

extern uint32_t ADF4351_steps[7]; 

void app_main()
{
    //ADF4351 device configuration struct
    static ADF4351_cfg vfo = 
    {
        .pwrlevel = 3,
        .RD2refdouble = 0, ///< ref doubler off
        .RD1Rdiv2 = 0,   ///< ref divider off
        .ClkDiv = 150,
        .BandSelClock = 200,
        .RCounter = 1,  ///< R counter to 1 (no division)
        .ChanStep = 1000000,  ///< set to 1 MHz steps
        .pins = 
        {
            .gpio_ce = 13,
            .gpio_cs = 14, // dummy pin
            .gpio_le = 15, 
            .gpio_sclk = 16,
            .gpio_mosi = 12, 
            .gpio_miso = 11, // dummy pin
            .gpio_ld = 10,
        }
    };

    ESP_LOGI(TAG, "Initialising ADF4351...");
    ADF4351_initialise(&vfo); // initialise the chip
    ESP_LOGI(TAG, "ADF initialisation finished!");

    if(ADF4351_set_ref_freq(&vfo, 25000000) != 0)
        ESP_LOGE(TAG, "Reference frequency input invalid");

    ADF4351_enable(&vfo); // power on the device

    ADF4351_set_freq(&vfo, 440000000); // set output frequency to 440MHz
    ESP_LOGI(TAG, "Frequency set to 440Mhz");

    while(1)
    {
        vTaskDelay(2500 / portTICK_PERIOD_MS);
        ADF4351_disable(&vfo);
        vTaskDelay(2500 / portTICK_PERIOD_MS);
        ADF4351_enable(&vfo);
    }
}
