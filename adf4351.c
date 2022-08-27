/*!
   @file adf4351.c
   @mainpage ADF4351 ESP32 library driver for Wideband Frequency Synthesizer
   @section intro_sec Introduction
   This is a C port of the original ADF4351 library made by David Fannin, KK6DF, et al.
   The driver has been tested on a ESP32-devkitc-v4 module connected to a Analog Devices ADF4351 eval board.
   License goes to the authors of the original library.

   The ADF4351 chip is a wideband freqency synthesizer integrated circuit that can generate frequencies
   from 35 MHz to 4.4 GHz. It incorporates a PLL (Fraction-N and Integer-N modes) and VCO, along with
   prescalers, dividers and multipiers. The users add a PLL loop filter and reference frequency to
   create a frequency generator with a very wide range, that is tuneable in settable frequency steps.
   The ADF4351 chip provides an I2C interface for setting the device registers that control the
   frequency and output levels, along with several IO pins for gathering chip status and
   enabling/disabling output and power modes.
   The ADF4351 library provides an Arduino API for accessing the features of the ADF chip.
   The basic PLL equations for the ADF4351 are:
   \f$ RF_{out} = f_{PFD} \times (INT +(\frac{FRAC}{MOD})) \f$
   where:
   \f$ f_{PFD} = REF_{IN} \times \left[ \frac{(1 + D)}{( R \times (1 + T))} \right]  \f$
   \f$ D = \textrm{RD2refdouble, ref doubler flag}\f$
   \f$ R = \textrm{RCounter, ref divider}\f$
   \f$ T = \textrm{RD1Rdiv2, ref divide by 2 flag}\f$
   @section author Author
   Yuting Xu
   @section license License
   MIT License
*/


#include "adf4351.h"

static const char *TAG = "PLL";

uint32_t ADF4351_steps[] = {100, 500, 10000, 50000, 100000, 500000, 1000000};

int ADF4351_set_freq(ADF4351_cfg *pcfg, uint32_t freq)
{
    // input range checks
    // this will never trigger
    // if(freq > ADF_FREQ_MAX) 
    //     return -1;
    if(freq < ADF_FREQ_MIN)
        return -1;

    // equation for RF_OUT
    // RF_OUT = [INT + (FRAC/MOD)] * (f_PFD/RF_DIVIDER)
    // RF_OUT is the RF frequency output
    // INT is the integer division facgtor
    // FRAC is the numerator of the fractional division (0 to MOD - 1)
    // MOD is the preset fractional modulus (2 to 4095)
    // RF_DIVIDER is the output dividers that divides down the VCO frequency
    // f_PFD = REF_IN * (1 + D)/(R * (1+T))

    int localosc_ratio = 2200000000 / freq;
    pcfg->_outdiv = 1;
    int RfDivSel = 0;

    // select the output divider
    while(pcfg->_outdiv <= localosc_ratio && pcfg->_outdiv <= 64)
    {
        pcfg->_outdiv *= 2;
        RfDivSel++;
    }

    if (freq > 3600000000/pcfg->_outdiv)
    {
        pcfg->Prescaler = 1;
    }
    else 
    {
        pcfg->Prescaler = 0;
    }

    double PFDFreq = pcfg->_reffreq * (1.0 + pcfg->RD2refdouble) / (pcfg->RCounter * (1.0 + pcfg->RD1Rdiv2)); // find the loop frequency

    double N = freq * pcfg->_outdiv / PFDFreq;
    pcfg->_N_Int = (uint16_t) N;
    double Mod = PFDFreq / pcfg->ChanStep;
    Mod = (double) ((uint32_t) Mod);
    double Frac = (N - pcfg->_N_Int) * Mod + 0.5;
    pcfg->_Frac = (uint32_t) Frac;
    pcfg->_Mod = (uint32_t) Mod;

    if(pcfg->_Frac != 0) 
    {
        uint32_t gcd = ADF4351_gcd_iter(pcfg->_Frac, pcfg->_Mod) ;

        if(gcd > 1) 
        {
            pcfg->_Frac /= gcd;
            Frac = (double) pcfg->_Frac;
            pcfg->_Mod /= gcd;
            Mod = (double) Mod;
        }
    }

    double cfreq;

    if(pcfg->_Frac == 0) 
    {
        cfreq = (PFDFreq * N) / pcfg->_outdiv;
    } 
    else 
    {
        cfreq = (PFDFreq * (N + (Frac/Mod))) / pcfg->_outdiv;
    }

    pcfg->_cfreq = cfreq;

    if(freq != (uint32_t) cfreq)
    {
        ESP_LOGI(TAG, "Calculated frequency different from input frequency");
    }
    
    // error checks
    if(pcfg->_Mod < 2 || pcfg->_Mod > 4095)
    {
        ESP_LOGE(TAG, "Mod out of range");
        return -1;
    }

    if(pcfg->_Frac > pcfg->_Mod - 1)
    {
        ESP_LOGE(TAG, "Frac out of range");
        return -1;
    }

    if((pcfg->Prescaler == 0) && (pcfg->_N_Int < 23))
    {
        ESP_LOGE(TAG, "N_int out of range");
        return -1;
    }

    else if((pcfg->Prescaler == 1) && (pcfg->_N_Int < 75))
    {
        ESP_LOGE(TAG, "N_int out of range");
        return -1;
    }

    // set the registers to 0 first
    memset(&pcfg->_registers, 0, sizeof(pcfg->_registers));

    // configure register 0
    // (0,3,0) control bits
    ADF4351_set_register_bf(&pcfg->_registers[0], 3, 12, pcfg->_Frac);
    ADF4351_set_register_bf(&pcfg->_registers[0], 15, 16, pcfg->_N_Int);
    ESP_LOGI(TAG, "N_Int calculated to be %d", pcfg->_N_Int);

    // configure register 1
    ADF4351_set_register_bf(&pcfg->_registers[1], 0, 3, 1);
    ADF4351_set_register_bf(&pcfg->_registers[1], 3, 12, pcfg->_Mod);
    ADF4351_set_register_bf(&pcfg->_registers[1], 15, 12, 1);
    ADF4351_set_register_bf(&pcfg->_registers[1], 27, 1, pcfg->Prescaler);
    // (28,1,0) phase adjust

    // configure register 2
    ADF4351_set_register_bf(&pcfg->_registers[2], 0, 3, 2);
    // (3,1,0) counter reset
    // (4,1,0) cp3 state
    // (5,1,0) power down
    ADF4351_set_register_bf(&pcfg->_registers[2], 6, 1, 1); // pd polarity
    if(pcfg->_Frac == 0)  
    {
        ADF4351_set_register_bf(&pcfg->_registers[2], 7, 1, 1); // LDP, int-n mode
        ADF4351_set_register_bf(&pcfg->_registers[2], 8, 1, 1); // ldf, int-n mode
    } 
    else 
    {
        ADF4351_set_register_bf(&pcfg->_registers[2], 7, 1, 0); // LDP, frac-n mode
        ADF4351_set_register_bf(&pcfg->_registers[2], 8, 1, 0); // ldf, frac-n mode
    }
    ADF4351_set_register_bf(&pcfg->_registers[2], 9, 4, 7); // charge pump
    // (13,1,0) dbl buf
    ADF4351_set_register_bf(&pcfg->_registers[2], 14, 10, pcfg->RCounter); // R counter
    ADF4351_set_register_bf(&pcfg->_registers[2], 24, 1, pcfg->RD1Rdiv2)  ; // RD1_RDiv2
    ADF4351_set_register_bf(&pcfg->_registers[2], 25, 1, pcfg->RD2refdouble)  ; // RD2refdouble
    // R[2].setbf(26,3,0) ; //  muxout, not used
    // (29,2,0) low noise and spurs mode

    // configure register 3
    ADF4351_set_register_bf(&pcfg->_registers[3], 0, 3, 3) ; // control bits
    ADF4351_set_register_bf(&pcfg->_registers[3], 3, 12, pcfg->ClkDiv) ; // clock divider
    // (15,2,0) clk div mode
    // (17,1,0) reserved
    // (18,1,0) CSR
    // (19,2,0) reserved
    if(pcfg->_Frac == 0 ) 
    {
        ADF4351_set_register_bf(&pcfg->_registers[3], 21, 1, 1); //  charge cancel, reduces pfd spurs
        ADF4351_set_register_bf(&pcfg->_registers[3], 22, 1, 1); //  ABP, int-n
    } 
    else  
    {
        ADF4351_set_register_bf(&pcfg->_registers[3], 21, 1, 0) ; //  charge cancel
        ADF4351_set_register_bf(&pcfg->_registers[3], 22, 1, 0); //  ABP, frac-n
    }
    ADF4351_set_register_bf(&pcfg->_registers[3], 23, 1, 1) ; // Band Select Clock Mode
    // (24,8,0) reserved

    // configure register 4
    ADF4351_set_register_bf(&pcfg->_registers[4], 0, 3, 4) ; // control bits
    ADF4351_set_register_bf(&pcfg->_registers[4], 3, 2, pcfg->pwrlevel) ; // output power 0-3 (-4dbM to 5dbM, 3db steps)
    ADF4351_set_register_bf(&pcfg->_registers[4], 5, 1, 1) ; // rf output enable
    // (6,2,0) aux output power
    // (8,1,0) aux output enable
    // (9,1,0) aux output select
    // (10,1,0) mtld
    // (11,1,0) vco power down
    ADF4351_set_register_bf(&pcfg->_registers[4], 12, 8, pcfg->BandSelClock) ; // band select clock divider
    ADF4351_set_register_bf(&pcfg->_registers[4], 20, 3, RfDivSel) ; // rf divider select
    ADF4351_set_register_bf(&pcfg->_registers[4], 23, 1, 1) ; // feedback select
    // (24,8,0) reserved

    // configure register 5
    ADF4351_set_register_bf(&pcfg->_registers[5], 0, 3, 5) ; // control bits
    // (3,16,0) reserved
    ADF4351_set_register_bf(&pcfg->_registers[5], 19, 2, 3) ; // Reserved field,set to 11
    // (21,1,0) reserved
    ADF4351_set_register_bf(&pcfg->_registers[5], 22, 2, 1) ; // LD Pin Mode
    // (24,8,0) reserved

    int i;
    for(i = 5; i > -1; i--)
    {
        ESP_LOGI(TAG, "Writing register with content 0x%08x", pcfg->_registers[i]);
        ADF4351_write_register(pcfg, pcfg->_registers[i]);
        ets_delay_us(1500); // why wait so long?
    }

    return 0;
}

int ADF4351_set_ref_freq(ADF4351_cfg *pcfg, uint32_t ref_freq)
{
    // check if input is valid
    if(ref_freq > ADF_REFIN_MAX)
        return -1; // change return from 1 to -1 for clearer sign of error
    else if(ref_freq < 100000)
        return -1;

    float new_ref_freq = ref_freq * (1.0 + pcfg->RD2refdouble) / (pcfg->RCounter * (1.0 + pcfg->RD1Rdiv2));

    if(new_ref_freq > ADF_PFD_MAX)
        return -1;
    else if(ref_freq < ADF_PFD_MIN)
        return -1;

    pcfg->_reffreq = ref_freq;
    
    return 0;
}

void ADF4351_write_register(ADF4351_cfg *pcfg, uint32_t reg)
{
    // ensure LE pin is low
    gpio_set_level(pcfg->pins.gpio_le, 0);
    ets_delay_us(5);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.tx_data[0] = (reg>>24)&0xff;
    t.tx_data[1] = (reg>>16)&0xff;
    t.tx_data[2] = (reg>> 8)&0xff;
    t.tx_data[3] = (reg    )&0xff;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 32;
    t.rx_buffer = NULL;

    spi_device_transmit(spi_handle, &t);


    gpio_set_level(pcfg->pins.gpio_le, 1);    // pull LE pin high to latch incoming word into register
    ets_delay_us(5);
    gpio_set_level(pcfg->pins.gpio_le, 0);    // pull LE pin to its default low
}

uint32_t ADF4351_gcd_iter(uint32_t u, uint32_t v)
{
    uint32_t t;

    while(v) 
    {
        t = u;
        u = v;
        v = t % v;
    }

    return u;
}

void ADF4351_enable(ADF4351_cfg *pcfg)
{
    // check if CE pin was initialised
    if(pcfg->_ce_initialised)
    {
        gpio_set_level(pcfg->pins.gpio_ce, 1);
        pcfg->_enabled = true;
    }
    else
        ESP_LOGE(TAG, "Attempting to toggle CE pin without initialisation");
}

void ADF4351_disable(ADF4351_cfg *pcfg)
{
    // check if CE pin was initialised
    if(pcfg->_ce_initialised)
    {
        gpio_set_level(pcfg->pins.gpio_ce, 0);
        pcfg->_enabled = false;
    }
    else
        ESP_LOGE(TAG, "Attempting to toggle CE pin without initialisation");
}

void ADF4351_set_register_bf(uint32_t *preg, uint8_t start, uint8_t len, uint32_t value)
{
    uint32_t bitmask = (1 << len) - 1;
    value &= bitmask;
    bitmask <<= start;
    *preg = (*preg & (~bitmask)) | (value << start);
}

uint32_t ADF4351_get_register_bf(uint32_t *preg, uint8_t start, uint8_t len)
{
    uint32_t bitmask = ((1 << len) - 1) << start;
    uint32_t result = (*preg & bitmask) >> start;
    return result;
}

void ADF4351_initialise(ADF4351_cfg *pcfg)
{
    // initialise internal fields
    pcfg->_enabled = false;
    pcfg->_reffreq = REF_FREQ_DEFAULT;
    pcfg->_cfreq = 0.0;

	// Step1, initalise SPI perpheral and GPIO
	// Configuration for the SPI bus
	spi_bus_config_t buscfg = 
    {
		.mosi_io_num = pcfg->pins.gpio_mosi,
		.miso_io_num = pcfg->pins.gpio_miso,
		.sclk_io_num = pcfg->pins.gpio_sclk,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	// Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg = 
    {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = APB_CLK_FREQ/80, // run at 1MHz
        .duty_cycle_pos = 128,        // 50% duty cycle
        .mode = 0,
        .spics_io_num = pcfg->pins.gpio_cs, // does not matter, not using the SPI CS pin anyways
        .cs_ena_posttrans = 0,        
        .queue_size = 1
    };

    // GPIO settings for load enable pin (active high)
    gpio_config_t ADF4351_le_cfg = 
    {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 1,
        .pin_bit_mask = (1 << pcfg->pins.gpio_le)
    };

    // GPIO settings for chip enable pin (active high)
    gpio_config_t ADF4351_ce_cfg = 
    {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 1,
        .pin_bit_mask = (1 << pcfg->pins.gpio_ce)
    };

    // GPIO settings for lock detect pin (high - PLL lock, low - loss of PLL lock)
    gpio_config_t ADF4351_ld_cfg = 
    {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1 << pcfg->pins.gpio_ld)
    };

	esp_err_t ret;

    ret = gpio_config(&ADF4351_le_cfg);
    assert(ret == ESP_OK);
    pcfg->_le_initialised = true;

    ret = gpio_config(&ADF4351_ce_cfg);
    assert(ret == ESP_OK);
    pcfg->_ce_initialised = true;

    ret = gpio_config(&ADF4351_ld_cfg);
    assert(ret == ESP_OK);
    pcfg->_ld_initialised = true;

    // drive LE and CE low just in case
    gpio_set_level(pcfg->pins.gpio_le, 0);
    gpio_set_level(pcfg->pins.gpio_ce, 0);

    ESP_LOGI(TAG, "GPIO successfully initialised");

	ret = spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    ESP_LOGI(TAG, "SPI bus successfully initialised");

	ret = spi_bus_add_device(SENDER_HOST, &devcfg, &spi_handle);
	assert(ret == ESP_OK);

    ESP_LOGI(TAG, "SPI device successfully attached");
    pcfg->_spi_initialised = true;
}

