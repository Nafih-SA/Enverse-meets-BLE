/// Defined in file lcd.h ///////////
#define ROW 448
#define COL 368

/// SPI access code for NRF 52 ////
static int spi_acess(const struct device *spi, struct spi_config *spi_cfg, uint8_t cmd, uint16_t addr, void *data, size_t len)
{
    uint8_t access[2];
    struct spi_buf bufs[] =
        {
            {.buf = data,
             .len = len}};
    struct spi_buf_set tx =
        {
            .buffers = bufs,
            .count = 1};
    return spi_write(spi, spi_cfg, &tx);
}

/// SPI wtite code for NRF52 ////////
int write_bytes(const struct device *spi, struct spi_config *spi_cfg,
                uint16_t addr, uint8_t *data, uint32_t num_bytes)
{
    int err;
    err = spi_acess(spi, spi_cfg, 0, addr, data, num_bytes);
    if (err)
    {
        printk("Error during SPI write\n");
        return -EIO;
    }
    return 0;
}

/// Write command function ///////
void WriteComm(uint8_t data)
{
    const struct device *spi;
    struct spi_config spi_cfg = {0};
    spi = device_get_binding(DT_LABEL(DT_ALIAS(spi_2)));
    if (!spi)
    {
        printk("Could not find SPI driver\n");
        return;
    }
    //spi_cfg.operation = SPI_WORD_SET(8);
    spi_cfg.operation = 256;
    spi_cfg.frequency = spi_freq;
    setOutputcs(0);
    setOutputDcx(0);
    int err;
    err = write_bytes(spi, &spi_cfg, 0x00, &data, 1);
    if (err)
    {
        printk("Error writing to FRAM! errro code (%d)\n", err);
        return;
    }
    setOutputcs(1);
}

/// Write data function /////////
void WriteData(uint8_t data)
{
    const struct device *spi;
    struct spi_config spi_cfg = {0};
    spi = device_get_binding(DT_LABEL(DT_ALIAS(spi_2)));
    if (!spi)
    {
        printk("Could not find SPI driver\n");
        return;
    }
    //spi_cfg.operation = SPI_WORD_SET(8);
    spi_cfg.operation = 256;
    spi_cfg.frequency = spi_freq;
    setOutputcs(0);
    setOutputDcx(1);
    int err;
    err = write_bytes(spi, &spi_cfg, 0x00, &data, 1);
    if (err)
    {
        printk("Error writing to FRAM! errro code (%d)\n", err);
        return;
    }
    setOutputcs(1);
}

// Block write function /////////
void BlockWrite(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend)
{

    // WriteComm(0x2A);
    // WriteData(Xstart >> 8);
    // WriteData(0x10 + Xstart);
    // WriteData(Xend >> 8 + 0x10);
    // WriteData(Xend + 0x10);

    // WriteComm(0x2B);
    // WriteData(Ystart >> 8);
    // WriteData(Ystart);
    // WriteData(Yend >> 8);
    // WriteData(Yend);

    WriteComm(0x2A);
    WriteData(Xstart >> 8);
    WriteData(Xstart);
    WriteData(Xend >> 8);
    WriteData(Xend);

    WriteComm(0x2B);
    WriteData(Ystart >> 8);
    WriteData(Ystart);
    WriteData(Yend >> 8);
    WriteData(Yend);

    WriteComm(0x2c);
}

/// Function to display colour on OLED //////////
void DispColor(unsigned int color)
{
    const struct device *spi;
    struct spi_config spi_cfg = {0};
    spi = device_get_binding(DT_LABEL(DT_ALIAS(spi_2)));
    if (!spi)
    {
        printk("Could not find SPI driver\n");
        return;
    }
    //spi_cfg.operation = SPI_WORD_SET(8);
    spi_cfg.operation = 256;
    spi_cfg.frequency = spi_freq;
    //spi_cfg.frequency = 256000U;
    unsigned int i, j;
    BlockWrite(0, COL - 1, 0, ROW - 1);
    setOutputcs(0);
    setOutputDcx(1);
    for (i = 0; i < ROW; i++)
    {
        for (j = 0; j < COL; j++)
        {
            u8_t data;
            data = color >> 8;
            write_bytes(spi, &spi_cfg, 0x00, &data, 1);
            write_bytes(spi, &spi_cfg, 0x00, &color, 1);
        }
    }
    setOutputcs(1);
}

/// Function to init OLED screen //////
void LCD_Init(void)
{
    init_lcd_output();
    const struct device *spi;
    struct spi_config spi_cfg = {0};
    int err;
    k_msleep(1);
    printk("Initialising DISPLAY  : ");
    setOutputVciEn(1);
    k_msleep(100);
    setOutputcs(1);
    setOutputLcdRst(1);
    k_msleep(80);
    setOutputLcdRst(0);
    k_msleep(80);
    setOutputLcdRst(1);
    k_msleep(480);
    spi = device_get_binding(DT_LABEL(DT_ALIAS(spi_2)));
    if (!spi)
    {
        printk("Could not find SPI driver\n");
        return;
    }
    //spi_cfg.operation = SPI_WORD_SET(8);
    spi_cfg.operation = 256;
    spi_cfg.frequency = spi_freq;
    setDisplay();
    printk("OK\n");
}

void setDisplay()
{
    WriteComm(0xFE);
    WriteData(0x01);
    WriteComm(0x04);
    WriteData(0xA0);
    WriteComm(0x70);
    WriteData(0x55);

    WriteComm(0xFE);
    WriteData(0x0A);
    WriteComm(0x29);
    WriteData(0x10);

    WriteComm(0xFE);
    WriteData(0x05);
    WriteComm(0x05);
    WriteData(0x00);

    WriteComm(0xFE);
    WriteData(0x00);
    WriteComm(0x51);
    WriteData(0xaf);
    WriteComm(0x53);
    WriteData(0x10);

    WriteComm(0x53);
    WriteData(0x20);
    WriteComm(0x35);
    WriteData(0x00);
    WriteComm(0x3A);
    WriteData(0x55); //RGB565

    WriteComm(0x2A);
    WriteData(0x00);
    WriteData(0x10);
    WriteData(0x01);
    WriteData(0x7F);
    WriteComm(0x2B);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x01);
    WriteData(0xBF);

    WriteComm(0x11);
    k_msleep(480);
    WriteComm(0x29);
    k_msleep(80);
}