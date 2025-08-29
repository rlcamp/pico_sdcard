int spi_sd_read_blocks(void * buf, unsigned long blocks, unsigned long long block_address);
int spi_sd_write_blocks(const void * buf, const unsigned long blocks, const unsigned long long block_address);

int spi_sd_write_pre_erase(unsigned long blocks);
int spi_sd_init(void);

int spi_sd_write_pre_erase(unsigned long blocks);
int spi_sd_write_blocks_start(unsigned long long block_address);
int spi_sd_write_some_blocks(const void * buf, const unsigned long blocks);
void spi_sd_write_blocks_end(void);
