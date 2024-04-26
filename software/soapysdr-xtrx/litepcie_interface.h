#include "liblitepcie.h"

#include <fcntl.h>
#include <unistd.h>

#define LITEPCIE_SPI_CS_HIGH (0 << 0)
#define LITEPCIE_SPI_CS_LOW  (1 << 0)
#define LITEPCIE_SPI_START   (1 << 0)
#define LITEPCIE_SPI_DONE    (1 << 0)
#define LITEPCIE_SPI_LENGTH  (1 << 8)

//#define DBG_TRANSACTION

static inline uint32_t litepcie_interface_transact(void *handle, const uint32_t data_in, const bool readback)
{
    int *fd = (int *)handle;

    //load tx data
    litepcie_writel(*fd, CSR_LMS7002M_SPI_MOSI_ADDR, data_in);

    //start transaction
    litepcie_writel(*fd, CSR_LMS7002M_SPI_CONTROL_ADDR, 32*LITEPCIE_SPI_LENGTH | LITEPCIE_SPI_START);

    //wait for completion
    while ((litepcie_readl(*fd, CSR_LMS7002M_SPI_STATUS_ADDR) & LITEPCIE_SPI_DONE) == 0);

    //load rx data
    if (readback) {
        uint32_t ret = litepcie_readl(*fd, CSR_LMS7002M_SPI_MISO_ADDR) & 0xffff;
#ifdef DBG_TRANSACTION
        printf("%s read  addr: 0x%04x -> %08x\n", __func__, 0x7fff & (data_in >> 16), ret);
#endif
        return ret;
    } else {
#ifdef DBG_TRANSACTION
        printf("%s write addr: 0x%04x value: 0x%04x\n", __func__,
            0x7fff & (data_in >> 16), data_in & 0xffff);
#endif
        return 0;
    }
}
