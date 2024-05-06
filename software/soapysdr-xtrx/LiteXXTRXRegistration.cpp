#include <fcntl.h>
#include <unistd.h>

#include <SoapySDR/Registry.hpp>
#include "LiteXXTRXDevice.hpp"

/***********************************************************************
 * Find available devices
 **********************************************************************/

std::string getXTRXIdentification(int fd) {
    char fpga_identification[256];
    for (int i = 0; i < 256; i ++)
        fpga_identification[i] = litepcie_readl(fd, CSR_IDENTIFIER_MEM_BASE + 4 * i);
    return std::string(&fpga_identification[0]);
}

std::string getXTRXSerial(int fd) {
    char serial[32];
    snprintf(serial, 32, "%x%08x",
                litepcie_readl(fd, CSR_DNA_ID_ADDR + 4 * 0),
                litepcie_readl(fd, CSR_DNA_ID_ADDR + 4 * 1));
    return std::string(&serial[0]);
}

std::vector<SoapySDR::Kwargs> findXTRX(const SoapySDR::Kwargs &args) {
    std::vector<SoapySDR::Kwargs> discovered;
    if (args.count("path") != 0) {
        // respect user choice
        int fd = open(args.at("path").c_str(), O_RDWR);
        if (fd < 0)
            throw std::runtime_error("Invalid device path specified (should be an accessible device node)");

        // gather device info
        SoapySDR::Kwargs dev(args);
        dev["device"] = "LiteXXTRX";
        dev["serial"] = getXTRXSerial(fd);
        dev["identification"] = getXTRXIdentification(fd);
        dev["version"] = "1234";
        size_t ofs = 0;
        while (ofs < sizeof(dev["serial"]) and dev["serial"][ofs] == '0') ofs++;
        char label_str[256];
        sprintf(label_str, "%s %s %s %s", dev["device"].c_str(), args.at("path").c_str(),
                dev["serial"].c_str()+ofs, dev["identification"].c_str());
        dev["label"] = label_str;
        close(fd);

        discovered.push_back(dev);
    } else {
        // find all LitePCIe devices
        for (int i = 0; i < 10; i++) {
            std::string path = "/dev/litepcie" + std::to_string(i);
            int fd = open(path.c_str(), O_RDWR);
            if (fd < 0)
                continue;

            // check the FPGA identification to see if this is an XTRX
            std::string fpga_identification = getXTRXIdentification(fd);
            if (strstr(fpga_identification.c_str(), "LiteX SoC on Fairwaves XTRX") != NULL) {
                // gather device info
                SoapySDR::Kwargs dev(args);
                dev["device"] = "LiteXXTRX";
                dev["path"] = path;
                dev["serial"] = getXTRXSerial(fd);
                dev["identification"] = &fpga_identification[0];
                dev["version"] = "1234";
                size_t ofs = 0;
                while (ofs < sizeof(dev["serial"]) and dev["serial"][ofs] == '0') ofs++;
                char label_str[256];
                sprintf(label_str, "%s %s %s %s", dev["device"].c_str(), path.c_str(),
                    dev["serial"].c_str()+ofs, dev["identification"].c_str());
                dev["label"] = label_str;
                close(fd);

                // filter by serial if specified
                if (args.count("serial") != 0) {
                    // filter on serial number
                    if (args.at("serial") != dev["serial"])
                        continue;
                }

                discovered.push_back(dev);
            }
        }
    }

    return discovered;
}


/***********************************************************************
 * Make device instance
 **********************************************************************/

SoapySDR::Device *makeLiteXXTRX(const SoapySDR::Kwargs &args) {
    return new SoapyLiteXXTRX(args);
}


/***********************************************************************
 * Registration
 **********************************************************************/

static SoapySDR::Registry registerLiteXXTRX("LiteXXTRX", &findXTRX, &makeLiteXXTRX,
                                       SOAPY_SDR_ABI_VERSION);
