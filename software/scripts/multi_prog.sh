#! /usr/bin/env bash

seq 0 8 | parallel ../litepcie/user/litepcie_util -c {} flash_write ../../build/fairwaves_xtrx_platform/gateware/fairwaves_xtrx_platform.bin