# vspi_drv
Virtual SPI Linux kernel driver to be able to simulate failures / unreliability features.

(c) Matthias Behr, 2011 - 2020

## Goal / motivation

This was developed to test e.g. two SW modules communicating via SPI typically from one ECU/SOC to another one.

This module should help to inject failures that are otherwise only really sporadic and difficult to evaluate/test.

## Details

After loading the module you have two character devices that are virtually connected to each other.

Parameters like **"simulated BER" (bit error rate)** can be set at module load time or changed at runtime via sysfs. (/sys/modules/vspi_drv/parameters/).

## How to build
`make`

## How to test

Load the driver:

`sudo ./vspi_load`

Check whether the driver is loaded properly:

`dmesg`
 
You should see a message like `vspi_drv_init (HZ=xxx)...`.

You can now test with the spidev_test as well:

compile it:

`cc spidev_test.c && ./a.out -D /dev/vspi_drv0`
or for client side:

`./a.out -D /dev/vspi_drv1`

Unload the driver:

`sudo ./vspi_unload`
