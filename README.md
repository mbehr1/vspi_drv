# vspi_drv
Virtual SPI linux kernel driver to be able to simulate failures / unreliability features.

(c) Matthias Behr, 2011 - 2020

## Goal / motivation

This was developed to test e.g. two SW modules communicating via SPI typically from one ECU/SOC to another one.

This module should help to inject failures that are otherwise only really sporadic and difficult to evaluate/test.

