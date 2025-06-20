# SPI Communication Between Two AVR Boards

## Overview

This project demonstrates full-duplex SPI (Serial Peripheral Interface) communication between two AVR microcontrollers. The system is split into two modules:

- **SPI Master** — Initiates data transfers and controls communication.
- **SPI Slave** — Responds to master's requests and transmits data accordingly.

Both modules are implemented using low-level AVR C, directly accessing registers without Arduino libraries. The project is fully interrupt-driven to ensure efficient and accurate SPI communication.

---

## Project Structure

spi-dual-board/
├── master/ --> SPI Master source code
├── slave/ --> SPI Slave source code
└── README.md --> Project overview


Each directory contains all necessary source files, headers, and configuration files for independent compilation and flashing.

---

## System Description

### Hardware

- 2x ATmega328P Microcontrollers
- SPI Connections:
  - **MOSI**: Master Out Slave In
  - **MISO**: Master In Slave Out
  - **SCK**: Serial Clock (generated by Master)
  - **SS**: Slave Select
- Shared GND between both boards
- Optional USART connection for serial debugging

### Software Features

- Direct register manipulation using AVR C.
- SPI configured in:
  - Master: Mode 0, prescaler configured.
  - Slave: Fully interrupt-driven SPI reception.
- Debug output using USART.
- Flexible message handling with simple data frames for demonstration purposes.
