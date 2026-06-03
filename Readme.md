# ESP32-P4 USB RNDIS Host Internet Connectivity

## Overview

This project demonstrates how an **ESP32-P4** operates as a **USB Host** and obtains Internet connectivity from a **Custom Cellular Controller** through **USB Tethering (RNDIS)**.

The ESP32-P4 performs:

- USB device enumeration
- CDC-ACM communication for modem control
- Network session activation using AT commands
- RNDIS Host initialization
- IPv4 and IPv6 network configuration
- Internet connectivity verification using Ping, TCP, and HTTP requests

The solution is implemented using **ESP-IDF** and **ESP-IoT-Solution USB Host components**.

---

# Architecture

```text
+------------------------------------+
|      Custom Cellular Controller    |
|                                    |
|  Mobile Network (4G/5G)            |
|           |                        |
|       NAT / RNDIS                  |
+-----------+------------------------+
            |
            | USB
            |
+-----------v------------------------+
|            ESP32-P4                |
|                                    |
|  USB Host Stack                    |
|  CDC Host Driver                   |
|  RNDIS Host Driver                 |
|  ESP-NETIF                         |
|  LWIP TCP/IP Stack                 |
|                                    |
|  Ping / TCP / HTTP Validation      |
+------------------------------------+
```

---

# Features

## USB Host Functionality

- USB Host stack initialization
- USB device enumeration
- Composite USB device parsing
- Interface discovery
- USB hot-plug support
- Enumeration filtering

## CDC-ACM Control Channel

- Opens CDC interface for AT communication
- Sends modem initialization commands
- Reads and validates responses
- Handles command timeouts and retries

## RNDIS Networking

- USB RNDIS Host Driver
- Ethernet-over-USB interface
- Dynamic network interface creation
- Link status monitoring
- Event-driven connectivity handling

## IPv4 Support

- DHCP support
- Static IP fallback support
- Gateway configuration
- Network interface management

## IPv6 Support

- SLAAC address acquisition
- Global IPv6 address detection
- IPv6 routing validation
- Modem IPv6 comparison and verification

## Connectivity Validation

- ICMP Ping
- TCP Socket Connection Test
- DNS Reachability Check
- HTTP GET Request Validation

---

# ESP-IDF Components Used

## Public Components

```cmake
REQUIRES
    nvs_flash
    esp_event
    esp_netif
    esp_http_client
    mbedtls
    driver
    usb
    lwip
```

## Private Components

```cmake
PRIV_REQUIRES
    iot_usbh_cdc
    iot_usbh_rndis
    iot_eth
```

---

# Component Manager Dependencies

```yaml
dependencies:
  espressif/iot_usbh_cdc: "*"
  espressif/iot_usbh_rndis: "*"
  espressif/iot_eth: "*"
```

---

# Required ESP-IDF Configuration

Enable the following options:

```config
CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK=y
CONFIG_USB_HOST_HUBS_SUPPORTED=y
CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=512
```

---

# Network Initialization Flow

```text
USB Device Connected
          |
          v
Parse USB Descriptors
          |
          v
Open CDC Interface
          |
          v
Send AT Commands
          |
          v
Activate Mobile Data Session
          |
          v
Start RNDIS Host Driver
          |
          v
Create Ethernet Netif
          |
          v
Acquire IPv4/IPv6
          |
          v
Connectivity Validation
```

---

# AT Command Sequence

The ESP32-P4 sends a sequence of AT commands to configure and activate the cellular connection.

Example sequence:

```text
AT
AT+CGDCONT=1,"IPV4V6","<APN>"
AT$QCNETCFG="nat",1,"192.168.10.2"
AT$QCPCFG="usbCtrl",0
AT$QCPCFG="usbNet",0
AT+CGACT=1,1
AT$QCNETDEVCTL=3,1,1
```

These commands:

- Configure PDP context
- Enable USB networking
- Configure NAT mode
- Activate packet data connection
- Bind network stack to USB RNDIS interface

---

# Event Handling

The application monitors:

## USB Events

- Device Connected
- Device Disconnected

## Ethernet Events

- Link Up
- Link Down

## IP Events

- DHCP Address Acquired
- IPv6 Address Acquired

---

# Connectivity Verification

## IPv6 Ping Test

Target:

```text
2001:4860:4860::8888
```

Verifies:

- Internet reachability
- ICMP connectivity

---

## TCP Connectivity Test

Target:

```text
2606:4700:4700::1111:53
```

Verifies:

- TCP stack functionality
- Network routing

---

## HTTP Connectivity Test

Target:

```text
http://httpbin.org/get
```

Verifies:

- Application-layer Internet access
- DNS resolution
- End-to-end connectivity

---

# FreeRTOS Tasks

## USB Host Task

Responsibilities:

- USB Host event processing
- Enumeration handling

---

## AT Setup Task

Responsibilities:

- CDC communication
- Modem initialization
- AT command execution

---

## Network Task

Responsibilities:

- Link monitoring
- Connectivity testing
- IPv4/IPv6 validation

---

# Build Instructions

## Install Dependencies

```bash
idf.py add-dependency "espressif/iot_usbh_cdc"
idf.py add-dependency "espressif/iot_usbh_rndis"
idf.py add-dependency "espressif/iot_eth"
```

## Configure Project

```bash
idf.py menuconfig
```

Enable USB Host related options.

## Build

```bash
idf.py build
```

## Flash

```bash
idf.py -p <PORT> flash monitor
```

---

# Expected Runtime Flow

```text
USB Device Connected
CDC Interface Opened
AT Script Executed Successfully
RNDIS Driver Started
Link Up
IP Address Acquired
IPv6 Address Acquired
Ping Successful
TCP Test Successful
HTTP GET Successful
Internet Reachable
```

---

# Learning Outcomes

This project demonstrates:

- USB Host Programming on ESP32-P4
- CDC-ACM Communication
- USB RNDIS Networking
- Cellular Internet Integration
- ESP-NETIF Usage
- LWIP Networking
- IPv4/IPv6 Configuration
- Embedded TCP/IP Validation
- FreeRTOS Event-Driven Architecture

---

# Author

Embedded Systems and IoT Networking Research

Focus Areas:

- ESP32-P4
- USB Host Stack
- Cellular Networking
- RNDIS
- FreeRTOS
- ESP-IDF
- Embedded Internet Connectivity