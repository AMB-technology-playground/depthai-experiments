#!/usr/bin/env python3

import cv2
import depthai as dai

def printSystemInformation(info):
    m = 1024 * 1024 # MiB
    print(f"Drr used / total - {info.ddrMemoryUsage.used / m:.2f} / {info.ddrMemoryUsage.total / m:.2f} MiB")
    print(f"Cmx used / total - {info.cmxMemoryUsage.used / m:.2f} / {info.cmxMemoryUsage.total / m:.2f} MiB")
    print(f"LeonCss heap used / total - {info.leonCssMemoryUsage.used / m:.2f} / {info.leonCssMemoryUsage.total / m:.2f} MiB")
    print(f"LeonMss heap used / total - {info.leonMssMemoryUsage.used / m:.2f} / {info.leonMssMemoryUsage.total / m:.2f} MiB")
    t = info.chipTemperature
    print(f"Chip temperature - average: {t.average:.2f}, css: {t.css:.2f}, mss: {t.mss:.2f}, upa0: {t.upa:.2f}, upa1: {t.dss:.2f}")
    print(f"Cpu usage - Leon OS: {info.leonCssCpuUsage.average * 100:.2f}%, Leon RT: {info.leonMssCpuUsage.average * 100:.2f} %")
    print("----------------------------------------")

# Start defining a pipeline
pipeline = dai.Pipeline()

sysLog = pipeline.createSystemLogger()
sysLog.setRate(1)  # 1 Hz

# Send system info messages to the ESP32 via SPI
spi = pipeline.createSPIOut()
spi.setStreamName("sysinfo")
spi.setBusId(0)
sysLog.out.link(spi.input)

linkOut = pipeline.createXLinkOut()
linkOut.setStreamName("sysinfo")
sysLog.out.link(linkOut.input)

# Pipeline is defined, now we can connect to the device
with dai.Device(pipeline) as device:
    # Output queue will be used to get the system info
    qSysInfo = device.getOutputQueue(name="sysinfo", maxSize=4, blocking=False)

    while True:
        # Blocking call, will wait until a new data has arrived
        printSystemInformation(qSysInfo.get())
