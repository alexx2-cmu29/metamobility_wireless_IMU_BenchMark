import asyncio
import statistics
import socket
import time

from bleak import BleakScanner, BleakClient
from bleak.exc import BleakError


CMD_UUID = "d5913036-2d8a-41ee-85b9-4e361aa5c8a7"
DATA_UUID = "09bf2c52-d1d9-c0b7-4145-475964544307"

TEST_DURATION = 180

ACQUISITION_HZ = 100
EXPECTED_PERIOD_MS = 1000 / ACQUISITION_HZ


TELEPLOT_ADDR = ("172.26.99.90", 47269)

teleplot_sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)


def send_telemetry(name, value):
    now_ms = time.time() * 1000
    message = f"{name}:{now_ms}:{value}|g"
    teleplot_sock.sendto(message.encode(), TELEPLOT_ADDR)


muse_timestamps = []

gyro_x_data = []
gyro_y_data = []
gyro_z_data = []


def data_callback(sender, data):

    if len(data) < 32:
        return

    gyro_x = int.from_bytes(
        data[8:10],
        byteorder="little",
        signed=True
    )

    gyro_y = int.from_bytes(
        data[10:12],
        byteorder="little",
        signed=True
    )

    gyro_z = int.from_bytes(
        data[12:14],
        byteorder="little",
        signed=True
    )

    gyro_x_data.append(gyro_x)
    gyro_y_data.append(gyro_y)
    gyro_z_data.append(gyro_z)

    send_telemetry("gyro_x", gyro_x)
    send_telemetry("gyro_y", gyro_y)
    send_telemetry("gyro_z", gyro_z)

    muse_time_ms = int.from_bytes(
        data[26:32],
        byteorder="little",
        signed=False
    )

    muse_timestamps.append(muse_time_ms)


async def wait_for_services(client, timeout=10):

    start = time.monotonic()

    while time.monotonic() - start < timeout:

        try:
            services = client.services

            if len(services) > 0:
                print("GATT services discovered")
                return

        except BleakError:
            pass

        await asyncio.sleep(0.2)

    raise RuntimeError(
        "Timed out waiting for GATT service discovery"
    )


async def main():

    device = await BleakScanner.find_device_by_name("muse_1")

    if device is None:
        print("muse_1 not found")
        return

    print("Found:", device.name)

    async with BleakClient(device) as client:

        print("Connected:", client.is_connected)

        # Jetson / Linux:
        # wait until BlueZ has finished discovering GATT services
        await wait_for_services(client)

        await client.start_notify(
            DATA_UUID,
            data_callback
        )

        start_command = bytearray([
            0x02,
            0x05,
            0x08,
            0x33, 0x00, 0x00,
            0x04
        ])

        muse_timestamps.clear()
        gyro_x_data.clear()
        gyro_y_data.clear()
        gyro_z_data.clear()

        print("Starting 100 Hz direct acquisition...")
        print("Streaming gyro X/Y/Z to Teleplot...")
        print(f"Running for {TEST_DURATION} seconds...")

        await client.write_gatt_char(
            CMD_UUID,
            start_command,
            response=True
        )

        await asyncio.sleep(TEST_DURATION)

        stop_command = bytearray([
            0x02,
            0x01,
            0x02
        ])

        await client.write_gatt_char(
            CMD_UUID,
            stop_command,
            response=True
        )

        await client.stop_notify(DATA_UUID)

        print("Acquisition stopped")

    analyze_results()


def analyze_results():

    if len(muse_timestamps) < 2:
        print("Not enough samples received.")
        return

    received = len(muse_timestamps)

    muse_intervals = [
        muse_timestamps[i] - muse_timestamps[i - 1]
        for i in range(1, received)
    ]

    late_intervals = [
        interval
        for interval in muse_intervals
        if interval > EXPECTED_PERIOD_MS
    ]

    late_count = len(late_intervals)

    late_percent = (
        late_count / len(muse_intervals) * 100
    )

    if late_count > 0:
        average_excess = statistics.mean(
            interval - EXPECTED_PERIOD_MS
            for interval in late_intervals
        )
    else:
        average_excess = 0

    expected_samples = ACQUISITION_HZ * TEST_DURATION
    missing_samples = max(0, expected_samples - received)
    loss_percent = (
        missing_samples / expected_samples
    ) * 100

    print("\n===== 100 Hz DIRECT STREAM RESULTS =====")

    print("\nSamples:")
    print("Samples received:", received)
    print("Expected samples:", expected_samples)
    print("Missing samples:", missing_samples)
    print(f"Packet loss: {loss_percent:.3f}%")

    print(
        f"Average Muse interval: "
        f"{statistics.mean(muse_intervals):.3f} ms"
    )

    print(
        f"Minimum Muse interval: "
        f"{min(muse_intervals):.3f} ms"
    )

    print(
        f"Maximum Muse interval: "
        f"{max(muse_intervals):.3f} ms"
    )

    print("\nLate intervals:")

    print(
        "Intervals exceeding 10 ms:",
        late_count
    )

    print(
        f"Percent exceeding 10 ms: "
        f"{late_percent:.3f}%"
    )

    print(
        f"Average amount over 10 ms: "
        f"{average_excess:.3f} ms"
    )


asyncio.run(main())