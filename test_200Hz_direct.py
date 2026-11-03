import asyncio
import statistics
import socket
import time

from bleak import BleakScanner, BleakClient


# =========================================================
# BLE / MUSE SETUP
# =========================================================

CMD_UUID = "d5913036-2d8a-41ee-85b9-4e361aa5c8a7"
DATA_UUID = "09bf2c52-d1d9-c0b7-4145-475964544307"

TEST_DURATION = 180   # 3 minutes

ACQUISITION_HZ = 200
EXPECTED_PERIOD_MS = 1000 / ACQUISITION_HZ   # 5 ms


# =========================================================
# TELEPLOT SETUP
# =========================================================

TELEPLOT_ADDR = ("127.0.0.1", 47269)

teleplot_sock = socket.socket(
    socket.AF_INET, # use IPv4
    socket.SOCK_DGRAM # UDP protocal 
)


def send_telemetry(name, value):

    now_ms = time.time() * 1000

    message = f"{name}:{now_ms}:{value}|g"

    teleplot_sock.sendto(
        message.encode(),
        TELEPLOT_ADDR
    )


# =========================================================
# DATA STORAGE
# =========================================================

muse_timestamps = []

gyro_x_data = []
gyro_y_data = []
gyro_z_data = []


# =========================================================
# BLE CALLBACK
# =========================================================

def data_callback(sender, data):

    # Packet layout:
    # bytes 0-7   : header
    # bytes 8-13  : gyro XYZ
    # bytes 14-19 : accel XYZ
    # bytes 20-25 : quaternion
    # bytes 26-31 : timestamp

    if len(data) < 32:
        return


    # -----------------------------------------------------
    # Decode gyro XYZ
    # -----------------------------------------------------

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


    # Store gyro values.
    gyro_x_data.append(gyro_x)
    gyro_y_data.append(gyro_y)
    gyro_z_data.append(gyro_z)


    # -----------------------------------------------------
    # Send gyro XYZ to Teleplot
    # -----------------------------------------------------

    send_telemetry("gyro_x", gyro_x)
    send_telemetry("gyro_y", gyro_y)
    send_telemetry("gyro_z", gyro_z)


    # -----------------------------------------------------
    # Decode Muse timestamp
    # -----------------------------------------------------

    muse_time_ms = int.from_bytes(
        data[26:32],
        byteorder="little",
        signed=False
    )

    muse_timestamps.append(muse_time_ms)


# =========================================================
# MAIN
# =========================================================

async def main():

    device = await BleakScanner.find_device_by_name("muse_1")

    if device is None:
        print("muse_1 not found")
        return

    print("Found:", device.name)

    async with BleakClient(device) as client:

        print("Connected:", client.is_connected)

        await client.start_notify(
            DATA_UUID,
            data_callback
        )


        # -------------------------------------------------
        # Start direct streaming at 200 Hz
        # -------------------------------------------------
        #
        # 0x08 = direct streaming
        #
        # 0x33 00 00 =
        # gyro + accel + quaternion + timestamp
        #
        # final 0x08 = 200 Hz acquisition
        #

        start_command = bytearray([
            0x02,
            0x05,
            0x08,
            0x33, 0x00, 0x00,
            0x08
        ])


        muse_timestamps.clear()

        gyro_x_data.clear()
        gyro_y_data.clear()
        gyro_z_data.clear()


        print("Starting 200 Hz direct acquisition...")
        print("Streaming gyro X/Y/Z to Teleplot...")
        print(f"Running for {TEST_DURATION} seconds...")


        await client.write_gatt_char(
            CMD_UUID,
            start_command,
            response=True
        )


        await asyncio.sleep(TEST_DURATION)


        # -------------------------------------------------
        # Stop Muse
        # -------------------------------------------------

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

        print("Acquisition stopped")

    analyze_results()


# =========================================================
# ANALYSIS
# =========================================================

def analyze_results():

    if len(muse_timestamps) < 2:
        print("Not enough samples received.")
        return

    received = len(muse_timestamps)

    muse_intervals = [
        muse_timestamps[i] - muse_timestamps[i - 1]
        for i in range(1, received)
    ]


    # -----------------------------------------------------
    # Late intervals
    # -----------------------------------------------------

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


    # -----------------------------------------------------
    # Estimate missing samples
    # -----------------------------------------------------

    estimated_missing = 0

    for interval in muse_intervals:

        expected_periods = round(
            interval / EXPECTED_PERIOD_MS
        )

        if expected_periods > 1:
            estimated_missing += (
                expected_periods - 1
            )

    estimated_total = (
        received + estimated_missing
    )

    estimated_loss_percent = (
        estimated_missing
        / estimated_total
        * 100
    )


    # -----------------------------------------------------
    # Results
    # -----------------------------------------------------

    print(
        "\n===== 200 Hz DIRECT STREAM RESULTS ====="
    )

    print("\nSamples:")

    print(
        "Samples received:",
        received
    )

    print(
        f"Expected samples in {TEST_DURATION}s: "
        f"{ACQUISITION_HZ * TEST_DURATION}"
    )

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


    print("\nEstimated packet loss:")

    print(
        "Estimated missing samples:",
        estimated_missing
    )

    print(
        f"Estimated sample loss: "
        f"{estimated_loss_percent:.3f}%"
    )


asyncio.run(main())