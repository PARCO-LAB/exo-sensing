#!/usr/bin/env python3

import os
import glob
import csv
import cv2
import numpy as np

from concurrent.futures import ThreadPoolExecutor

import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from sensor_msgs.msg import CompressedImage, Image


# -----------------------------
# config
# -----------------------------

IMAGE_THREADS = 4
CSV_BUFFER_SIZE = 2000
OUTPUT_DIR = './camera_body'


# -----------------------------
# utils
# -----------------------------

def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def sanitize_topic(topic):
    return topic.replace("/", "_").strip("_")


def rosbag_time_to_sec(t):
    return t * 1e-9


# -----------------------------
# flatten ros message
# -----------------------------

def flatten_msg(msg, prefix=""):

    fields = {}

    if hasattr(msg, "__slots__"):

        for slot in msg.__slots__:

            val = getattr(msg, slot)
            name = slot.replace("_", "", 1)
            key = f"{prefix}{name}"

            if hasattr(val, "__slots__"):
                fields.update(flatten_msg(val, key + "_"))

            elif isinstance(val, (list, tuple)):
                fields[key] = list(val)

            else:
                fields[key] = val

    else:
        fields[prefix[:-1]] = msg

    return fields


# -----------------------------
# image saving (RGB)
# -----------------------------

def save_rgb_image(path, data):

    arr = np.frombuffer(data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)

    if img is not None:
        cv2.imwrite(path, img)


# -----------------------------
# depth saving (Z16 → PNG 16bit)
# -----------------------------

def save_depth_png(path, msg):

    depth = np.frombuffer(msg.data, dtype=np.uint16)
    depth = depth.reshape(msg.height, msg.width)

    # salva direttamente 16-bit (IMPORTANTISSIMO: nessuna normalizzazione)
    cv2.imwrite(path, depth)


# -----------------------------
# main
# -----------------------------

def main():
    record_dir = "/mnt/nvme/ros_bags/"
    all_path = glob.glob(record_dir+'*.bag')

    for bag_path in all_path: 
        output_dir = OUTPUT_DIR + bag_path.split('/')[-1].split('.')[0]

        # bag_path = "/mnt/nvme/ros_bags/all_data_2026-03-31_20-14-35.bag"
        # output_dir = "action1"

        ensure_dir(output_dir)

        storage_options = rosbag2_py.StorageOptions(
            uri=bag_path,
            storage_id="sqlite3"
        )

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topics = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in topics}

        print("\nTopics:")
        for t in topics:
            print(f"{t.name} -> {t.type}")

        # CSV
        csv_files = {}
        csv_writers = {}
        csv_buffers = {}
        csv_headers_written = {}
        csv_headers_per_topic = {}

        # image dirs
        image_dirs = {}

        executor = ThreadPoolExecutor(max_workers=IMAGE_THREADS)

        # prepare csv files
        for topic in type_map:

            clean = sanitize_topic(topic)
            csv_path = os.path.join(output_dir, clean + ".csv")

            f = open(csv_path, "w", newline="")
            writer = csv.writer(f)

            csv_files[topic] = f
            csv_writers[topic] = writer
            csv_buffers[topic] = []
            csv_headers_written[topic] = False
            csv_headers_per_topic[topic] = []

        msg_counter = 0

        # -----------------------------
        # reading bag
        # -----------------------------

        while reader.has_next():

            topic, data, t = reader.read_next()

            msg_type = type_map[topic]
            msg_class = get_message(msg_type)
            msg = deserialize_message(data, msg_class)

            writer = csv_writers[topic]

            timestamp = rosbag_time_to_sec(t)

            # -------------------------
            # RGB compressed image
            # -------------------------

            if isinstance(msg, CompressedImage):

                if topic not in image_dirs:

                    clean = sanitize_topic(topic)
                    img_dir = os.path.join(output_dir, clean)
                    ensure_dir(img_dir)

                    image_dirs[topic] = img_dir

                    writer.writerow([
                        "timestamp",
                        "sensor_timestamp",
                        "image_file"
                    ])

                    csv_headers_written[topic] = True

                img_dir = image_dirs[topic]

                # sensor timestamp
                if hasattr(msg, "header"):
                    st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    filename = f"{st}.jpg"
                else:
                    st = None
                    filename = f"{t}.jpg"

                path = os.path.join(img_dir, filename)

                executor.submit(save_rgb_image, path, msg.data)

                row = [timestamp, st, filename]

            # -------------------------
            # Depth image (Z16 → PNG)
            # -------------------------

            elif isinstance(msg, Image) and "aligned_depth_to_color" in topic:

                if topic not in image_dirs:

                    clean = sanitize_topic(topic)
                    img_dir = os.path.join(output_dir, clean)
                    ensure_dir(img_dir)

                    image_dirs[topic] = img_dir

                    writer.writerow([
                        "timestamp",
                        "sensor_timestamp",
                        "depth_file"
                    ])

                    csv_headers_written[topic] = True

                img_dir = image_dirs[topic]

                # sensor timestamp
                if hasattr(msg, "header"):
                    st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    filename = f"{st}.jpg"
                else:
                    st = None
                    filename = f"{t}.jpg"
                path = os.path.join(img_dir, filename)

                executor.submit(save_depth_png, path, msg)

                row = [timestamp, st, filename]

            # -------------------------
            # generic message
            # -------------------------

            else:

                flat = flatten_msg(msg)
                flat["timestamp"] = timestamp

                if not csv_headers_written[topic]:

                    headers = list(flat.keys())
                    csv_headers_per_topic[topic] = headers

                    writer.writerow(headers)
                    csv_headers_written[topic] = True

                else:

                    headers = csv_headers_per_topic[topic]

                    new_keys = [k for k in flat.keys() if k not in headers]

                    if new_keys:
                        print(f"[WARN] nuove colonne in {topic}: {new_keys}")
                        headers.extend(new_keys)

                headers = csv_headers_per_topic[topic]

                row = [flat.get(k, None) for k in headers]

            # -------------------------
            # buffering
            # -------------------------

            csv_buffers[topic].append(row)

            if len(csv_buffers[topic]) >= CSV_BUFFER_SIZE:
                writer.writerows(csv_buffers[topic])
                csv_buffers[topic].clear()

            msg_counter += 1

            if msg_counter % 10000 == 0:
                print(f"{msg_counter} messages processed")

        # -----------------------------
        # flush
        # -----------------------------

        for topic in csv_buffers:
            if csv_buffers[topic]:
                csv_writers[topic].writerows(csv_buffers[topic])

        for f in csv_files.values():
            f.close()

        executor.shutdown(wait=True)

        print("\nDataset extraction completed")
        print(f"Total messages: {msg_counter}")


if __name__ == "__main__":
    main()