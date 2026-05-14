#!/usr/bin/env python3

import argparse
import json
import sys
import time
from collections import OrderedDict

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


PUBLISH_INTERVAL_SECONDS = 1.0

# Mock publisher bilgisayarında doğrudan bu listeyi düzenleyebilirsiniz.
MOCK_BINS = [
    {
        "node_id": 2,
        "lat": 39.87225,
        "lng": 32.73535,
        "weightKg": 8.0,
        "fillPercent": 25.0,
        "isFull": True,
    },
    {
        "node_id": 3,
        "lat": 39.8724,
        "lng": 32.7355,
        "weightKg": 9.5,
        "fillPercent": 45.0,
        "isFull": False,
    },
    {
        "node_id": 4,
        "lat": 39.87255,
        "lng": 32.73565,
        "weightKg": 13.0,
        "fillPercent": 82.0,
        "isFull": True,
    },
    {
        "node_id": 5,
        "lat": 39.8727,
        "lng": 32.7358,
        "weightKg": 10.0,
        "fillPercent": 60.0,
        "isFull": False,
    },
    {
        "node_id": 6,
        "lat": 39.87285,
        "lng": 32.73595,
        "weightKg": 15.0,
        "fillPercent": 91.0,
        "isFull": True,
    },
    {
        "node_id": 7,
        "lat": 39.873,
        "lng": 32.7361,
        "weightKg": 6.0,
        "fillPercent": 10.0,
        "isFull": False,
    },
]


def coerce_bool(value, default=False):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "y", "on"}:
            return True
        if normalized in {"false", "0", "no", "n", "off"}:
            return False
    return default


def coerce_float(value, default=None):
    if value in (None, ""):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def coerce_int(value, default=None):
    if value in (None, ""):
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def require_paho():
    if mqtt is None:
        raise RuntimeError(
            "paho-mqtt is not installed. Install it with 'pip3 install paho-mqtt'."
        )


def normalize_bins(raw_bins):
    bins = []
    for raw_bin in raw_bins:
        if not isinstance(raw_bin, dict):
            raise ValueError("Each mock bin entry must be a JSON object.")

        if not coerce_bool(raw_bin.get("enabled", True), default=True):
            continue

        bin_id = coerce_int(raw_bin.get("node_id"))
        if bin_id is None:
            raise ValueError("Each mock bin entry must contain a numeric 'node_id'.")

        fill_percent = coerce_float(raw_bin.get("fillPercent"), default=0.0)
        fill_percent = max(0.0, min(100.0, fill_percent))

        bins.append(
            {
                "node_id": bin_id,
                "lat": coerce_float(raw_bin.get("lat")),
                "lng": coerce_float(raw_bin.get("lng")),
                "weightKg": coerce_float(raw_bin.get("weightKg"), default=0.0),
                "fillPercent": fill_percent,
                "isFull": coerce_bool(raw_bin.get("isFull"), default=False),
            }
        )

    if not bins:
        raise ValueError("No enabled mock bins configured.")

    return bins


class MockBinStatusPublisher:
    def __init__(self, args):
        require_paho()
        self.host = args.host
        self.port = args.port
        self.status_topic = args.status_topic
        self.clear_topic = args.clear_topic
        self.client_id = args.client_id
        self.username = args.username
        self.password = args.password
        self.qos = args.qos
        self.retain = args.retain
        self.shutdown_requested = False
        self.bins = normalize_bins(MOCK_BINS)
        self.current_is_full_by_id = {
            bin_payload["node_id"]: bin_payload["isFull"] for bin_payload in self.bins
        }

        self.client = mqtt.Client(client_id=self.client_id)
        if self.username:
            self.client.username_pw_set(self.username, self.password)

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            client.subscribe(self.clear_topic, qos=self.qos)
            print(
                "Connected to broker %s:%d and subscribed to %s"
                % (self.host, self.port, self.clear_topic)
            )
            return

        print("MQTT connect failed with rc=%s" % rc)

    def on_disconnect(self, client, userdata, rc, properties=None):
        if rc != 0 and not self.shutdown_requested:
            print("MQTT disconnected unexpectedly with rc=%s" % rc)

    def on_message(self, client, userdata, message):
        try:
            payload = json.loads(message.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            print("Ignoring invalid clear payload: %s" % error)
            return

        if not isinstance(payload, dict):
            return

        if not coerce_bool(payload.get("emptied")):
            return

        bin_id = coerce_int(payload.get("id"))
        if bin_id is None:
            return

        self.current_is_full_by_id[bin_id] = False
        print("Cleared mock alarm for bin %d" % bin_id)

    def build_status_payload(self, bin_payload):
        payload = OrderedDict()
        payload["node_id"] = bin_payload["node_id"]
        if bin_payload["lat"] is not None:
            payload["lat"] = bin_payload["lat"]
        if bin_payload["lng"] is not None:
            payload["lng"] = bin_payload["lng"]
        payload["weightKg"] = round(bin_payload["weightKg"], 1)
        payload["fillPercent"] = round(bin_payload["fillPercent"], 1)
        payload["isFull"] = self.current_is_full_by_id.get(
            bin_payload["node_id"], bin_payload["isFull"]
        )
        return payload

    def publish_once(self):
        for bin_payload in self.bins:
            status_payload = self.build_status_payload(bin_payload)
            payload_text = json.dumps(status_payload)
            info = self.client.publish(
                self.status_topic,
                payload_text,
                qos=self.qos,
                retain=self.retain,
            )

            if getattr(info, "rc", 0) != 0:
                print(
                    "Publish failed for bin %d with rc=%s"
                    % (bin_payload["node_id"], getattr(info, "rc", 0))
                )
                continue

            print("Published %s" % payload_text)

    def run(self):
        self.client.connect(self.host, self.port, keepalive=60)
        self.client.loop_start()
        print("Mock publisher started. Edit MOCK_BINS in this file to change values.")

        try:
            while not self.shutdown_requested:
                self.publish_once()
                time.sleep(PUBLISH_INTERVAL_SECONDS)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown_requested = True
            self.client.loop_stop()
            self.client.disconnect()


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Publish mock trash-bin status messages to MQTT."
    )
    parser.add_argument("--host", required=True, help="MQTT broker host/IP")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument(
        "--status-topic",
        default="bin/status",
        help="MQTT topic for status publishing",
    )
    parser.add_argument(
        "--clear-topic",
        default="bin/cleared",
        help="MQTT topic to subscribe for clear messages",
    )
    parser.add_argument(
        "--client-id",
        default="mock_bin_status_publisher",
        help="MQTT client id",
    )
    parser.add_argument("--username", default="", help="MQTT username")
    parser.add_argument("--password", default="", help="MQTT password")
    parser.add_argument("--qos", type=int, default=0, help="MQTT QoS")
    parser.add_argument(
        "--retain",
        action="store_true",
        help="Publish retained MQTT messages",
    )
    return parser


def main():
    parser = build_argument_parser()
    args = parser.parse_args()

    try:
        publisher = MockBinStatusPublisher(args)
        publisher.run()
    except Exception as error:
        print("Mock publisher failed: %s" % error, file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
