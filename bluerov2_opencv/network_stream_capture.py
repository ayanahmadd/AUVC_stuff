import argparse

import cv2


def main(ip_address):
    vcap = cv2.VideoCapture(f"rtsp://{ip_address}:8554/rovcam")

    try:
        while True:
            # Obtain the frame
            ret, frame = vcap.read()

            # Check frame was received successfully
            if ret:
                print(" YOU GOT THIS ")
                print(frame.shape)
                # TODO: Do something with the frame here

            else:
                pass

    except KeyboardInterrupt:
        # Close the connection
        vcap.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Network Stream Capture")
    parser.add_argument("--ip", type=str, help="IP Address of the Network Stream")
    args = parser.parse_args()

    main(args.ip)
