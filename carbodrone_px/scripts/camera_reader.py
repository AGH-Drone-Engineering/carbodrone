from picamera2 import Picamera2
import socket
import struct


def main():
    cap = Picamera2()
    cap.configure(cap.create_preview_configuration(main={"format": 'RGB888', "size": (1280, 720)}))
    cap.start()

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect('/tmp/carbodrone_px_camera')

    while True:
        frame = cap.capture_array('main')

        frame_bytes = frame.tobytes()
        height = frame.shape[0]
        width = frame.shape[1]
        channels = frame.shape[2]
        sock.sendall(struct.pack('<III', height, width, channels))
        sock.sendall(frame_bytes)


if __name__ == '__main__':
    main()
