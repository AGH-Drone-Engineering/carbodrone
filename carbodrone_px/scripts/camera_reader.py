import cv2
import socket
import struct


def main():
    cap = cv2.VideoCapture(0)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect('/tmp/carbodrone_px_camera')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_bytes = frame.tobytes()
        height = frame.shape[0]
        width = frame.shape[1]
        channels = frame.shape[2]
        sock.sendall(struct.pack('<III', height, width, channels))
        sock.sendall(frame_bytes)

    cap.release()
    sock.close()


if __name__ == '__main__':
    main()
