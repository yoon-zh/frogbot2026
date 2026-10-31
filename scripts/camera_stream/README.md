# Camera Stream (WebRTC via go2rtc)

This directory configures low-latency WebRTC video streaming from the Intel RealSense D455f camera to FrogBoard using [go2rtc](https://github.com/AlexxIT/go2rtc).

***

## 1. Initial Setup on Jetson Orin NX

### Step 1: Download `go2rtc` (ARM64)
```bash
wget https://github.com/AlexxIT/go2rtc/releases/latest/download/go2rtc_linux_arm64 -O go2rtc
chmod +x go2rtc
sudo mv go2rtc /usr/local/bin/
```

### Step 2: Install dependencies

Install ffmpeg:
```bash
sudo apt update
sudo apt install -y ffmpeg
```

Verify the version is at least 5.0:
```bash
ffmpeg -version
```

If not, then you need a static version of ffmpeg:

```bash
cd /tmp
wget -O ffmpeg-arm64.tar.xz https://johnvansickle.com/ffmpeg/releases/ffmpeg-release-arm64-static.tar.xz
tar -xvf ffmpeg-arm64.tar.xz
sudo cp ffmpeg-*-arm64-static/ffmpeg /usr/local/bin/
sudo cp ffmpeg-*-arm64-static/ffprobe /usr/local/bin/
sudo chmod +x /usr/local/bin/ffmpeg /usr/local/bin/ffprobe
hash -r
cd ~/frogbot2026
```

Then make sure the ffmpeg path is correct in [`go2rtc.yaml`](go2rtc.yaml).

### Step 3: Identify the RealSense RGB Device Node
The RealSense D455f registers multiple video nodes (Depth, IR, Color, Metadata). Run:

```bash
python3 -c '
import cv2
for i in range(6):
    cap = cv2.VideoCapture(i)
    if not cap.isOpened():
        print(f"/dev/video{i}: metadata node (cannot stream)")
        continue
    ret, frame = cap.read()
    if ret and len(frame.shape) == 3 and frame.shape[2] == 3:
        print(f">>> /dev/video{i} is RGB (Frame: {frame.shape}) <<<")
    elif ret:
        print(f"/dev/video{i}: opened (shape: {frame.shape}) - Depth or IR")
    else:
        print(f"/dev/video{i}: opened but returned no frame (Depth/IR raw)")
    cap.release()
' | grep ">>> /dev/video"
```

Look at the last `/dev/video*` output, and make sure it is the same one written in [`go2rtc.yaml`](go2rtc.yaml)

***

## 2. Running & Testing

In the robot, run:

```bash
go2rtc -config scripts/camera_stream/go2rtc.yaml
```
Open a browser on your phone/laptop connected to the robot Wi-Fi:
- Low-latency stream: `https://robotfrogboard.yoonzh.com:1985/stream.html?src=robot_cam&mode=webrtc`
- Dashboard: `http://robotfrogboard.yoonzh.com:1985`

***

## 3. Enable Systemd Daemon

```bash
sudo cp ~/frogbot2026/scripts/camera_stream/camera_stream.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable camera_stream.service
sudo systemctl start camera_stream.service
```

To check service status:
```bash
systemctl status camera_stream.service
```