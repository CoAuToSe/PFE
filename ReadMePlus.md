# Extras

## Camera on Husky A300

### Husky A300

#### Connect to the Husky A300

```bash
ssh robot@192.168.50.254
clearpath
```

#### Setup

```bash
sudo apt update
sudo apt install -y ustreamer
```

#### Run

```bash 
ustreamer --device=/dev/video0 --host=0.0.0.0 --port=8080 --format=MJPEG -- resolution=1280x720 --desired-fps=30
```

### External computer

#### Setup

```bash
sudo apt install -y v4l2loopback-dkms ffmpeg
sudo modprobe v4l2loopback devices=1 video_nr=10 card_label="RemoteCam" exclusive_caps=1
```

#### Create virtual camera

```bash
ffmpeg -i http://SERVEUR_IP:8080/stream -vf format=yuv420p -f v4l2 /dev/video10
```

#### See video stream

```bash
ffplay /dev/video10
```