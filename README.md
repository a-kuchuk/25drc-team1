# drc-team1
create 2025 drc team 1

# packages to pip install
opencv-python
TIME-python
numpy
imutils
python-math 
pigpio
RPi.GPIO
pigpio

install unzip

install python-venv

# other commands to run after ssh:
ssh team1-rpi@raspberrypi.local

sudo python3 -m venv ~/venv

sudo chmod 777 /dev/mem /dev/gpiomem 
cd 25drc-team1/rpi/
source ~/venv/bin/activate
sudo pigpiod