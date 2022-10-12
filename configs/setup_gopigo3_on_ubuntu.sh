#!/bin/bash

sudo git clone -b install_on_ubuntu http://www.github.com/DexterInd/GoPiGo3.git /home/pi/Dexter/GoPiGo3

sudo curl -kL dexterindustries.com/update_tools | bash -s -- --system-wide --use-python3-exe-too --install-deb-debs --install-python-package

sudo apt install -y --no-install-recommends python3-pip python3-numpy python3-curtsies

sudo git clone https://github.com/DexterInd/DI_Sensors.git /home/pi/Dexter/DI_Sensors

sudo apt install -y python3-rpi.gpio

sudo apt install -y unzip

# === install gcc and make
sudo apt install -y gcc
sudo apt install -y make

# === pigpiod

wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
cd ..
rm master.zip


sudo cp /home/pi/utils/pigpiod.service /etc/systemd/system
sudo systemctl enable pigpiod.service
sudo systemctl start pigpiod.service
systemctl status pigpiod.service

# === setup RFR_Tools
sudo git clone https://github.com/DexterInd/RFR_Tools.git /home/pi/Dexter/lib/Dexter/RFR_Tools
sudo apt  install -y libffi-dev

cd /home/pi/Dexter/lib/Dexter//RFR_Tools/miscellaneous/
sudo python3 setup.py install



# === wiringPi
cd /home/pi/Dexter/lib
git clone https://github.com/DexterInd/wiringPi/
cd wiringPi
sudo chmod +x ./build
sudo ./build

# ==== GPG3_POWER SERVICE ===
cd ~
sudo cp /home/pi/Dexter/GoPiGo3/Install/gpg3_power.service /etc/systemd/system
sudo chmod 644 /etc/systemd/system/gpg3_power.service
sudo systemctl daemon-reload
sudo systemctl enable gpg3_power.service
sudo systemctl start gpg3_power.service
systemctl status gpg3_power.service

# ==== SETUP GoPiGo3 and DI_Sensors Python3 eggs
cd /home/pi/Dexter/GoPiGo3/Software/Python
sudo python3 setup.py install
cd /home/pi/Dexter/DI_Sensors/Python
sudo python3 setup.py install

# ==== Setup non-root access rules ====

sudo cp /home/pi/utils/99-com.rules /etc/udev/rules.d

# === ESPEAK-NG
echo "To install espeak-ng"
echo "sudo apt install -y espeak-ng"
# sudo apt install -y espeak-ng
echo "To allow python to call espeak-ng"
echo "sudo pip3 install py-espeak-ng"
# sudo pip3 install py-espeak-ng
# espeak-ng "Ok to reboot now"

echo "Check that the mutex stuff will be available"
echo "unzip -l /usr/local/lib/python*/dist-packages/Dexter*.egg"
echo "unzip -l /usr/local/lib/python3.10/dist-packages/Dexter_AutoDetection_and_I2C_Mutex-0.0.0-py3.10.egg"

echo "Done gopigo3 setup - ready for reboot"
