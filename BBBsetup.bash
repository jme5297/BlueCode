cd build
echo Copying Interface code...
bash BBBcopyCode
echo Setting PRUs to idle mode...
./pwm_disabler1
./pwm_disabler2

echo Configuring PRU pins...
sudo config-pin P9.27 pruout
sudo config-pin P8.27 pruout
echo Configuring GPS pin...
sudo config-pin P9.11 uart
sudo config-pin P9.13 uart
echo Configuring GPIO ports...
sudo config-pin P9.23 gpio
sudo config-pin P9.25 gpio
sudo config-pin P8.12 gpio
sudo config-pin P8.14 gpio
sudo echo out > /sys/class/gpio/gpio49/direction
sudo echo out > /sys/class/gpio/gpio117/direction
echo Disabling GPIOs...
sudo echo 0 > /sys/class/gpio/gpio49/value
sudo echo 0 > /sys/class/gpio/gpio117/value
