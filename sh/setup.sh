sudo echo "/dev/disk/by-id/usb-MBED_microcontroller_0671FF535155878281081742-0:0 /mnt/usb-MBED_microcontroller_0671FF535155878281081742-0:0 auto rw,user,exec,umask=000,nosuid,nodev,nofail,noauto,x-gvfs$">>/etc/fstab

sudo echo "/dev/disk/by-id/usb-MBED_microcontroller_0669FF535155878281133010-0:0 /mnt/usb-MBED_microcontroller_0669FF535155878281133010-0:0 auto rw,user,exec,umask=000,nosuid,nodev,nofail,noauto,x-gvfs$">>/etc/fstab

sudo cp 99-usb-input.rules . /etc/udev/rules.d/

sudo udevadm control --reload-rules



