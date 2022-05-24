### ABOUT 

For running on the jetson nano, if you are getting usb suspend errors, \
```sudo vi /boot/extlinux/extlinux.conf``` \
At the end of APPEND add ```usbcore.autosuspend=-1``` \
With a space between the end of APPEND and the start of 'usbcore..' \
Then reboot. \
Upon reboot check ```cat /proc/cmdline``` to verify the change was put into place
