#!/bin/bash
# can setup script
cp can_setup.sh  /usr/bin/can_setup.sh
chmod a+x /usr/bin/can_setup.sh
# can service
cp can-setup.service /lib/systemd/system/can-setup.service
systemctl enable can-setup.service
