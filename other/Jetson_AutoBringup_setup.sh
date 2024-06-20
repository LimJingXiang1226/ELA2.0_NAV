cd ~
wget https://raw.githubusercontent.com/LimJingXiang1226/ELA2.0_NAV/main/other/ELA2.0_BRINGUP.sh && chmod +x ELA2.0_BRINGUP.sh
cd /etc/systemd/system
sudo wget https://raw.githubusercontent.com/LimJingXiang1226/ELA2.0_NAV/main/other/ela2.service && sudo systemctl daemon-reload && sudo systemctl enable ela2.service
