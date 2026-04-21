#!/bin/bash
set -e

# Copy all service files
sudo cp ./start_container.service /etc/systemd/system/
sudo cp ./power_monitor.service /etc/systemd/system/
sudo cp ./soft_start.service /etc/systemd/system/

# Copy scripts
sudo cp ../docker/start_docker.sh /usr/local/bin/
sudo cp ./power_monitor.py /usr/local/bin/

sudo chmod +x /usr/local/bin/start_docker.sh

sudo systemctl daemon-reload
sudo systemctl enable start_container.service
sudo systemctl enable power_monitor.service
sudo systemctl enable soft_start.service

# Start all services
sudo systemctl restart start_container.service
sudo systemctl restart power_monitor.service
sudo systemctl restart soft_start.service

export PYTHONPATH=/home/pi/PFE_Eurobot_2026/systemd
alias power='/home/pi/PFE_Eurobot_2026/venv/bin/python3 -c "import power_monitor; power_monitor.status()"'