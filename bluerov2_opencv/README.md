# cv-intro

## Capture Video Stream

For [`network_stream_capture.py`](network_stream_capture.py) and [`network_stream_capture_notebook.ipynb`](network_stream_capture_notebook.ipynb) to work, you will need to be connected to the `AUVC` network.

In the `backseat` computer, open the `wpa` configuration file:

```bash
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```

Add the `AUVC` network to the file and give it a higher priority.
The higher prority will ensure that your RPi will connect to `AUVC`, not `MIT` when `AUVC` is available

```bash
network={
        ssid="AUVC"
        psk="g0_fast!"
        priority=2
}
```

### Switch between networks

To force the RPi to connect to the network of your choice, you can use the commands below. This will be useful when you need internet access.

- See list of networks

```bash
sudo wpa_cli -i wlan0 list_networks
```

- Select desired network
  Replace 0 with the desired network

```bash
sudo wpa_cli -i wlan0 select_network 0
```
