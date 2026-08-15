# Pi service (`round2.service`)

Service name stays `round2`. The script it runs is `round2.py` in
`/home/pi/Documents/Test2_Round2`.

Copy or edit `/etc/systemd/system/round2.service` so `ExecStart` ends with
`round2.py`, then:

```bash
sudo systemctl daemon-reload
sudo systemctl restart round2
journalctl -u round2 -f
```

You should see `Loaded ONNX model` and later `>>> Sent WAYPOINT`.
Do not open Arduino Serial Monitor while this service is running.
