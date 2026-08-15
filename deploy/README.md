# Pi service (`round2.service`)

Copy to the Pi:

```bash
sudo cp round2.service /etc/systemd/system/round2.service
```

If the Python file is **not** named `detect.py`, edit `ExecStart` so the last path matches your file, for example:

```
ExecStart=/home/pi/Documents/Test2_Round2/venv/bin/python -u /home/pi/Documents/Test2_Round2/wro_block_detector.py
```

Then:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now round2
sudo systemctl restart round2
journalctl -u round2 -f
```

You should see `Loaded ONNX model` and later `>>> Sent WAYPOINT`.
Do not open Arduino Serial Monitor while this service is running.
