# vils

For send transimited CAN signal over UDP

## Requiremetn

- Over Python 3.9 
- [python-can](https://python-can.readthedocs.io/) package

```bash
pip install python-can
```

## python exe 파일 생성하기

'''bash
pyinstaller `  --onefile `  --noconsole `  --clean `  --collect-all libTSCANAPI `  --add-data "libTSCANAPI\windows\x64\libTSCAN.dll;libTSCANAPI/windows/x64" `  --add-data "libTSCANAPI\windows\x64\libTSH.dll;libTSCANAPI/windows/x64" ` tosun_can_udp_ui.py
'''

An .exe file is generated in dist directory!

### essential file
- .dbc file (for signal define)

### optional file
- .conf file (for saving/loading to transmit signal CAN to UDP)
