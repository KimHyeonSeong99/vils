# vils

이 저장소에는 TOSUN CAN USB 장치로부터 CAN 프레임을 수집하고 필요한 정보를
UDP로 전송하기 위한 간단한 Python 브리지 스크립트가 포함되어 있습니다.

## 요구 사항

- Python 3.9 이상
- [python-can](https://python-can.readthedocs.io/) 패키지

```bash
pip install python-can
```

## 구성

`config/example_config.json` 파일을 복사하여 환경에 맞도록 수정합니다.

```bash
cp config/example_config.json config/my_setup.json
```

구성 파일의 주요 항목은 다음과 같습니다.

- `can`: python-can에서 사용하는 인터페이스 정보입니다. TOSUN CAN USB가
  SocketCAN을 제공한다면 `interface`는 `"socketcan"`, `channel`은 `"can0"`처럼
  설정합니다. 다른 드라이버(`"pcan"`, `"usb2can"` 등)를 사용할 경우 해당 드라이버에
  맞게 값을 조정하십시오.
- `udp`: 수신할 호스트와 포트 번호입니다.
- `signals`: 특정 CAN ID에서 추출할 신호 정의 목록입니다. 각 신호는 다음 필드를
  가집니다.
  - `name`: UDP로 전송할 때 사용될 신호 이름
  - `can_id`: 프레임의 식별자. 16진수 문자열(`"0x100"`) 또는 정수로 지정할 수 있습니다.
  - `start_bit`: 추출할 비트의 시작 위치
  - `length`: 추출할 비트 길이
  - `byte_order`: `"little"`(기본값) 또는 `"big"`
  - `signed`: 부호가 있는 값인지 여부 (기본값 `false`)
  - `scale`, `offset`: 실제 물리 값을 계산하기 위한 선형 변환 계수
- `include_raw_frame`: `true`로 설정하면 원본 프레임 데이터를 16진수 문자열로 함께 전송합니다.

필요한 신호가 없다면 `signals` 배열을 비워 두어도 됩니다. 이 경우 해당 CAN ID는
UDP로 전달되지 않습니다.

## 실행 방법

```bash
python -m src.can_udp_bridge --config config/my_setup.json --log-level INFO
```

스크립트는 지정된 CAN 인터페이스에서 프레임을 수신하고 설정된 신호를
추출하여 JSON 형태로 UDP 패킷에 담아 전송합니다. 전송되는 예시는 다음과 같습니다.

```json
{
  "timestamp": 1709875306.123,
  "can_id": 256,
  "extended": false,
  "signals": {
    "vehicle_speed": 12.34
  },
  "raw_data": "1122334455667788"
}
```

## 참고 사항

- 실환경에서는 TOSUN CAN USB 드라이버가 OS에 제대로 설치되어 있고 python-can에서
  해당 인터페이스를 인식할 수 있어야 합니다.
- UDP 수신 측에서는 JSON 메시지를 파싱하여 필요한 정보를 추출할 수 있습니다.
- 로그 레벨을 `DEBUG`로 올리면 프레임 전송 내역을 자세히 확인할 수 있습니다.

## 윈도우용 실행 파일 만들기

Python이 설치되어 있지 않은 PC에서 사용하거나 배포가 필요하다면
[PyInstaller](https://pyinstaller.org/)를 이용해 실행 파일(`.exe`)로 묶을 수 있습니다.

1. PyInstaller 설치

   ```powershell
   pip install pyinstaller
   ```

2. 저장소 루트에서 제공하는 빌드 스크립트를 실행합니다.

   ```powershell
   python scripts/build_executable.py
   ```

   Windows에서는 `dist\can_udp_bridge.exe`가 생성됩니다. 다른 운영체제에서도 같은
   명령을 사용할 수 있으며, 해당 플랫폼용 실행 파일이 만들어집니다.

3. 실행 파일과 함께 사용할 구성 파일을 같은 폴더에 배치합니다.

   ```powershell
   copy config\example_config.json can_config.json
   ```

   생성된 `can_udp_bridge.exe`를 다음과 같이 실행할 수 있습니다.

   ```powershell
   .\dist\can_udp_bridge.exe --config can_config.json --log-level INFO
   ```

   `--config`에 전달하는 경로는 실행 파일이 위치한 폴더 기준 상대 경로이거나
   절대 경로를 사용할 수 있습니다.
