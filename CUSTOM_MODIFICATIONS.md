# PX4-Optimized: Palletrone 4 kg DShot 변경본

이 소스 트리는 Pixhawk 6X용 기존 Palletrone custom firmware를 다음 하드웨어에 맞게 수정한 버전이다.

- 4.0 kg X-frame
- CoG–motor arm 0.181 m
- 기존 관성의 1/2
- T-Motor F60 PRO V-LV 2020KV + 6S
- AUX 1–4 DShot600
- force-to-DShot lookup model
- allocator 및 실제 DShot 명령 80% 제한
- 구형 대형 기체 gain의 일회성 targeted reset
- MAIN/PX4IO 설정 유지

전체 변경 파일, 수치, 빌드 및 실기 확인 절차는 [`README_PALLETRONE_4KG_DSHOT.md`](README_PALLETRONE_4KG_DSHOT.md)를 참조한다.
