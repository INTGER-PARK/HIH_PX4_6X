# Palletrone Admittance

`mc_pos_control`에 통합된 Palletrone 4-DoF admittance 기능의 관련 파일을 한곳에 모은 디렉터리다.

- `AdmittanceManager.hpp`: uORB 입력, 상태 전이, 안전 조건 및 setpoint 연동
- `Admittance4D.hpp`: uORB와 독립적인 4축 admittance 동역학 코어
- `palletrone_admittance_params.yaml`: `PADM_*` PX4 파라미터
- `test/test_admittance.py`: host 기반 동역학 및 상태 불변조건 테스트
- `IMPLEMENTATION_REPORT.md`: 구현 상세와 설계 결정
- `MOB_ADMITTANCE_CASCADE_BLOCK_DIAGRAM.txt`: MOB-admittance cascade 구조

다음 메시지 파일은 PX4 uORB 생성 규칙 때문에 저장소의 `msg/` 디렉터리에 유지한다.

- `msg/PalletroneAdmittanceCommand.msg`
- `msg/PalletroneAdmittanceStatus.msg`

