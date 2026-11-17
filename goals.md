# Goals

## Goal
Pelajari struktur, komponen utama, serta desain firmware proyek Line Follower uKit Explore 2: baca modul inti, EEPROM, fitur opsional, gotchas, dan identifikasi potensi isu/tunjangan untuk analisis.

## Steps
1. Baca README.md dan AGENTS.md untuk overview proyek
2. Baca LineFollowerFin.ino untuk memahami state machine dan alur utama
3. Baca config.h untuk detail konfigurasi, EEPROM layout, dan fitur opsional
4. Identifikasi modul inti (sensor, PID, penggerak, pengaman, mapper, dll.)
5. Catat gotchas kritis dan tuning considerations
6. Tentukan area potensi isu/tunjangan untuk analisis

## Verification
- [x] Baca README.md dan AGENTS.md
- [x] Baca LineFollowerFin.ino
- [x] Baca config.h
- [ ] Verifikasi pemahaman EEPROM layout dan magic value
- [ ] Verifikasi pemahaman state machine dan sub-state
- [ ] Verifikasi fitur opsional (path planner, fuzzy PID, dll.)
- [ ] Verifikasi gotchas kritis (blocking code, EEPROM template, dll.)

## Risks / Open Questions
- Apakah EEPROM magic value 123 dan 77 benar-benar validasi utama?
- Apakah forward declaration di fwd.h benar-benar diperlukan untuk template EEPROM?
- Apakah sub-state machine (SUBSTATE_INTER_CREEP, SUBSTATE_TURNING) benar-benar non-blocking?
- Apakah fitur path planner benar-benar tidak dipanggil otomatis?
- Apakah tuning via tombol bersamaan memang membutuhkan kalibrasi IMU?

## Status
active