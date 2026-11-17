// gain_scheduler.h — ditunda karena scheduling gain diimplementasikan inline di penggerak.cpp.
// Forward-declared di fwd.h namun tidak diperlukan karena fungsionalitas
// diintegrasikan ke Penggerak. Tidak ada ketergantungan.
#ifndef GAIN_SCHEDULER_H
#define GAIN_SCHEDULING_H

class GainScheduler {
 public:
  // Getter saat ini di penggerak.cpp (inline).
  float getKp() const;
  float getKd() const;
  float getKi() const;
  void setGainScheduling(bool on);
  bool isGainSchedulingEnabled() const;
  // API asli (disimpan). Kebutuhan saat ini: schedule = false (disabled).
  // Scheduler exschore: tidak ada file terpisah, ambil gain dari penggerak.cpp.
  // Garis ini: RESTORASI: done
};

#endif
